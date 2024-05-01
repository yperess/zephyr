load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain", "use_cpp_toolchain")
load("@rules_cc//cc:action_names.bzl", "C_COMPILE_ACTION_NAME")

"""
cpp  -x assembler-with-cpp -nostdinc \
    $BETTY_ROOT/third_party/zephyr/tests/lib/devicetree/api/app.overlay \
    -include $BETTY_ROOT/third_party/zephyr/boards/posix/native_sim/native_sim.dts \
    -I$BETTY_ROOT/third_party/zephyr/include \
    -I$BETTY_ROOT/third_party/zephyr/dts \
    -I$BETTY_ROOT/third_party/zephyr/dts/common \
    -I \
    -D__DTS__ -E \
    -o preprocessed.dts
"""

def _get_inputs(ctx):
    all_input_files = []
    all_input_files.extend(ctx.files.srcs)
    for label in ctx.attr.references:
        if type(label) == "Target" and label.files.to_list():
            all_input_files.extend(label.files.to_list())
        else:
            fail("Unsupported target kind in references:", label)
    return depset(all_input_files)

def _dts_library_impl(ctx):
    sources = ctx.files.srcs
    include_paths = ctx.attr.includes
    output_header = ctx.actions.declare_file(ctx.label.name + ".dts")
    cc_toolchain = find_cpp_toolchain(ctx)
    workspace_root = ctx.label.workspace_root
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    cxx_compiler_path = cc_common.get_tool_for_action(
        feature_configuration = feature_configuration,
        action_name = C_COMPILE_ACTION_NAME,
    )

    print("Declaring file: ", output_header.path)
    print("workspace_name: ", ctx.label.workspace_root)
    for dep in ctx.attr.deps:
        if hasattr(dep, "files"):
            sources.extend(dep.files)
        else:
            fail("Dependency", dep, "does not contain files.")

    args = [
        "-x",
        "assembler-with-cpp",
        "-nostdinc",
    ]
    source_count = 0
    for src in sources:
        if source_count != 0:
            args.append("-include")
        args.append(src.path)
        source_count += 1
    for path in include_paths:
        if path.startswith("//"):
            path = workspace_root + path.replace("//", "/")
        args.append("-I" + path)
    args.extend([
        "-I",
        "-D__DTS__",
        "-E",
        "-o",
        output_header.path,
    ])
    print("Running: ", cxx_compiler_path, " ".join(args))
    
    ctx.actions.run(
        inputs = _get_inputs(ctx),
        outputs = [output_header],
        arguments = args,
        executable = cxx_compiler_path,
        tools = cc_toolchain.all_files,
        mnemonic = "DtsPreCompile"
    )
    return [
        DefaultInfo(files = depset([output_header])),
    ]

dts_library = rule(
    implementation = _dts_library_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "references": attr.label_list(allow_files = True),
        "deps": attr.label_list(allow_files = False),
        "includes": attr.string_list(),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")
        ),
    },
    toolchains = use_cpp_toolchain(),
    fragments = ["cpp"],
)

def _dts_cc_library_impl(ctx):
    output_dts = ctx.actions.declare_file("dts_out.dts")
    output_header = ctx.actions.declare_file("devicetree_generated.h")
    dts_file = ctx.attr.dts_lib[DefaultInfo].files.to_list()[0]

    gen_defines_target = ctx.attr._gen_defines[DefaultInfo]
    
    args = [
        "--dts",
        dts_file.path,
        "--bindings-dirs",
        "external/zephyr/dts/bindings",
        "--header-out",
        output_header.path,
        "--dts-out",
        output_dts.path,
        "--dtc-flags",
        "Wno-simple_bus_reg",
    ]
    print(" ".join(args))
    ctx.actions.run(
        inputs = gen_defines_target.files.to_list() + [dts_file],
        outputs = [output_header, output_dts],
        executable = gen_defines_target.files_to_run.executable,
        arguments = args,
        mnemonic = "DtsGenDefines",
        progress_message = "Running DTS definition generator",
    )
    return [
        DefaultInfo(files = depset([output_header])),
        CcInfo(
            compilation_context = cc_common.create_compilation_context(
                includes = depset([output_header.path[:-len(output_header.basename)]]),
                headers = depset([output_header]),
            )
        ),
    ]

dts_cc_library = rule(
    implementation = _dts_cc_library_impl,
    attrs = {
        "dts_lib": attr.label(allow_files = False),
        "_gen_defines": attr.label(
            default = "//scripts/dts:gen_defines",
        ),
    },
    provides = [CcInfo],
)