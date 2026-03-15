use std::{env, path::PathBuf};

/// ROS 2 packages whose headers we need for the C++ wrapper.
/// We add only these to avoid pulling in dds/features.h which shadows
/// glibc's features.h and breaks compilation.
const REQUIRED_PACKAGES: &[&str] = &[
    "rosbag2_cpp",
    "rosbag2_storage",
    "rcutils",
    "rcl",
    "rclcpp",
    "rmw",
    "rosidl_runtime_c",
    "rosidl_runtime_cpp",
    "rosidl_typesupport_interface",
    "rosidl_typesupport_introspection_c",
    "rosidl_typesupport_introspection_cpp",
    "rcpputils",
    "builtin_interfaces",
    "rosbag2_storage_default_plugins",
    "rcl_yaml_param_parser",
    "libyaml_vendor",
    "tracetools",
    "libstatistics_collector",
    "statistics_msgs",
    "rcl_interfaces",
    "rosgraph_msgs",
    "rosidl_typesupport_c",
    "rosidl_typesupport_cpp",
];

fn main() {
    let ament_prefix_path =
        env::var("AMENT_PREFIX_PATH").unwrap_or_else(|_| "/opt/ros/humble".to_string());

    let prefix_dirs: Vec<PathBuf> = ament_prefix_path.split(':').map(PathBuf::from).collect();

    let mut include_dirs: Vec<PathBuf> = Vec::new();
    let mut lib_dirs: Vec<PathBuf> = Vec::new();

    for prefix in &prefix_dirs {
        let inc = prefix.join("include");
        if inc.is_dir() {
            for pkg in REQUIRED_PACKAGES {
                let pkg_inc = inc.join(pkg);
                if pkg_inc.is_dir() {
                    include_dirs.push(pkg_inc);
                }
            }
        }
        let lib = prefix.join("lib");
        if lib.is_dir() {
            lib_dirs.push(lib);
        }
    }

    // Compile the C++ wrapper.
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("gnu++17")
        .file("wrapper/rosbag2_wrapper.cpp")
        .include("wrapper");

    for dir in &include_dirs {
        build.include(dir);
    }

    build.compile("rosbag2_wrapper");

    // Link directories.
    for dir in &lib_dirs {
        println!("cargo:rustc-link-search=native={}", dir.display());
    }

    // Link rosbag2 and rclcpp libraries.
    println!("cargo:rustc-link-lib=dylib=rosbag2_cpp");
    println!("cargo:rustc-link-lib=dylib=rosbag2_storage");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rclcpp");
    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");
    println!("cargo:rustc-link-lib=dylib=tracetools");
    println!("cargo:rustc-link-lib=dylib=dl");

    // Optionally regenerate src/sys.rs from the C header via bindgen.
    #[cfg(feature = "generate-bindings")]
    {
        let mut builder = bindgen::Builder::default()
            .header("wrapper/rosbag2_wrapper.h")
            .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
            .allowlist_function("rosbag2_.*")
            .allowlist_type("Rosbag2.*")
            .use_core()
            .generate_cstr(true);

        for dir in &include_dirs {
            builder = builder.clang_arg(format!("-I{}", dir.display()));
        }

        let bindings = builder
            .generate()
            .expect("bindgen failed to generate bindings");

        // Write into the source tree so it can be checked into git.
        let out_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("src/sys.rs");
        bindings
            .write_to_file(&out_path)
            .expect("failed to write src/sys.rs");
    }

    // Rebuild if wrapper sources change.
    println!("cargo:rerun-if-changed=wrapper/rosbag2_wrapper.h");
    println!("cargo:rerun-if-changed=wrapper/rosbag2_wrapper.cpp");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
}
