DEPS = [
    "@nimage//:lib",
    "@nmath//:lib",
    "@nmisc//:lib",
    "@libglog//:lib",
    "@gflags//:lib",
]

package_group(
    name = "subpackages",
    packages = [
        "//src/...",
    ],
)

cc_library(
    name = "geometry",
    hdrs = [
        "axis_aligned_box.h",
        "bounding_cylinder.h",
        "plane.h",
    ],
    deps = DEPS,
)

cc_library(
    name = "weapon",
    hdrs = ["weapon.h"],
)

cc_library(
    name = "agent",
    srcs = [
        "power_up.cc",
        "simple_agent.cc",
        "search.cc",
    ],
    hdrs = [
        "agent.h",
        "power_up.h",
        "simple_agent.h",
        "math.h",
        "observable_state.h",
        "search.h"
    ],
    deps = DEPS + [
        ":proto",
        ":weapon",
        ":geometry",
        "//src/proto:actions_proto_cc",
        "//src/proto:simple_agent_proto_cc",
        "//src/proto:level_proto_cc",
    ],
)

cc_library(
    name = "proto",
    hdrs = ["proto.h"],
    visibility = [
      ":subpackages",
    ],
    deps = [
      "//src/proto:vec3_proto_cc",
    ] 
)

cc_library(
    name = "world",
    srcs = ["world.cc"],
    hdrs = ["world.h"],
    deps = DEPS + [
        ":geometry",
        ":agent",
        ":weapon",
        ":proto",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [":subpackages"],
)

cc_library(
    name = "world_renderer",
    hdrs = ["world_renderer.h"],
    srcs = ["world_renderer.cc"],
    deps = [
        ":world",
            "//src/utigl:ffont",        
    ],
    visibility = [":subpackages"],
)

cc_library(
    name = "simple_window",
    hdrs = ["simple_window.h"],
    deps = DEPS + [
        ":geometry",
        ":agent",
        ":weapon",
        ":proto",
        ":world_renderer",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [":subpackages"],
)

cc_binary(
    name = "validate_level",
    srcs = ["validate_level.cc"],
    deps = [
        ":proto",
        ":world",
        "//src/proto:level_proto_cc",
    ],
)

cc_binary(
    name = "ui",
    srcs = ["ui.cc"],
    copts = [
       "-I/usr/include/OIS",
    ],
    linkopts = ["-lglut -lGL -lGLU -lOIS -lOgreMain -lOgrePaging -lOgreProperty -lGL -lGLU"],
    deps = [
        ":proto",
        ":world",
        ":world_renderer",
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
        "//src/utigl:glwindow",
        "//src/utigl:ogre_window",
        "//src/utigl:ffont",
        "@freetype2//:lib",
    ],
)


cc_binary(
    name = "benchmark_simple_agents",
    srcs = ["benchmark_simple_agents.cc"],
    linkopts = ["-lglut -lGL -lGLU"],
    deps = [
        ":proto",
        ":world",
        ":world_renderer",
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
    ],
)

cc_test(
    name = "test_geometry",
    srcs = ["test_geometry.cc"],
    deps = [
        ":geometry",
        "@gtest//:lib",
    ],
)


cc_binary(
    name = "bake_ambient_occlusion",
    srcs = ["bake_ambient_occlusion.cc"],
    linkopts = ["-lglut -lGL -lGLU",
              "-L/home/birkbeck/src/cycles.build/lib"],
    deps = [
        ":proto",
        ":world",
        ":world_renderer",
        "//src/utigl:glwindow", 
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
        "@ngeotc//:lib",
        "@nmath//:lib",
        "@nimage//:lib",
        "@cycles//:lib",
    ],
)
