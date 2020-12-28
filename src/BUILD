DEPS = [
    "@nimage//:lib",
    "@nmath//:lib",
    "@nmisc//:lib",
    "@libglog//:lib",
    "@gflags//:lib",
]

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
    ],
    hdrs = [
        "agent.h",
        "observable_state.h",
        "power_up.h",
        "simple_agent.h",
    ],
    deps = DEPS + [
        ":weapon",
        ":geometry",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
)

cc_library(
    name = "level",
    hdrs = ["level.h"],
    deps = DEPS + ["//src/proto:level_proto_cc"],
)

cc_library(
    name = "world",
    srcs = ["world.cc"],
    hdrs = ["world.h"],
    deps = DEPS + [
        ":geometry",
        ":agent",
        ":weapon",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
)

cc_binary(
    name = "validate_level",
    srcs = ["validate_level.cc"],
    deps = [
        ":level",
        ":world",
        "//src/proto:level_proto_cc",
    ],
)

cc_binary(
    name = "ui",
    srcs = ["ui.cc"],
    linkopts = ["-lglut -lGL -lGLU"],
    deps = [
        ":level",
        ":world",
        "//src/proto:level_proto_cc",
        "//src/utigl:glwindow",
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
