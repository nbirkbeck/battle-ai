proto_library(
    name = "vec3_proto",
    srcs = ["vec3.proto"],
)

proto_library(
    name = "actions_proto",
    srcs = ["actions.proto"],
    deps = ["vec3_proto"],
)

cc_proto_library(
    name = "actions_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        "actions_proto",
    ],
)

proto_library(
    name = "level_proto",
    srcs = ["level.proto"],
    deps = ["vec3_proto"],
)

cc_proto_library(
    name = "level_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        "level_proto",
    ],
)
