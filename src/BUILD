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
    name = "weapon",
    hdrs = ["weapon.h"],
    visibility = [
      ":subpackages",
    ],
)    

cc_library(
    name = "power_up_hdr",
    hdrs = ["power_up.h"],
    deps = [
        "//src/proto:level_proto_cc"
    ],
    visibility = [
      ":subpackages",
    ],
)

cc_library(
    name = "power_up",
    srcs = [
       "power_up.cc"
    ],
    deps = [
      "//src/agent",
      ":power_up_hdr"
    ],
    visibility = [
      ":subpackages",
    ],
)

cc_library(
    name = "accessibility_map",
    hdrs = ["accessibility_map.h"],
    deps = [ "//src/geometry"],
    visibility = [
      ":subpackages",
    ],
)

cc_library(
    name = "observable_state",
    hdrs = ["observable_state.h"],
    deps = [
         ":weapon",
         ":accessibility_map",
         ":power_up_hdr"
    ],
    visibility = [
      ":subpackages",
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
        ":weapon",
        ":proto",
        ":power_up",
        "//src/geometry",
        "//src/agent",
        "//src/agent:simple_agent",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [":subpackages"],
)

