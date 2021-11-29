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
  name = "user_agent",
  hdrs = ["user_agent.h"],
  visibility = [
    ":subpackages",
  ],
)

cc_library(
  name = "search",
  hdrs = ["search.h"],
  srcs = ["search.cc"],
  deps = [
    "//src:accessibility_map",
    "//src:observable_state"
  ],
  visibility = [
    ":subpackages",
  ],
)

cc_library(
    name = "agent",
    hdrs = [
        "agent.h",
        "math.h",
    ],
    deps = DEPS + [
        "//src:power_up_hdr",
        "//src:proto",
        "//src:weapon",
        "//src:observable_state",
        "//src/geometry",
        "//src/proto:actions_proto_cc",
        "//src/proto:simple_agent_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [
      ":subpackages",
    ],
)

cc_library(
    name = "simple_agent",
    srcs = [
        "simple_agent.cc",
    ],
    hdrs = [
        "simple_agent.h",
    ],
    deps = DEPS + [
       ":search",
       ":agent",
        "//src:proto",
        "//src:weapon",
        "//src:power_up_hdr",
        "//src:observable_state",
        "//src/geometry",
        "//src/proto:actions_proto_cc",
        "//src/proto:simple_agent_proto_cc",
        "//src/proto:level_proto_cc",
    ],
     visibility = [
      ":subpackages",
    ],
)
