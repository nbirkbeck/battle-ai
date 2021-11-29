cc_binary(
    name = "validate_level",
    srcs = ["validate_level.cc"],
    deps = [
        "//src:proto",
        "//src:world",
        "//src/proto:level_proto_cc",
    ],
)

cc_binary(
    name = "benchmark_simple_agents",
    srcs = ["benchmark_simple_agents.cc"],
    linkopts = ["-lglut -lGL -lGLU"],
    deps = [
        "//src:power_up",
        "//src/agent:simple_agent",
        "//src:proto",
        "//src:world",
        "//src/ui:world_renderer",
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
    ],
)

cc_binary(
    name = "bake_ambient_occlusion",
    srcs = ["bake_ambient_occlusion.cc"],
    linkopts = ["-lglut -lGL -lGLU",
              "-L/home/birkbeck/src/cycles.build/lib"],
    deps = [
        "//src:proto",
        "//src:world",
        "//src/ui:world_renderer",
        "//src/ui/utigl:glwindow", 
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
        "@ngeotc//:lib",
        "@nmath//:lib",
        "@nimage//:lib",
        "@cycles//:lib",
    ],
)
