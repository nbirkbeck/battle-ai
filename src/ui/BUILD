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
    name = "world_renderer",
    hdrs = ["world_renderer.h"],
    srcs = ["world_renderer.cc"],
    deps = [
        "//src:world",
        "//src/ui/utigl:ffont",        
    ],
    visibility = [":subpackages"],
)

cc_library(
    name = "simple_window",
    hdrs = ["simple_window.h"],
    deps = DEPS + [
        "//src/geometry",
        "//src/agent",
        "//src:weapon",
        "//src:proto",
        ":world_renderer",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [":subpackages"],
)

cc_library(
    name = "ogre_win",
    hdrs = ["ogre_win.h"],
    copts = [
       "-I/usr/include/OGRE",
    ],
    deps = DEPS + [
        "//src/ui/utigl:ogre_window", 
        "//src/geometry",
        "//src/agent:user_agent",
        "//src/agent",
        "//src:weapon",
        "//src:proto",
        ":world_renderer",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
    ],
    visibility = [":subpackages"],
)


cc_binary(
    name = "ui",
    srcs = ["ui.cc"],
    copts = [
       "-I/usr/include/OIS",
       "-I/usr/include/OGRE",
    ],
    linkopts = ["-lglut -lGL -lGLU -lOIS -lOgreMain -lOgrePaging -lOgreProperty -lGL -lGLU"],
    deps = [
        "//src:proto",
        "//src:world",
        ":world_renderer",
        ":ogre_win",
        "//src/proto:level_proto_cc",
        "//src/proto:agent_list_proto_cc",
        "//src/ui/utigl:glwindow",
        "//src/ui/utigl:ogre_window",
        "//src/ui/utigl:ffont",
        "@freetype2//:lib",
    ],
)
