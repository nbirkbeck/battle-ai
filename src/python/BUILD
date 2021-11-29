load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
DEPS = [
    "@nimage//:lib",
    "@nmath//:lib",
    "@nmisc//:lib",
    "@libglog//:lib",
    "@gflags//:lib",
]

pybind_extension(
    name = "battle_ai",
    srcs = ["battle_ai.cc"],
    copts = [
          "-I/usr/include/python3.7m",
          "-I/usr/include/OIS",
          "-I/usr/include/OGRE",
    ],
    linkopts = ["-lglut -lGL -lGLU -lGLEW"],
    deps = DEPS + [
        "//src:world",
        "//src/ui:simple_window",
        "//src/proto:actions_proto_cc",
        "//src/proto:level_proto_cc",
        "//src/ui/utigl:glwindow",
        "//src/ui/utigl:ogre_window",
        "//src/ui:ogre_win",
    ],
)


