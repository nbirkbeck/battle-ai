load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
     name = "com_google_protobuf",
     strip_prefix = "protobuf-master",
     urls = ["https://github.com/protocolbuffers/protobuf/archive/master.zip"],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()


http_archive(
    name = "bazel_pkg_config",
    strip_prefix = "bazel_pkg_config-master",
    urls = ["https://github.com/cherrry/bazel_pkg_config/archive/master.zip"],
)

load("@bazel_pkg_config//:pkg_config.bzl", "pkg_config")

pkg_config(
    name = "freetype2",
)

pkg_config(
    name = "nimage",
)

pkg_config(
    name = "nmath",
)

pkg_config(
    name = "nmisc",
)

pkg_config(
    name = "libglog",
)

pkg_config(
    name = "gflags",
)

pkg_config(
    name = "gtest",
)
