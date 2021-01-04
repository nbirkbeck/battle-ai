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

pkg_config(
    name = "freetype2",
)

pkg_config(
    name = "cycles",
)

pkg_config(
    name = "ngeotc",
)

http_archive(
  name = "pybind11_bazel",
  strip_prefix = "pybind11_bazel-26973c0ff320cb4b39e45bc3e4297b82bc3a6c09",
  urls = ["https://github.com/pybind/pybind11_bazel/archive/26973c0ff320cb4b39e45bc3e4297b82bc3a6c09.zip"],
)

# We still require the pybind library.
http_archive(
  name = "pybind11",
  build_file = "@pybind11_bazel//:pybind11.BUILD",
  strip_prefix = "pybind11-2.6.1",
  urls = ["https://github.com/pybind/pybind11/archive/v2.6.1.tar.gz"],
)

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python")