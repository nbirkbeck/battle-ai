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
    visibility = [
      ":subpackages",
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
