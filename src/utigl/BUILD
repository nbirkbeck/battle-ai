cc_library(
    name = "glwindow",
    hdrs = ["glwindow.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "ogre_window",
    hdrs = ["ogre_window.h"],
    visibility = ["//visibility:public"],
    linkopts = [
     "-lOIS -lOgreMain -lOgrePaging -lOgreProperty -lGL -lGLU"
   ]
)

cc_library(
    name = "ffont",
    hdrs = ["ffont.h"],
    srcs = ["ffont.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@freetype2//:lib",
    ],
)
