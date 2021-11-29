#ifndef _SIMPLE_WINDOW_H_
#define _SIMPLE_WINDOW_H_ 1

#include <GL/glew.h>
#include <GL/gl.h>

#include "src/world.h"
#include "src/ui/utigl/glwindow.h"
#include "src/ui/world_renderer.h"

#include <GL/freeglut_ext.h>

class SimpleWindow : public GLWindow {
public:
  SimpleWindow() : GLWindow(1920/2, 1080/2) {
    world_ = 0;
    setRefreshRate(60);

    cpos = nacb::Vec3d(0, 30, 0);
    cquat = nacb::Quaternion::rod(nacb::Vec3d(-M_PI/2, 0, 0));
    glewInit();
  }
  ~SimpleWindow() {
    fprintf(stderr, "Deleting window\n");
    world_ = 0;
  }

  void drawScene() override {
    if (world_) {
      world_renderer_.Draw(world_);
    }
  }

  void SetWorld(World* world) {
    LOG(INFO) << "Setting world";
    world_ = world;
    ProcessEvents();
  }

  void ProcessEvents() {
    try {
      glutMainLoopEvent();
    } catch(const char* e) {
      LOG(ERROR) << e;
    }
  }

 protected:
  WorldRenderer world_renderer_;
  World* world_;
};

#endif
