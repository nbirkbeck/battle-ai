#ifndef _WORLD_RENDERER_H_
#define _WORLD_RENDERER_H_

#include "world.h"
#include "src/utigl/ffont.h"

class GLUquadric;

class WorldRenderer {
 public:
  WorldRenderer(const char* font_name);
  void Draw(const World* world);
  GLUquadric* cylinder_;
  FFont font_;
};

#endif
