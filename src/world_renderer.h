#ifndef _WORLD_RENDERER_H_
#define _WORLD_RENDERER_H_

#include <GL/glew.h>
#include <GL/gl.h>

#include "world.h"
#include "src/utigl/ffont.h"
#include <memory>
#include <nappear/mesh.h>

class GLUquadric;

class WorldRenderer {
 public:
  WorldRenderer(const char* font_name = "",
                const std::string& level_obj = "",
                const std::string& level_texture = "");
  void Draw(const World* world);

  std::unique_ptr<nappear::Mesh> level_mesh_;
  std::unique_ptr<nappear::Mesh> body_;
  std::unique_ptr<nappear::Mesh> head_;
  std::unique_ptr<nappear::Mesh> left_wheel_;
  std::unique_ptr<nappear::Mesh> right_wheel_;
  std::vector<std::pair<nacb::Vec3d, nacb::Vec3d> > segments_;

  GLuint level_tex_ = 0;
  GLUquadric* cylinder_;
  FFont font_;
};

#endif
