#include <GL/glew.h>
#include "src/ui/world_renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <nmath/vec3.h>
#include <nmath/mat4x4.h>
#include <nimage/image.h>

// This is a global object (only used for debugging)
std::deque<nacb::Vec3d> g_def;

std::vector<std::pair<nacb::Vec3d, nacb::Vec3d> >
GetCreaseLines(const nappear::Mesh& mesh) {
  std::vector<std::pair<nacb::Vec3d, nacb::Vec3d> > segments;
  std::unordered_map<int, int> edge_face;
  for (int i = 0; i < (int)mesh.faces.size(); ++i) {
    const auto& face = mesh.faces[i];
    for (int j = 0; j < 3; ++j) {
      const int e1 = face.vi[(j + 1) % 3]  * mesh.vert.size() + face.vi[j];
      edge_face[e1] = i;
    }
  }
  for (int i = 0; i < (int)mesh.faces.size(); ++i) {
    const auto& face = mesh.faces[i];
    const auto& n1 = mesh.norm[face.ni[0]];
    for (int j = 0; j < 3; ++j) {
      const int e2 = face.vi[j % 3]  * mesh.vert.size() + face.vi[(j + 1) % 3];
      double dot = 0;
      bool has_neigh = false;
      if (edge_face.count(e2)) {
        const auto& n2 = mesh.norm[mesh.faces[edge_face[e2]].ni[0]];
        dot = n2.dot(n1);
        has_neigh = true;
      }
      if (dot < 1e-4 && (!has_neigh || face.vi[j] < face.vi[(j + 1) % 3])) {
        segments.push_back(std::make_pair(mesh.vert[face.vi[j]],
                                          mesh.vert[face.vi[(j + 1) % 3]]));
      }
    }
  }
  for (int i = 0; i < (int)segments.size(); ++i) {
    nacb::Vec3d d1 = segments[i].second - segments[i].first;
    double l1 = d1.normalize();
    segments[i].first -= d1 * (0.125 / 8);
    segments[i].second += d1 * (0.125 / 8);
    continue;
    for (int j = i + 1; j < (int)segments.size(); ++j) {
      nacb::Vec3d d2 = segments[j].second - segments[j].first;
      double l2 = d2.normalize();
      if (d1.dot(d2) < -0.99) {
        LOG(INFO) << "Found match";
        if (fabs(l1 - l2) / l1 < 1e-5) {
          segments[i].first -= d1 * 0.125;
          segments[i].second += d1 * 0.125;
          segments[j].first -= d1 * 0.125;
          segments[j].second += d1 * 0.125;
          // segments[j].second = segments[j].first + d2 * (l2 * 1.2);
          //segments[j].first = segments[j].second;
        }
        break;
      }
    }
  }
  return segments;
}

void DrawCube(const nacb::Vec3d pos, const nacb::Vec3d size, bool wire = false) {
  glPushMatrix();
  glTranslatef(pos.x, pos.y, pos.z);
  glScalef(size.x, size.y, size.z);
  if (wire) {
    glutWireCube(1);
  } else {  
    glutSolidCube(1);
  }
  glPopMatrix();
}

void SetupLighting() {
  glEnable(GL_LIGHTING);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, true);

  float pos[4] = {0, 1, 0, 0};
  float white[4] = {0.7, 0.7, 0.7, 1};
  float spec[4] = {0.1, 0.1, 0.1, 1};
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, pos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT0, GL_SPECULAR, spec);

  float pos1[4] = {1, 1, 0.1, 0};
  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_POSITION, pos1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT1, GL_SPECULAR, spec);

  float pos2[4] = {-0.1, 1, 1, 0};
  glEnable(GL_LIGHT2);
  glLightfv(GL_LIGHT2, GL_POSITION, pos2);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT2, GL_SPECULAR, spec);
}

void DrawSimpleAgent(FFont& font, GLUquadric* cylinder, const Agent* agent) {
  glPushMatrix();
  glTranslatef(agent->pos().x, 0, agent->pos().z);

  glPushMatrix();
  glEnable(GL_TEXTURE_2D);
  glTranslatef(0, 2, 0);
  nacb::Mat4x4 m = nacb::Mat4x4::modelview();
  double z = fabs(m(2, 3));
  m(0, 3) = 0;
  m(1, 3) = 0;
  m(2, 3) = 0;
  m = m.transpose();
  m.glMult();
  glScaled(0.03 * z, 0.03 * z, 0.03 * z);
  char buf[1024];
  snprintf(buf, sizeof(buf), "h/a: %g %g", agent->health(), agent->armor()); 
  font.drawString(buf, -0.5, 0);
  glDisable(GL_TEXTURE_2D);
  glPopMatrix();
    
  glPushMatrix();
  glRotatef(-90, 1, 0, 0);
  gluCylinder(cylinder, 0.5, 0.5, 2, 32, 2);
  glPopMatrix();

  glPushMatrix();
  agent->quat().glRotate();
  glTranslatef(agent->pos().x, 0.75, agent->pos().z);
  gluCylinder(cylinder, 0.25, 0.25, 0.5, 32, 0.5);
  glPopMatrix();

  glPopMatrix();
}

WorldRenderer::WorldRenderer(const char* font_name,
                             const std::string& level_obj,
                             const std::string& level_tex) {
  cylinder_ = gluNewQuadric();
  if (font_name) {
    font_ = FFont(font_name, 20);
    font_.setScale(1, 1);
  }
  if (!level_obj.empty()) {
    level_mesh_.reset(new nappear::Mesh);
    if (!level_mesh_->readObj(level_obj.c_str())) {
      level_mesh_.reset();
    } else {
      level_mesh_->initNormals(true);
      segments_ = GetCreaseLines(*level_mesh_);
    }
  }
  if (!level_tex.empty()) {
    nacb::Image8 image;
    if (image.read(level_tex.c_str())) {
      glGenTextures(1, &level_tex_);
      glBindTexture(GL_TEXTURE_2D, level_tex_);
      image.initTexture();
      glBindTexture(GL_TEXTURE_2D, 0);
    }
  }
  body_.reset(new nappear::Mesh);
  if (!body_->readObj("models/body.obj")) {
    body_.reset();
  } else {
    body_->initNormals(true);
  }
  head_.reset(new nappear::Mesh);
  if (!head_->readObj("models/head.obj")) {
    head_.reset();
  } else {
    head_->initNormals(true);
  }
  left_wheel_.reset(new nappear::Mesh);
  if (!left_wheel_->readObj("models/left_wheel.obj")) {
    left_wheel_.reset();
  }
  right_wheel_.reset(new nappear::Mesh);
  if (!right_wheel_->readObj("models/right_wheel.obj")) {
    right_wheel_.reset();
  }
}

void WorldRenderer::Draw(const World* world) {
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_CULL_FACE);

  SetupLighting();
  if (level_mesh_) {
    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, level_tex_);
    level_mesh_->draw();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    glDisable(GL_LIGHTING);
    glColor3f(0, 0, 0);
    glLineWidth(3);
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(1.04, 1.04);
    
    glBegin(GL_LINES);
    for (const auto& segment: segments_) {
      glVertex3dv(segment.first.data);
      glVertex3dv(segment.second.data);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    glLineWidth(1);
  } else {
    for (const auto& g: world->geoms()) {
      glColor3f(1, 1, 1);
      DrawCube(g->pos, g->size);
    }
  }
  for (const auto& pup: world->power_ups()) {
    if (!pup->IsActive()) continue;
    glColor3f(0, 0, 1);
    DrawCube(pup->pos(), nacb::Vec3d(1, 1, 1));
  }
  for (const auto& agent: world->agents()) {
    if (body_) {
      glColor3f(1, 1, 1);
      glPushMatrix();
      glTranslatef(agent->pos().x,
                   agent->pos().y * 0,
                   agent->pos().z);

      glPushMatrix();
      const nacb::Vec3d d = agent->pos() - agent->last_pos();
      glRotatef(180 * atan2(d.x, d.z) / M_PI, 0, 1, 0);
      body_->draw();

      // This is a total hack to approximating wheel motion.
      const double a = -agent->pos().len();
      glPushMatrix();
      glTranslatef(0, 0.5, 0.0);
      glRotatef(-180 * a / M_PI, 1, 0, 0);
      glTranslatef(0, -0.5, 0.0);
      left_wheel_->draw();
      glPopMatrix();

      glPushMatrix();
      glTranslatef(0, 0.5, 0.0);
      glRotatef(-180 * a / M_PI, 1, 0, 0);
      glTranslatef(0, -0.5, 0.0);
      right_wheel_->draw();
      glPopMatrix();
      
      glPopMatrix();

      glPushMatrix();
      nacb::Vec3d head_d = agent->quat().rotate(nacb::Vec3d(0, 0, 1));
      glRotatef(180 * atan2(head_d.x, head_d.z) / M_PI, 0, 1, 0);
      head_->draw();
      glPopMatrix();

      
      glPopMatrix();
    } else {
      DrawSimpleAgent(font_, cylinder_, agent.get());
    }
  }
  for (const auto& projectile: world->projectiles()) {
    glColor3f(1, 0, 0);

    glPushMatrix();
    glTranslatef(projectile.p().x,
                 projectile.p().y,
                 projectile.p().z);
    double theta = atan2(projectile.v().x,
                         projectile.v().z);
    glRotatef(theta * 180 / M_PI, 0, 1, 0);
    gluCylinder(cylinder_, 0.25, 0.25, 0.5, 32, 0.5);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(projectile.p().x,
                 projectile.p().y,
                 projectile.p().z);
    glRotatef(theta * 180 / M_PI, 0, 1, 0);
    DrawCube(nacb::Vec3d(0, 0, 0), nacb::Vec3d(0.1, 0.1, 1));
    glPopMatrix();
  }
  for (int i = 0; i < (int)g_def.size(); ++i) {
    glPushMatrix();
    glTranslatef(g_def[i].x,
                 g_def[i].y,
                 g_def[i].z);
    DrawCube(nacb::Vec3d(0, 0, 0), nacb::Vec3d(0.01, 0.01, 0.01));
    glPopMatrix();
  }
}
