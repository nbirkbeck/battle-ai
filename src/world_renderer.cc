#include <GL/glew.h>
#include "src/world_renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <nmath/vec3.h>
#include <nmath/mat4x4.h>
#include <nimage/image.h>

std::deque<nacb::Vec3d> g_def;

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

WorldRenderer::WorldRenderer(const char* font_name) {
  cylinder_ = gluNewQuadric();
  if (font_name) {
    font_ = FFont(font_name, 12);
    font_.setScale(1, 1);
  }
}

void WorldRenderer::Draw(const World* world) {
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_CULL_FACE);

  SetupLighting();
  for (const auto& g: world->geoms()) {
    glColor3f(1, 1, 1);
    DrawCube(g->pos, g->size);
  }
  for (const auto& pup: world->power_ups()) {
    if (!pup->IsActive()) continue;
    glColor3f(0, 0, 1);
    DrawCube(pup->pos(), nacb::Vec3d(1, 1, 1));
  }
  for (const auto& agent: world->agents()) {
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
    font_.drawString(buf, -0.5, 0);
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
    
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    gluCylinder(cylinder_, 0.5, 0.5, 2, 32, 2);
    glPopMatrix();

    glPushMatrix();
    agent->quat().glRotate();
    glTranslatef(agent->pos().x, 0.75, agent->pos().z);
    gluCylinder(cylinder_, 0.25, 0.25, 0.5, 32, 0.5);
    glPopMatrix();

    glPopMatrix();
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
  for (int i = 0; i < g_def.size(); ++i) {
    glPushMatrix();
    glTranslatef(g_def[i].x,
                 g_def[i].y,
                 g_def[i].z);
    DrawCube(nacb::Vec3d(0, 0, 0), nacb::Vec3d(0.01, 0.01, 0.01));
    glPopMatrix();
  }
}
