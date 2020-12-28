#include "utigl/glwindow.h"
#include "world.h"
#include "level.h"
#include "observable_state.h"
#include "src/proto/level.pb.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <nimage/image.h>
#include <GL/glu.h>

DEFINE_string(filename, "", "Path to input filename");

class UserAgent : public Agent {
public:
  UserAgent(const nacb::Vec3d& pos,
            const nacb::Quaternion& quat,
            const std::vector<Weapon> weapons) :
    Agent(pos, quat, std::move(weapons)) {
    target_quat_ = quat;
    for (int i = 0; i < 256; ++i) {
      key_down_[i] = 0;
    }
  }

  void Rotate(double dx) {
    target_quat_ = nacb::Quaternion::rod(nacb::Vec3d(0, dx, 0)) * target_quat_;
    LOG(INFO) << target_quat_.v << " " << target_quat_.a;    
  }
  bool KeyboardDown(unsigned char c, int x, int y) {
    key_down_[(int)c] = 1;
    return true;
  }
  bool KeyboardUp(unsigned char c, int x, int y) {
    key_down_[(int)c] = 0;
    return true;
  }

  void GetActions(const ObservableState& observable_state,
                  battle::Actions* actions) override {
    nacb::Vec3d dir(0, 0, 0);
    if (key_down_['w']) {
      dir.z = 1;
    }
    if (key_down_['s']) {
      dir.z = -1;
    }
    if (key_down_['a']) {
      dir.x = 1;
    }
    if (key_down_['d']) {
      dir.x = -1;
    }
    if (key_down_[' ']) {
      actions->mutable_shoot()->set_shoot(true);
    }
    auto* target = actions->mutable_move()->mutable_target();
    dir = target_quat_.rotateNormal(dir);
    target->set_x(pos().x + dir.x);
    target->set_y(pos().y + dir.y);
    target->set_z(pos().z + dir.z);

    auto* v = actions->mutable_rotate()->mutable_v();
    v->set_x(target_quat_.v.x);
    v->set_y(target_quat_.v.y);
    v->set_z(target_quat_.v.z);
    actions->mutable_rotate()->set_a(target_quat_.a);
  }
  std::array<bool, 256> key_down_;
  nacb::Quaternion target_quat_;
};

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

UserAgent* user_agent = nullptr;
Agent* CreateAgent(int i, const nacb::Vec3d& pos,
                   const nacb::Quaternion& quat,
                   std::vector<Weapon> weapons) {
  if (i == 0) {
    return user_agent = new UserAgent(pos, quat, weapons);
  }
  return new SimpleAgent(pos, quat, weapons);
}

std::deque<nacb::Vec3d> g_def;

class Win : public GLWindow {
public:
  Win(const state::Level& level) : GLWindow(1920/2, 1080/2) {

    if (!world_.LoadFromProto(level, CreateAgent)) {
      LOG(ERROR) << "Error loading level.";
      exit(1);
    }

    cylinder_ = gluNewQuadric();
    setRefreshRate(60);
  }

  void drawScene() override {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    
    SetupLighting();
    for (const auto& g: world_.geoms()) {
      glColor3f(1, 1, 1);
      DrawCube(g->pos, g->size);
    }
    for (const auto& pup: world_.power_ups()) {
      if (!pup->IsActive()) continue;
      glColor3f(0, 0, 1);
      DrawCube(pup->pos(), nacb::Vec3d(1, 1, 1));
    }
    for (const auto& agent: world_.agents()) {
      glPushMatrix();
      glTranslatef(agent->pos().x, 0, agent->pos().z);
      glRotatef(-90, 1, 0, 0);
      gluCylinder(cylinder_, 0.5, 0.5, 2, 32, 2);
      glPopMatrix();

      glPushMatrix();
      glTranslatef(agent->pos().x, 0.5, agent->pos().z);
      agent->quat().glRotate();
      gluCylinder(cylinder_, 0.25, 0.25, 0.5, 32, 0.5);
      glPopMatrix();
    }
    for (const auto& projectile: world_.projectiles()) {
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
    if (animating_) {
      world_.Step(1.0/60);
    }
  }
  virtual void applyModelview(){
    if (animating_) {
      const auto& pos = user_agent->pos();
      const auto quat = user_agent->quat();
      glRotatef(10, 1, 0, 0);
      glTranslatef(0, -2.5, -5);
      glRotatef(180, 0, 1, 0);
      quat.conj().glRotate();
      glTranslatef(-pos.x, -pos.y, -pos.z);
    } else {
      GLWindow::applyModelview();
    }
  }

  void motion(int x, int y) {
    if (bdown == 1) {
      double dx = x - mpos[0], dy = y - mpos[1];
      dx /= height();
      dy /= height();
      user_agent->Rotate(-dx);
    }
    return GLWindow::motion(x, y);
  }
  virtual bool keyboardUp(unsigned char c, int x, int y) override {
    return user_agent->KeyboardUp(c, x, y);
  }
  virtual bool keyboard(unsigned char c, int x, int y) override{
    switch (c) {
    case '0':
      animating_ = !animating_;
      break;
    }
    return user_agent->KeyboardDown(c, x, y);
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

protected:
  bool animating_ = false;
  GLUquadric* cylinder_;
  World world_;
};

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);

  google::InitGoogleLogging(av[0]);

  state::Level level;
  if (!Level::ReadProto(FLAGS_filename, &level)) {
    LOG(ERROR) << "Error loading level.";
    return -1;
  }
  Win win(level);
  win.loop(true);
  return 0;
}
