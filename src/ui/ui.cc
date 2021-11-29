#include <GL/glew.h>

#include "src/ui/utigl/glwindow.h"
#include "src/ui/ogre_win.h"
#include "src/agent/user_agent.h"
#include "src/agent/simple_agent.h"
#include "src/world.h"
#include "src/ui/world_renderer.h"
#include "src/proto.h"
#include "src/observable_state.h"
#include "src/proto/level.pb.h"
#include "src/proto/agent_list.pb.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <nimage/image.h>
#include <GL/glu.h>
#include "src/ui/utigl/ffont.h"

DEFINE_string(filename, "", "Path to input filename");
DEFINE_string(agent_filename, "", "Path to input filename");
DEFINE_bool(user_agent, true, "Allow user to play");
DEFINE_bool(use_ogre, true, "Whether to use OGRE or not");



UserAgent* user_agent = nullptr;
Agent* CreateAgent(int i, const nacb::Vec3d& pos,
                   const nacb::Quaternion& quat,
                   std::vector<Weapon> weapons) {
  if (i == 0) {
    user_agent = new UserAgent(pos, quat, weapons);
    if (FLAGS_user_agent) return user_agent;
  }
  SimpleAgent* simple_agent = new SimpleAgent(pos, quat, weapons);
  if (!FLAGS_agent_filename.empty()) {
    battle::AgentList agents;
    Proto::ReadProto(FLAGS_agent_filename, &agents);
    if (i < agents.agents_size()) {
      LOG(INFO) << "Using agent:" << agents.agents(i).name();
      simple_agent->SetParams(agents.agents(i).params());
    }
  }
  return simple_agent;
}


class Win : public GLWindow {
public:
  Win(const std::string& level_mesh,
      const std::string& level_texture) : GLWindow(1920/2, 1080/2) {

    glEnable(GL_MULTISAMPLE);
    glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

    world_renderer_.reset(new WorldRenderer("/usr/share/fonts/bitstream-vera/Vera.ttf",
                                            level_mesh, level_texture));
    setRefreshRate(60);

    glutMainLoopEvent();
  }

  void SetWorld(World* world) {
    world_ = world;
  }

  void drawScene() override {
    if (!world_) return;
    if (world_) {
      world_renderer_->Draw(world_);
    }
    if (animating_) {
      world_->Step(1.0/60);
    }
    /*
      DEbug collisions.
    nacb::Vec3d up(0, 1, 0);
    for (int i = 0; i < 360; i += 1) {
      nacb::Vec3d d =
        nacb::Quaternion::rod(nacb::Vec3d(0, 1 * double(i) / 360 * 2.0 * M_PI, 0)).rotate(nacb::Vec3d(0, 0, 1));
      nacb::Vec3d p2 = user_agent->pos() + d * 50;
      std::vector<const Agent*> agents;
      Agent* agent;
      double t = 0;
      nacb::Vec3d p1 = user_agent->pos();
      LineHitsAnything(p1, p2, world_->geoms(), agents, &t, &agent); 
      glBegin(GL_LINES);
      glVertex3dv(p1.data);
      glVertex3dv((p1 + (p2 - p1) * t).data);
      glEnd();
    }

    return;
    */
    auto state = world_->GetObservableStateForAgent(user_agent);
    if (!state.visible_agents.empty() && 0) {
      LOG(INFO) << state.visible_agents[0].pos << " "
                << state.visible_agents[0].confidence;
      glBegin(GL_LINES);
      glVertex3dv(user_agent->pos().data);
      glVertex3dv(state.visible_agents[0].pos.data);
      glEnd();

      glBegin(GL_LINES);
      for (const auto& line :  state.visible_agents[0].lines) {
        glVertex3dv(line.first.data);
        glVertex3dv(line.second.data);
      }
      glEnd();
    }
  }
  
  virtual void applyModelview(){
    if (0 && animating_) {
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

  bool keyboardUp(unsigned char c, int x, int y) override {
    return user_agent->KeyboardUp(c, x, y);
  }

  bool keyboard(unsigned char c, int x, int y) override{
    switch (c) {
    case '0':
      animating_ = !animating_;
      break;
    }
    return user_agent->KeyboardDown(c, x, y);
  }

protected:
  std::unique_ptr<WorldRenderer> world_renderer_;
  bool animating_ = false;
  World* world_ = nullptr;
};


int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);

  google::InitGoogleLogging(av[0]);

  state::Level level;
  if (!Proto::ReadProto(FLAGS_filename, &level)) {
    LOG(ERROR) << "Error loading level.";
    return -1;
  }
  World world;
  if (!world.LoadFromProto(level, CreateAgent)) {
    LOG(ERROR) << "Error loading level.";
    exit(1);
  }

  if (FLAGS_use_ogre) {
    const std::string level_mesh = FLAGS_filename + ".mesh";
    OgreWin win(level_mesh);
    win.SetWorld(&world);
    if (user_agent) {
      win.SetUserAgent(user_agent);
    }
    win.loop();
  } else {
    const std::string level_obj = FLAGS_filename + ".obj";
    const std::string level_png = FLAGS_filename + ".png";
    Win win(level_obj, level_png);
    win.SetWorld(&world);
    win.loop(true);
  }
  return 0;
}
