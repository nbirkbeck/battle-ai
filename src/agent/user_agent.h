#ifndef USER_AGENT_
#define USER_AGENT_
#include "agent.h"

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

#endif 
