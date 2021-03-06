#include "src/power_up.h"
#include "src/agent/agent.h"

void PowerUp::Give(Agent* agent) {
  switch (type_) {
  case state::PowerUp::POWER_UP_HEALTH:
    agent->AddHealth(amount_);
    break;
  case state::PowerUp::POWER_UP_ARMOR:
    agent->AddArmor(amount_);
    break;
  }
  time_until_respawn_ = respawn_delay_;
}
