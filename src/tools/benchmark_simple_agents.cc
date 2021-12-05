#include "src/world.h"
#include "src/proto.h"
#include "src/agent/simple_agent.h"
#include "src/proto/agent_list.pb.h"
#include "src/proto/level.pb.h"


#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(level_filename, "", "Path to input level filename");
DEFINE_string(agent_filename, "", "Config path for agents");
DEFINE_int32(num_trials, 100, "Number of trials per comparison");

double sign(double x) {
  if (x == 0) return 0;
  return x < 0 ? -1 : 1;
}

// This simulates a deathmatch game type.
std::pair<double, double> RunTrial(const state::Level& level,
                                   const battle::SimpleAgentParams& p1,
                                   const battle::SimpleAgentParams& p2) {
  World world;
  if (!world.LoadFromProto(level)) {
    LOG(ERROR) << "Error loading level from:" << FLAGS_level_filename;
    return {-1, -1};
  }

  std::vector<Weapon> weapons;
  weapons.push_back({});

  SimpleAgent* a1 = new SimpleAgent({}, {}, weapons);
  SimpleAgent* a2 = new SimpleAgent({}, {}, weapons);
  a1->SetParams(p1);
  a2->SetParams(p2);
  world.ReplaceAgent(0, a1);
  world.ReplaceAgent(1, a2);

  const double kFragLimit = 8;
  const double kMaxWorldTime = 30 * kFragLimit;
  const double dt = 1.0/10;
  int num_deaths[2] = {0, 0};
  while (world.world_time() < kMaxWorldTime &&
         std::max(num_deaths[0], num_deaths[1]) < kFragLimit) {
    world.Step(dt);
    if (a1->health() <= 0) {
      num_deaths[0]++;
      world.ResetAgent(0);
    }
    if (a2->health() <= 0) {
      num_deaths[1]++;
      world.ResetAgent(1);
    }
  }
  const double relative_time = world.world_time() / kMaxWorldTime;
  if (num_deaths[0] >= kFragLimit &&
      num_deaths[1] >= kFragLimit) {
    return {0, relative_time};
  } else if (num_deaths[0] >= kFragLimit) {
    return {-1, relative_time};
  } else if (num_deaths[1] >= kFragLimit) {
    return {1, relative_time};
  }
  return {sign(num_deaths[1] - num_deaths[0]), relative_time};
}

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  state::Level level;
  if (!Proto::ReadProto(FLAGS_level_filename, &level)) {
    LOG(ERROR) << "Unable to load level from :" << FLAGS_level_filename;
    return -1;
  }

  battle::AgentList config;
  if (!Proto::ReadProto(FLAGS_agent_filename, &config)) {
    LOG(ERROR) << "Unable to agents from :" << FLAGS_agent_filename;
    return -1;
  }
  for (int i = 0; i < config.agents_size(); ++i) {
    for (int j = i + 1; j < config.agents_size(); ++j) {
      const auto& agent1 = config.agents(i);
      const auto& agent2 = config.agents(j);
      std::cout << agent1.name() << " vs " << agent2.name() << ": ";
      double avg = 0;
      for (int i = 0; i < FLAGS_num_trials; ++i) {
        auto result = RunTrial(level, agent1.params(), agent2.params());
        avg += result.first;
      }
      avg /= FLAGS_num_trials;
      std::cout << avg << std::endl;
    }
  }

  return 0;
}
