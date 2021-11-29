import battle_ai
import numpy as np
import gym
import math
import time
from gym import spaces

class HighLevelEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, level_name):
    super(HighLevelEnv, self).__init__()

    self.world = battle_ai.World()
    if not self.world.load_from_file(level_name):
      raise 'unable to load file'

    self.agent = battle_ai.HighLevelAgent()
    self.other_agent = battle_ai.SimpleAgent()
    battle_ai.replace_agent(self.world, 0, self.agent)
    battle_ai.replace_agent_simple(self.world, 1, self.other_agent)

    pup_times = battle_ai.powerup_times(self.world)
    obs = self._get_state()
    
    # For simplicity, assume we can move in one of 4-directions or towards a powerup
    n_actions = 4 + len(pup_times)
    self.action_space = spaces.Discrete(n_actions)
    self.observation_space = spaces.Box(low=-1, high=1, shape=(len(obs),), dtype=np.float32)
    self.window = None
    self.current_step = 0
    self.render_all = False
    self.level_name = level_name
  
  def _get_state(self):
    return self.agent.observe(self.world)

  def reset(self):
    # Initialize agent to be the 0-th spawn point
    self.world.reset()
    self.agent.reset()
    self.current_step = 0
    self.num_kills = 0
    self.num_deaths = 0
    return self._get_state()

  def step(self, action):
    # Update action to MOVE in direction d
    health_before = self.agent.health()
    armor_before = self.agent.armor()

    other_health_before = self.other_agent.health()
    other_armor_before = self.other_agent.armor()

    last_action = self.agent.last_action()
    self.agent.set_action(action)

    # Move half a second in world time.
    for i in range(0, 15):
      self.world.step(1.0 / 30.0)
      if self.render_all:
        self.render('human', delay=1.0/90.0)

    moved = self.agent.moved_distance()
    reward_weights = {
      "no_motion": 1e-3,
      "health": 1.0/10000*0,
      "armor": 1.0/10000*0,
      "other_health": 1.0/10000*0,
      "other_armor": 1.0/10000*0,
      "kill": 1,
      "die": -0.25*0,
      "change_plan": 0
    }

    # Reward proportional to maximum velocity moved
    reward = 0

    # Small negative reward for not moving
    if moved < 1e-6:
      reward -= reward_weights['no_motion']

    # Small reward for getting health/armor?
    health_delta = self.agent.health() - health_before
    armor_delta = self.agent.armor() - armor_before
    reward += health_delta * reward_weights['health']
    reward += armor_delta * reward_weights['armor']

    other_health_delta = self.other_agent.health() - other_health_before
    other_armor_delta = self.other_agent.armor() - other_armor_before
    if other_health_delta < 0:
      reward -= other_health_delta * reward_weights['other_health']
    if other_armor_delta < 0:
      reward -= other_armor_delta * reward_weights['other_armor']
    
    # Small negative reward for changing plan.
    if (last_action >= 4) and (last_action != action):
      reward -= reward_weights['change_plan']

    # Big reward for killing opponent
    if self.agent.health() < 0:
      reward += reward_weights['die']
      print('died')
      self.num_deaths += 1
      self.world.reset_agent(0)
      
    if self.other_agent.health() < 0:
      reward += reward_weights['kill']
      print('killed other')
      self.num_kills += 1
      self.world.reset_agent(1)
      
    self.current_step += 1

    time_limit = 800
    done = (self.current_step >= time_limit)

    info = {'moved': moved}
    return self._get_state(), reward, done, info

  def render(self, mode='console', delay=0):
    if mode != 'human':
      raise NotImplementedError()
    if not self.window:
      if False:
        self.window = battle_ai.SimpleWindow()
      else:
        self.window = battle_ai.OgreWin(self.level_name + ".mesh")
      self.window.set_world(self.world)

    self.window.set_info_string('Kills: %d Deaths: %d' % (self.num_kills, self.num_deaths))
    self.window.refresh()
    for i in range(0, 10):
      self.window.process_events()
    self.window.draw_scene()
    if delay > 0:
      time.sleep(delay)

  
  def close(self):
    pass

