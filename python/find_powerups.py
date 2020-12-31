import battle_ai
import numpy as np
import gym
import math
from gym import spaces

class FindPowerupsEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, level_name):
    super(FindPowerupsEnv, self).__init__()

    self.world = battle_ai.World()
    if not self.world.load_from_file(level_name):
      raise 'unable to load file'

    self.agent = battle_ai.DiscreteAgent()
    battle_ai.replace_agent(self.world, 0, self.agent)
    pup_times = battle_ai.powerup_times(self.world)
    
    # For simplicity, assume we can move in one of 8-directions.
    n_actions = 8
    self.action_space = spaces.Discrete(n_actions)
    self.observation_space = spaces.Box(low=-1, high=1, shape=(4 + len(pup_times),), dtype=np.float32)
    self.window = None
    self.current_step = 0

  def _get_state(self):
    pup_times = battle_ai.powerup_times(self.world)
    return np.array([p / 10 for p in self.agent.pos2d()] +
                    [self.agent.health() / 200, self.agent.armor() / 200] + 
                    [p / 60.0 for p in pup_times]).astype(np.float32)

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    # Initialize agent to be the 0-th spawn point
    self.world.reset()
    self.current_step = 0
    return self._get_state()

  def step(self, action):
    n = math.sqrt(2.0)
    directions = [[0, 1.0],
                  [1.0/n, 1.0/n],
                  [1.0, 0],
                  [1.0/n, -1.0/n],
                  [0, -1.0],
                  [-1.0/n, -1.0/n],
                  [-1.0, 0],
                  [-1.0/n, 1.0/n]]
    d = directions[action]

    # Update action to MOVE in direction d
    health_before = self.agent.health()
    armor_before = self.agent.armor()
    
    self.agent.clear_actions()
    self.agent.move_direction(d)
    self.world.step(1.0 / 8.0)

    moved = self.agent.moved_distance()

    # Reward proportional to maximum velocity moved
    reward = 0 * moved / 20.0
    if moved < 1e-6:
      reward -= 1
      self.agent.do_damage(1)
    #reward += self.agent.powerup_dist(self.world)
    self.current_step += 1

    near = self.agent.near_powerup(self.world)
    done = (self.current_step >= 800) or (self.agent.health() <= 0)
    reward += (self.agent.health() - health_before) + (self.agent.armor() - armor_before)
    if near and False:
      print('Near: %d (%f, %f)!' % (self.current_step, self.agent.health(), self.agent.armor()))
      print(battle_ai.powerup_times(self.world))
      # reward += 1
      # self.render('human')

    # Optionally we can pass additional info, we are not using that for now
    info = {'moved': moved}
    pup_times = battle_ai.powerup_times(self.world)
    return self._get_state(), reward, done, info

  def render(self, mode='console'):
    if mode != 'human':
      raise NotImplementedError()
    if not self.window:
      self.window = battle_ai.SimpleWindow()
      self.window.set_world(self.world)

    self.window.refresh()
    self.window.process_events()
  
  def close(self):
    pass



class FindPowerupsGridEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, level_name):
    super(FindPowerupsEnv, self).__init__()

    self.world = battle_ai.World()
    if not self.world.load_from_file(level_name):
      raise 'unable to load file'

    self.agent = battle_ai.DiscreteAgent()
    battle_ai.replace_agent(self.world, 0, self.agent)

    access_map = get_map(self.world, self.agent)
    w = access_map[-2]
    h = access_map[-1]
    # For simplicity, assume we can move in one of 8-directions.
    n_actions = 8
    self.action_space = spaces.Discrete(n_actions)
    self.observation_space = spaces.Box(low=-1,
                                        high=1,
                                        shape=(h,w), dtype=np.uint8)
    self.window = None
    self.current_step = 0
    
  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    # Initialize agent to be the 0-th spawn point
    self.world.reset()
    self.current_step = 0
    access_map = get_map(self.world, self.agent)
    return np.array(access_map[0:-2]).astype(np.uint8)

  def step(self, action):
    n = math.sqrt(2.0)
    directions = [[0, 1.0],
                  [1.0/n, 1.0/n],
                  [1.0, 0],
                  [1.0/n, -1.0/n],
                  [0, -1.0],
                  [-1.0/n, -1.0/n],
                  [-1.0, 0],
                  [-1.0/n, 1.0/n]]
    d = directions[action]

    # Update action to MOVE in direction d
    health_before = self.agent.health_before()
    armor_before = self.agent.armor_before()

    self.agent.clear_actions()
    self.agent.move_direction(d)
    self.world.step(1.0 / 8.0)
    self.current_step += 1

    near = self.agent.near_powerup(self.world)
    done = near or (self.current_step >= 200)
    reward += (self.agent.health() - health_before) + (self.agent.armor() - armor_before)

    # Optionally we can pass additional info, we are not using that for now
    info = {'moved': moved}
    access_map = get_map(self.world, self.agent)
    return np.array(access_map[0:-2]).astype(np.uint8), reward, done, info

  def render(self, mode='console'):
    if mode != 'human':
      raise NotImplementedError()
    if not self.window:
      self.window = battle_ai.SimpleWindow()
      self.window.set_world(self.world)

    self.window.refresh()
    self.window.process_events()
  
  def close(self):
    pass
    
