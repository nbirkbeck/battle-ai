import find_powerups
import os
from stable_baselines3 import PPO, A2C
from sb3_contrib import QRDQN
from stable_baselines3.common.cmd_util import make_vec_env

# Instantiate the env
filename = '/home/birkbeck/Desktop/BattleAI/levels/w1.textpb'
use_dqn = True
train = not True
save_dir = "/tmp/gym"

if use_dqn:
    env = find_powerups.FindPowerupsEnv(filename)
    env = make_vec_env(lambda: env, n_envs=1)
    model = QRDQN('MlpPolicy', env, verbose=2,
                  learning_rate = 0.001, #1e-4,
                  buffer_size=100000,
                  exploration_initial_eps=1,
                  exploration_fraction=0.7,
                  exploration_final_eps=0.5)
else:
    env = find_powerups.FindPowerupsEnv(filename)
    env = make_vec_env(lambda: env, n_envs=1)
    model = PPO('MlpPolicy', env, verbose=1)
    
if train:
    model.learn(1000000)
    os.makedirs(save_dir, exist_ok=True)
    model.save(save_dir + "/10k")
else:
    model.load(save_dir + "/10k")

obs = env.reset()
n_steps = 100000
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=False)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  if done:
      obs = env.reset()
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render(mode='human')
