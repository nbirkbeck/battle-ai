import battle_ai
import find_powerups
import os
import sys
import time
import argparse
import numpy as np
from PIL import Image

from stable_baselines3 import PPO, A2C
from sb3_contrib import QRDQN
from stable_baselines3.common.cmd_util import make_vec_env

parser = argparse.ArgumentParser(description='Train to find powerups.')
parser.add_argument('--num_train_its', type=int,
                    help='Number of training iterations', default=10000)
parser.add_argument('--model', type=str, help='DQN or PPO', default='DQN')
parser.add_argument('--filename', type=str, help='Path to world',
                    default='../levels/w1.textpb')
parser.add_argument('--save_dir', type=str, help='Director to save state',
                    default='/tmp/gym')
parser.add_argument('--train', type=int, help='Whether to train or not',
                    default=0)
parser.add_argument('--heat_map', type=int, help='Whether to output the heat map',
                    default=0)
args = parser.parse_args(sys.argv[1:])
print(args)

if args.model == 'DQN':
    env_raw = find_powerups.FindPowerupsEnv(args.filename)
    env = make_vec_env(lambda: env_raw, n_envs=1)
    model = QRDQN('MlpPolicy', env, verbose=2,
                  learning_rate = 0.00035,
                  buffer_size=1000000,
                  exploration_initial_eps=1,
                  exploration_fraction=0.95,
                  exploration_final_eps=0.2)
else:
    env_raw = find_powerups.FindPowerupsEnv(args.filename)
    env = make_vec_env(lambda: env_raw, n_envs=1)
    model = PPO('MlpPolicy', env, verbose=1)
    
if args.train:
    model.learn(args.num_train_its)
    os.makedirs(args.save_dir, exist_ok=True)
    model.save(args.save_dir + "/10k")
    if args.model == 'DQN':
        model.save_replay_buffer(args.save_dir + "/replay_buffer")
else:
    if args.model == 'DQN':
        model = QRDQN.load(args.save_dir + "/10k")
        model.load_replay_buffer(args.save_dir + "/replay_buffer")
    else:
        model.load(args.save_dir + "/10k")
#print(model)
#sys.exit(1)

WORLD_SIZE = 10
access_map = env_raw.world.build_accessibility_map(30)
heat_map = np.zeros((access_map.image().height(), access_map.image().width(), 3))
im_color = battle_ai.Image8(access_map.image().height(), access_map.image().width(), 3)
HEAT_MAP_SIZE = access_map.image().height()

obs = env.reset()
n_steps = 100000
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=True)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  pos = env_raw.agent.pos2d()
  if args.heat_map:
    x = np.floor(HEAT_MAP_SIZE * (pos[0] + WORLD_SIZE) / (2 * WORLD_SIZE))
    y = np.floor(HEAT_MAP_SIZE * (pos[1] + WORLD_SIZE) / (2 * WORLD_SIZE))
    sp = 8
    for xi in range(max(0, int(np.floor(x)) - sp), min(HEAT_MAP_SIZE - 1, int(np.floor(x)) + sp + 1)):
        for yi in range(max(0, int(np.floor(y)) - sp), min(HEAT_MAP_SIZE - 1, int(np.floor(y)) + sp + 1)):
            r = (np.power(float(x - xi) / sp, 2) + np.power(float(y - yi) / sp, 2))
            a = np.exp(-r / (0.5*0.5))
            heat_map[yi, xi, 0] = np.power(step + 1, 0.75) * a + (1 - a) * heat_map[yi, xi, 0]
            heat_map[yi, xi, 1] = (step + 1)
    if (step % args.heat_map == 0):
        im = np.power(heat_map[:, :, 0], 0.25) #* (heat_map[:, :, 1] / (step + 1)), 0.1)
        im = 255 * im / np.max(im)
        for y in range(0, heat_map.shape[0]):
            for x in range(0, heat_map.shape[1]):
                im_color.set(x, y, 0, int(im[y, x]))
                im_color.set(x, y, 1, int((255 - access_map.image().get(x, y, 0)) / 2))
                im_color.set(x, y, 2, int((255 - access_map.image().get(x, y, 0)) / 2))
        im_color.save('/tmp/heat_map_%05d.png' % step)

  if done:
      obs = env.reset()
      if args.heat_map:
          break
  print('obs=', obs, 'reward=', reward, 'done=', done)
  time.sleep(1.0/60.0)
  env.render(mode='human')



