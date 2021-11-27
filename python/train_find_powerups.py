import find_powerups
import os
import sys
import argparse

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
args = parser.parse_args(sys.argv[1:])
print(args)

if args.model == 'DQN':
    env = find_powerups.FindPowerupsEnv(args.filename)
    env = make_vec_env(lambda: env, n_envs=1)
    model = QRDQN('MlpPolicy', env, verbose=2,
                  learning_rate = 0.00035,
                  buffer_size=1000000,
                  exploration_initial_eps=1,
                  exploration_fraction=0.95,
                  exploration_final_eps=0.2)
else:
    env = find_powerups.FindPowerupsEnv(args.filename)
    env = make_vec_env(lambda: env, n_envs=1)
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

obs = env.reset()
n_steps = 100000
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=True)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  if done:
      obs = env.reset()
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render(mode='human')
