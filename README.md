# Building


## Requirements

(TODO: add links)

Most of the C++ dependencies can be found in the bazel WORKSPACE
file. The rest of the dependencies are:

* Bazel
* pybind
* python3
* stable-baselines:
    Fetch URL: https://github.com/hill-a/stable-baselines
    Hash: 259f27868f0d727d990f50e04da6e3a5d5367582

## Building

TODO:

bazel ...

### Training a simple agent

Before trying to learn complicated strategies / agents (and
also since I have no idea what I'm doing), I created a simple agent
where the actions are which direction the agent should move,
and the reward is proportinal to whether the agent picks up a
power-up or not.

Action Space:

*  Discrete direction (8-directions, sideways and diagonals)

Observation space

* 2D Position of the agent
* Agent health
* Agent armor
* Time until next powerup spawn

Reward:

* Increase in health or armor from getting a power-up
* Minus 1 if the action caused no movement (e.g., hitting a wall)
* [Optional] Small reward proportional to movement (commented out in code)


python/find_powerups.py
python/train_find_powerups.py

python3 train_find_powerups.py --model=DQN --train=True

For 1M iterations, you should start to see episode reward >= 140 (at about) 700K)

Expected behavior: go to 1 power-up, then over to another while one is still spawning,
 then back to another spawn point (etc.)


To simulate some runs with the agent, use:
python3 train_find_powerups.py --model=DQN --train=False

    env = find_powerups.FindPowerupsEnv(args.filename)
    env = make_vec_env(lambda: env, n_envs=1)
    model = QRDQN('MlpPolicy', env, verbose=2,
                  learning_rate = 0.0005, 
                  buffer_size=1000000,
                  exploration_initial_eps=1,
                  exploration_fraction=0.95,
                  exploration_final_eps=0.01)
    if not args.train:
        model.exploration_rate = 0.01
