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


### Training the high-level agent

As I didn't want to wait forever to train an agent capable of learning simple
strategies before learning any more higher-level behaviors, the high level
agent is parameterized using a higher-level set of actions.

Each step, the agent chooses from one of 4+N actions, where N is the number
of power ups in the level. There are 4 directional actions: forward, backward,
left, right.

For a new action (different from previous), the agent creates a plan in
the direction of movement (1 unit distance in the world) that includes
avoiding obstacles. The default size of the world is a [10, 10] and max
velocity is 4, so moving from one end of the world to the other takes 5 seconds.

The other N actions correspond moving along an optimal path towards one of the
powerups. Agent training happens at the granularity of 1/2 a second in the world
time, and the simulation runs 15 iterations with a time delta of 1/30th of a second
to achieve this.

In the case that the action is FORWARD and an opponent is visible, the search
plan will be along the direction to the opponent.

If the action is the same as the previous action, the same plan is followed
until it is complete (or action is changed). And if it is completed, a new
plan is created as described above.

Code links:
 * [python/high_level_env.py](python/high_level_env.py): The "gym" environment defining the simulation environment.
 * [python/train_high_level_agent.py](python/train_high_level_agent.py): Scripts to train the high-level agent.
 * [src/agent/high_level_agent.h](high_level_agent.h): Header for HighLevelAgent.
 * [src/agent/high_level_agent.h](high_level_agent.cc): Implementation of sensing/actions for the HighLevelAgent.