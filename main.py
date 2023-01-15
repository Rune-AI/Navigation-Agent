from mlagents_envs.environment import UnityEnvironment

env = UnityEnvironment(file_name="Builds\Current\Environment.exe", seed=1, side_channels=[])

env.reset()

print(list(env.behavior_specs))

behavior_name = env.behavior_specs.keys()[0]


env.set_actions(behavior_name, action: ActionTuple)


env.close()