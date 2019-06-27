from gym.envs.registration import register

register(
    id='cat-v0',
    entry_point='gym_cat.envs:cat',
)
