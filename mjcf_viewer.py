# Copyright (c) 2018 ikeyasu (http://ikeyasu.com)
import gym, roboschool
import numpy as np
from mjcf_xml_env import RoboschoolMjcfXmlEnv
from gym.envs.registration import register


def random_action(action_space):
    a = action_space.sample()
    if isinstance(a, np.ndarray):
        a = a.astype(np.float32)
    return a


def zero_action(action_space):
    a = action_space.sample()
    a = np.zeros(a.shape, np.float32)
    return a


def run(model_xml, robot_name, foot_list):
    # env = gym.make("RoboschoolHumanoidFlagrun-v1")
    robot = type("Robo", (RoboschoolMjcfXmlEnv,), {
        "foot_list": foot_list,
        "__init__": lambda self: RoboschoolMjcfXmlEnv.__init__(self, model_xml, robot_name, action_dim=8, obs_dim=28,
                                                               power=2.5)
    })
    register(
        id='RoboschoolRobo-v1',
        entry_point=robot,
        max_episode_steps=1000,
        reward_threshold=800.0,
        tags={"pg_complexity": 1 * 1000000},
    )
    env = gym.make("RoboschoolRobo-v1")

    env.reset()
    env.render("human")  # This creates window to set callbacks on

    while 1:
        env.step(random_action(env.action_space))
        # env.step(zero_action(env.action_space))
        still_open = env.render("human")
        if not still_open:
            return


if __name__ == "__main__":
    foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']
    run(model_xml="ant.xml", robot_name="torso", foot_list=foot_list)
