# Copyright (c) 2018 ikeyasu (http://ikeyasu.com)
import os
import gym
import numpy as np
# noinspection PyUnresolvedReferences
import roboschool
from gym.envs.registration import register
from roboschool import cpp_household
from roboschool.gym_forward_walker import RoboschoolForwardWalker
from roboschool.gym_mujoco_xml_env import RoboschoolMujocoXmlEnv

from gym_forward_walker_servo import RoboschoolForwardWalkerServo


def random_action(action_space):
    a = action_space.sample()
    if isinstance(a, np.ndarray):
        a = a.astype(np.float32)
    return a


def zero_action(action_space):
    a = action_space.sample()
    a = np.zeros(a.shape, np.float32)
    return a


def sin_action(old_rads):
    rads = old_rads + 0.01
    a = np.sin(rads)
    if isinstance(a, np.ndarray):
        a = a.astype(np.float32)
    return a, rads


def _robo_init(self, model_xml, robot_name, action_dim):
    if self.servo:
        RoboschoolForwardWalkerServo.__init__(self, power=2.5)
    else:
        RoboschoolForwardWalker.__init__(self, power=2.5)
    RoboschoolMujocoXmlEnv.__init__(self,
                               model_xml,
                               robot_name,
                               action_dim=action_dim, obs_dim=70)


def _robot_specific_reset(self):
    if self.servo:
        RoboschoolForwardWalkerServo.robot_specific_reset(self)
    else:
        RoboschoolForwardWalker.robot_specific_reset(self)
    self.set_initial_orientation(yaw_center=0, yaw_random_spread=np.pi)
    #self.head = self.parts["head"]


def _set_initial_orientation(self, yaw_center, yaw_random_spread):
    # noinspection PyArgumentList
    cpose = cpp_household.Pose()
    yaw = yaw_center

    cpose.set_xyz(self.start_pos_x, self.start_pos_y, self.start_pos_z + 1.0)
    cpose.set_rpy(0, 0, yaw)  # just face random direction, but stay straight otherwise
    self.cpp_robot.set_pose_and_speed(cpose, 0, 0, 0)
    self.initial_z = 1.5


def run(model_xml, robot_name, footlist, servo=False, action_dim=8):
    # env = gym.make("RoboschoolHumanoidFlagrun-v1")
    walker_class = RoboschoolForwardWalkerServo if servo else RoboschoolForwardWalker
    robot = type("Robo", (walker_class, RoboschoolMujocoXmlEnv,), {
        "servo": servo,
        "foot_list": footlist,
        "__init__": lambda self: _robo_init(self, model_xml, robot_name, action_dim=action_dim),
        "alive_bonus": lambda self, z, pitch: 1,
        "robot_specific_reset": _robot_specific_reset,
        "set_initial_orientation": _set_initial_orientation
    })
    register(
        id='RoboschoolRoboMjcf-v1',
        entry_point=robot,
        max_episode_steps=1000,
        tags={"pg_complexity": 200 * 1000000}
    )
    env = gym.make("RoboschoolRoboMjcf-v1")

    env.reset()
    env.render("human")  # This creates window to set callbacks on

    rads = np.zeros(env.action_space.sample().shape, np.float32)
    while 1:
        # Sin wave action
        actions, rads = sin_action(rads)
        env.step(actions)
        # Random action
        #env.step(random_action(env.action_space))
        # Zero action
        #env.step(zero_action(env.action_space))
        still_open = env.render("human")
        if not still_open:
            return


if __name__ == "__main__":
    foot_list = []
    run(model_xml=os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples/ant.xml"),
        robot_name="torso", footlist=foot_list, servo=True, action_dim=8)
