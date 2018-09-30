# Copyright (c) 2018 ikeyasu (http://ikeyasu.com)
import gym
import numpy as np
# noinspection PyUnresolvedReferences
import roboschool
from gym.envs.registration import register
from roboschool import cpp_household
from roboschool.gym_forward_walker import RoboschoolForwardWalker
from roboschool.gym_urdf_robot_env import RoboschoolUrdfEnv
from roboschool.scene_stadium import SinglePlayerStadiumScene


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
    rads = old_rads + 0.025
    a = np.sin(rads)
    if isinstance(a, np.ndarray):
        a = a.astype(np.float32)
    return a, rads


def _robo_init(self, model_urdf, robot_name):
    RoboschoolForwardWalker.__init__(self, power=0.30)
    RoboschoolUrdfEnv.__init__(self,
                               model_urdf,
                               robot_name,
                               action_dim=30, obs_dim=70,
                               fixed_base=False,
                               self_collision=True)


def _robot_specific_reset(self):
    RoboschoolForwardWalker.robot_specific_reset(self)
    self.set_initial_orientation(yaw_center=0, yaw_random_spread=np.pi)
    self.head = self.parts["head"]


def _set_initial_orientation(self, yaw_center, _yaw_random_spread):
    # noinspection PyArgumentList
    cpose = cpp_household.Pose()
    yaw = yaw_center

    cpose.set_xyz(self.start_pos_x, self.start_pos_y, self.start_pos_z + 1.0)
    cpose.set_rpy(0, 0, yaw)  # just face random direction, but stay straight otherwise
    self.cpp_robot.set_pose_and_speed(cpose, 0, 0, 0)
    self.initial_z = 1.5


def run(model_urdf, robot_name, footlist):
    # env = gym.make("RoboschoolHumanoidFlagrun-v1")
    robot = type("Robo", (RoboschoolForwardWalker, RoboschoolUrdfEnv,), {
        "foot_list": footlist,
        "__init__": lambda self: _robo_init(self, model_urdf, robot_name),
        "create_single_player_scene":
            lambda self: SinglePlayerStadiumScene(gravity=9.8, timestep=0.0165 / 8, frame_skip=8),
        "alive_bonus": lambda self, z, pitch: 1,
        "robot_specific_reset": _robot_specific_reset,
        "set_initial_orientation": _set_initial_orientation
    })
    register(
        id='RoboschoolRoboUrdf-v1',
        entry_point=robot,
        max_episode_steps=1000,
        tags={"pg_complexity": 200 * 1000000}
    )
    env = gym.make("RoboschoolRoboUrdf-v1")

    env.reset()
    env.render("human")  # This creates window to set callbacks on

    rads = np.zeros(env.action_space.sample().shape, np.float32)
    while 1:
        # Sin wave action
        actions, rads = sin_action(rads)
        env.step(actions)
        # Random action
        # env.step(random_action(env.action_space))
        # Zero action
        # env.step(zero_action(env.action_space))
        still_open = env.render("human")
        if not still_open:
            return


if __name__ == "__main__":
    foot_list = []
    run(model_urdf="atlas_description/urdf/atlas_v4_with_multisense.urdf",
        robot_name="pelvis", footlist=foot_list)
