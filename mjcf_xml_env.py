# Copyright (c) 2018 ikeyasu (http://ikeyasu.com)

from roboschool.gym_mujoco_walkers import RoboschoolForwardWalkerMujocoXML


# This code is originally from
# https://github.com/openai/roboschool/blob/1f5c1b36b3a71508c775ccc8c4344b694ff8873f/roboschool/gym_mujoco_xml_env.py#L36
# License: https://github.com/openai/roboschool/blob/1f5c1b36b3a71508c775ccc8c4344b694ff8873f/LICENSE.md
class RoboschoolMjcfXmlEnv(RoboschoolForwardWalkerMujocoXML):

    def __init__(self, model_xml, robot_name, action_dim, obs_dim, power):
        RoboschoolForwardWalkerMujocoXML.__init__(self, "dummy.xml", robot_name, action_dim=action_dim, obs_dim=obs_dim, power=power)
        self.model_xml = model_xml

    def alive_bonus(self, z, pitch):
        return +1

    def reset(self):
        return self._reset()

    def step(self, action):
        return self._step(action)

    def render(self, mode='human'):
        return self._render(mode, False)

    def _reset(self):
        if self.scene is None:
            self.scene = self.create_single_player_scene()
        if not self.scene.multiplayer:
            self.scene.episode_restart()
        self.mjcf = self.scene.cpp_world.load_mjcf(self.model_xml)
        self.ordered_joints = []
        self.jdict = {}
        self.parts = {}
        self.frame = 0
        self.done = 0
        self.reward = 0
        dump = 0
        for r in self.mjcf:
            if dump: print("ROBOT '%s'" % r.root_part.name)
            if r.root_part.name == self.robot_name:
                self.cpp_robot = r
                self.robot_body = r.root_part
            for part in r.parts:
                if dump: print("\tPART '%s'" % part.name)
                self.parts[part.name] = part
                if part.name == self.robot_name:
                    self.cpp_robot = r
                    self.robot_body = part
            for j in r.joints:
                if dump: print(
                    "\tALL JOINTS '%s' limits = %+0.2f..%+0.2f effort=%0.3f speed=%0.3f" % ((j.name,) + j.limits()))
                if j.name[:6] == "ignore":
                    j.set_motor_torque(0)
                    continue
                j.power_coef = 100.0
                self.ordered_joints.append(j)
                self.jdict[j.name] = j
        assert (self.cpp_robot)
        self.robot_specific_reset()
        for r in self.mjcf:
            r.query_position()
        s = self.calc_state()  # optimization: calc_state() can calculate something in self.* for calc_potential() to use
        self.potential = self.calc_potential()
        self.camera = self.scene.cpp_world.new_camera_free_float(self.VIDEO_W, self.VIDEO_H, "video_camera")
        return s
