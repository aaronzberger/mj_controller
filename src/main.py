#!/usr/bin/python3

import json
import os
import sys
from pathlib import Path

import mujoco_py as mp
import rospkg
import rospy
from mj_controller.srv import RegisterGroup, RegisterGroupResponse
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from termcolor import colored

POSITIONS_TOPIC = '/actuator_positions'


class MJ_Controller():
    @classmethod
    def __init__(cls):
        path = os.path.realpath(__file__)
        path = str(Path(path).parent.parent.parent)

        print(colored('Loading UR5 model...', color='yellow'), end='')
        cls.model = mp.load_model_from_path(os.path.join(
            rospkg.RosPack().get_path('mj_controller'), 'UR5+gripper', 'UR5gripper_2_finger.xml'))
        print(colored('LOADED', color='green'))

        cls.actuator_ids = json.load(open(os.path.join(
            rospkg.RosPack().get_path('mj_controller'), 'actuators.json'), 'r'))
        cls.motor_group_subs = []

        # Always create the services at the end so all necessary variables exist when they are called
        cls.motor_registration_service = rospy.Service('register_motor_group', RegisterGroup, cls.register_motor_group)
        cls.airflow_on_service = rospy.Service('airflow_on', Trigger, cls.airflow_on)
        cls.airflow_off_service = rospy.Service('airflow_off', Trigger, cls.airflow_off)

        cls.sim_started = False
        cls.start_sim_service = rospy.Service('start_sim', Trigger, cls.start_sim)

        print('Waiting for subsystems to register...')

    @classmethod
    def handle_update(cls, data, ids):
        if not cls.sim_started:
            return
        # Set the actuator velocities to the desired velocities from the controllers
        for id, pos in zip(ids, data.data):
            cls.control[id] = pos

    @classmethod
    def register_motor_group(cls, req):
        try:
            motor_ids = cls.actuator_ids[req.group]
        except KeyError:
            return RegisterGroupResponse(success=False, position_topic=None)

        cls.motor_group_subs.append(
            rospy.Subscriber(
                req.velocity_topic, Float64MultiArray, callback=cls.handle_update, callback_args=motor_ids))

        print(colored('Registered subsystem {} to the simulation'.format(req.group), color='cyan'))

        return RegisterGroupResponse(success=True, position_topic=POSITIONS_TOPIC, ids=motor_ids)

    @classmethod
    def start_sim(cls, _):
        print(colored('Starting simulation... ', color='yellow'), end='')

        assert len(cls.motor_group_subs) > 0, 'Simulation started with no actuators registered'
        cls.sim = mp.MjSim(cls.model)
        cls.viewer = mp.MjViewer(cls.sim)
        cls.control = cls.sim.data.ctrl

        cls.actuators = []
        for i in range(len(cls.control)):
            cls.actuators.append([
                i,
                cls.model.actuator_id2name(i),
                cls.model.actuator_trnid[i][0],
                cls.model.joint_id2name(cls.model.actuator_trnid[i][0])
            ])

        cls.pub_positions = rospy.Publisher(POSITIONS_TOPIC, Float64MultiArray, queue_size=1)

        # Start executing the planned actions
        rospy.Timer(rospy.Duration(secs=0.01), cls.sim_step)

        sys.stdout.write('\033[F\033[K')
        print('\r' + colored('Starting simulation... ', color='yellow'), end='', flush=True)
        print(colored('STARTED', color='green'))

        cls.sim_started = True

        return TriggerResponse(success=True)

    @classmethod
    def sim_step(cls, _: rospy.timer.TimerEvent):
        # Get positions of actuators from Mujoco
        current_joint_values = cls.sim.data.qpos[[i[2] for i in cls.actuators]]

        # Publish them
        cls.pub_positions.publish(Float64MultiArray(data=current_joint_values))

        cls.sim.step()
        cls.viewer.render()

    @classmethod
    def airflow_on(cls, req: Trigger):
        cls.model.body_pos[3] = [0, 1.25, 1.8]
        cls.model.body_pos[2] = [0, 1.18, 1.8]
        return TriggerResponse(success=True)

    @classmethod
    def airflow_off(cls, req: Trigger):
        cls.model.body_pos[2] = [0, 1.25, 1.8]
        cls.model.body_pos[3] = [0, 1.18, 1.8]
        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('mj_controller', log_level=rospy.INFO)

    mj_controller_node = MJ_Controller()

    rospy.spin()
