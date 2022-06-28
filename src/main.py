import rospy
import os
from pathlib import Path
import mujoco_py as mp
from termcolor import colored
import ikpy
import ikpy.chain
import rospkg
from mj_controller.srv import RegisterGroup, RegisterGroupResponse
import json
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse


POSITIONS_TOPIC = '/actuator_positions'


class MJ_Controller():
    @classmethod
    def __init__(cls):
        path = os.path.realpath(__file__)
        path = str(Path(path).parent.parent.parent)
        cls.model = mp.load_model_from_path(os.join(
            rospkg.RosPack().get_path('mj_controller'), 'UR5+gripper' + 'UR5gripper_2_finger.xml'))

        cls.ee_chain = ikpy.chain.Chain.from_urdf_file(os.join(
            rospkg.RosPack().get_path('mj_controller'), 'UR5+gripper', 'ur5.urdf'))

        cls.motor_registration_service = rospy.Service('register_motor_group', RegisterGroup, cls.register_motor_group)
        cls.start_sim_service = rospy.Service('start_sim', Empty, cls.start_sim)

        cls.actuator_ids = json.load(os.path.join(
            rospkg.RosPack().get_path('mj_controller'), 'actuators.json'))
        cls.motor_group_subs = []

        rospy.Timer(rospy.Duration(secs=0.01), cls.sim_step)

    @classmethod
    def handle_update(cls, data, ids):
        # Set the actuator velocities to the desired velocities from the controllers
        cls.control[ids] = data.data

    @classmethod
    def register_motor_group(cls, req):
        try:
            motor_ids = cls.actuator_ids[req.group]
        except KeyError:
            return RegisterGroupResponse(success=False, position_topic=None)

        cls.motor_group_subs.append(
            rospy.Subscriber(
                req.velocity_topic, Float64MultiArray, callback=cls.handle_update, callback_args=motor_ids))

        return RegisterGroupResponse(success=True, position_topic=POSITIONS_TOPIC, ids=motor_ids)

    @classmethod
    def start_sim(cls):
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

        return EmptyResponse

    @classmethod
    def sim_step(cls, _: rospy.timer.TimerEvent):
        # Get positions of actuators from Mujoco
        current_joint_values = cls.sim.data.qpos[[i[2] for i in cls.actuators]]

        # Publish them
        cls.pub_positions.publish(Float64MultiArray(data=current_joint_values))

        cls.sim.step()
        cls.viewer.render()


if __name__ == '__main__':
    rospy.init_node('mj_controller', log_level=rospy.INFO)

    mj_controller_node = MJ_Controller()

    rospy.spin()
