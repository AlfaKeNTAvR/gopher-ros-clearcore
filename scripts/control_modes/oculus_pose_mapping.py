#!/usr/bin/env python
"""

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.

TODO:

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float32,
)
from geometry_msgs.msg import (Pose)
from std_srvs.srv import (Empty)

# # Third party messages and services:
from oculus_ros.msg import (ControllerButtons)


class OculusPoseMapping:
    """
    
    """

    def __init__(
        self,
        node_name,
        controller_side,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__CONTROLLER_SIDE = controller_side

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__chest_z = None
        self.__input_z = None
        self.__input_chest_z_difference = None

        self.__tracking_button = False
        self.__tracking_state_machine_state = 0

        self.__input_tracking = False

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status['chest_pid'] = False
        self.__dependency_status_topics['chest_pid'] = (
            rospy.Subscriber(
                '/chest_pid/is_initialized',
                Bool,
                self.__chest_pid_callback,
            )
        )

        self.__dependency_status['controller_feedback'] = False
        self.__dependency_status_topics['controller_feedback'] = (
            rospy.Subscriber(
                f'/{self.__CONTROLLER_SIDE}/controller_feedback/is_initialized',
                Bool,
                self.__controller_feedback_callback,
            )
        )

        # # Service subscriber:
        self.__chest_stop = rospy.ServiceProxy(
            '/chest_control/stop',
            Empty,
        )

        # # Topic publisher:
        self.__chest_pid_goal_position = rospy.Publisher(
            '/chest_pid/goal_position',
            Float32,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/chest_logger/current_position',
            Float32,
            self.__chest_current_position_callback,
        )
        rospy.Subscriber(
            f'/{self.__CONTROLLER_SIDE}/controller_feedback/pose',
            Pose,
            self.__oculus_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.__CONTROLLER_SIDE}/controller_feedback/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __chest_pid_callback(self, message):
        """Monitors /chest_pid/is_initialized topic.
        
        """

        self.__dependency_status['chest_pid'] = message.data

    def __controller_feedback_callback(self, message):
        """Monitors /controller_feedback/is_initialized topic.
        
        """

        self.__dependency_status['controller_feedback'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __chest_current_position_callback(self, message):
        """
        
        """

        self.__chest_z = message.data

    def __oculus_pose_callback(self, message):
        """

        """

        self.__input_z = message.position.z

    def __oculus_buttons_callback(self, message: ControllerButtons):
        """

        """

        self.__tracking_button = message.grip_button

    # # Timer callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                self.__tracking_state_machine_state = 0
                self.__input_tracking = False

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __tracking_state_machine(self, button):
        """
        
        """

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):

            self.__tracking_state_machine_state = 1

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):

            self.__tracking_state_machine_state = 2

            self.__input_chest_z_difference = (self.__input_z - self.__chest_z)
            self.__input_tracking = True

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):

            self.__tracking_state_machine_state = 3
            self.__input_tracking = False

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):

            self.__tracking_state_machine_state = 0

    def __publish_chest_position(self):
        """
        
        """

        # Protection against 0, 0, 0 controller input values.
        # Controller loses connection, goes into a sleep mode etc.
        if self.__input_z == 0:
            self.__tracking_state_machine_state = 0
            self.__input_tracking = False

            rospy.logerr(
                (
                    f'/{self.__NODE_NAME}: '
                    f'\n0 position while active tracking! '
                    '\nStopped input tracking.'
                ),
            )

            return

        compensated_input_z_position = (
            self.__input_z - self.__input_chest_z_difference
        )

        goal_position_message = Float32()
        goal_position_message.data = float(
            np.clip(compensated_input_z_position, 0.0, 0.44)
        )
        self.__chest_pid_goal_position.publish(goal_position_message)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        self.__tracking_state_machine(self.__tracking_button)

        if self.__input_tracking:
            self.__publish_chest_position()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.
        self.__chest_stop()

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'oculus_pose',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    controller_side = rospy.get_param(
        param_name=f'{node_name}/controller_side',
        default='right',
    )

    class_instance = OculusPoseMapping(
        node_name=node_name,
        controller_side=controller_side,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
