#!/usr/bin/env python
"""

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float32,
    Float32MultiArray,
)
from std_srvs.srv import (Empty)

# # Third party messages and services:
from oculus_ros.msg import (ControllerJoystick)
from gopher_ros_clearcore.srv import (MovePosition)


class PresetHeightsControl:
    """
    
    """

    def __init__(
        self,
        node_name,
        controller_side,
        max_speed_fraction,
        preset_heights_number,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__CONTROLLER_SIDE = controller_side
        self.__MAX_SPEED_FRACTION = max_speed_fraction

        if preset_heights_number < 3:
            raise ValueError('preset_heights_number should be >= 3.')

        interval_length = 0.44 / (preset_heights_number - 1)
        self.__PRESET_HEIGHTS = [
            interval_length * i for i in range(preset_heights_number)
        ]

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__oculus_joystick = ControllerJoystick()
        self.__joystick_state = 0
        self.__preset_height_idx = None
        self.__current_chest_position = None

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
        self.__dependency_status = {
            'chest_control': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['chest_control'] = (
            rospy.Subscriber(
                f'/chest_control/is_initialized',
                Bool,
                self.__chest_control_callback,
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

        # # Service provider:

        # # Service subscriber:
        self.__chest_stop = rospy.ServiceProxy(
            '/chest_control/stop',
            Empty,
        )
        self.__chest_absolute_position = rospy.ServiceProxy(
            '/chest_control/move_absolute_position',
            MovePosition,
        )

        # # Topic publisher:
        self.__current_preset = rospy.Publisher(
            f'{self.__NODE_NAME}/current_preset',
            Float32MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.__CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        rospy.Subscriber(
            '/chest_logger/current_position',
            Float32,
            self.__chest_current_position_callback,
        )

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __chest_control_callback(self, message):
        """Monitors /chest_control/is_initialized topic.
        
        """

        self.__dependency_status['chest_control'] = message.data

    def __controller_feedback_callback(self, message):
        """Monitors /controller_feedback/is_initialized topic.
        
        """

        self.__dependency_status['controller_feedback'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __chest_current_position_callback(self, message):
        """
        
        """

        self.__current_chest_position = message.data

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
        if (
            self.__dependency_initialized
            and self.__current_chest_position != None
        ):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __select_initial_preset_height(self):
        """
        
        """

        closest_preset = min(
            self.__PRESET_HEIGHTS,
            key=lambda x: abs(x - self.__current_chest_position),
        )
        self.__preset_height_idx = (self.__PRESET_HEIGHTS.index(closest_preset))

    def __move_preset_height(self, direction):
        """
        
        """

        self.__preset_height_idx += direction
        self.__preset_height_idx = np.clip(
            self.__preset_height_idx,
            0,
            len(self.__PRESET_HEIGHTS) - 1,
        )

        # Chest_control service call.
        self.__chest_absolute_position(
            self.__PRESET_HEIGHTS[self.__preset_height_idx],
            self.__MAX_SPEED_FRACTION,
        )

    def __joystick_state_machine(self):
        """
        
        """

        # State 0: Joystick was moved.
        if (
            abs(self.__oculus_joystick.position_y) >= 0.05
            and self.__joystick_state == 0
        ):
            if self.__oculus_joystick.position_y > 0:
                direction = 1
            else:
                direction = -1

            self.__move_preset_height(direction)
            self.__joystick_state = 1

        # State 0: Joystick was released.
        elif (
            abs(self.__oculus_joystick.position_y) < 0.05
            and self.__joystick_state == 1
        ):
            self.__joystick_state = 0

    def __publish_current_preset_height(self):
        """
        
        """

        preset_height_message = Float32MultiArray()
        preset_height_message.data = [
            self.__PRESET_HEIGHTS[self.__preset_height_idx],
            self.__preset_height_idx,
        ]
        self.__current_preset.publish(preset_height_message)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        if self.__preset_height_idx == None:
            self.__select_initial_preset_height()

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__joystick_state_machine()
        self.__publish_current_preset_height()

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
        'preset_heights_control',
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
    max_speed_fraction = rospy.get_param(
        param_name=f'{node_name}/max_speed_fraction',
        default=1.0,
    )
    preset_heights_number = rospy.get_param(
        param_name=f'{node_name}/preset_heights_number',
        default=3,
    )

    class_instance = PresetHeightsControl(
        node_name=node_name,
        controller_side=controller_side,
        max_speed_fraction=max_speed_fraction,
        preset_heights_number=preset_heights_number,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
