#!/usr/bin/env python
"""

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

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
from std_srvs.srv import (
    Empty,
    SetBool,
)

# # Third party messages and services:
from oculus_ros.msg import (ControllerJoystick)


class OculusChestMapping:
    """
    
    """

    def __init__(
        self,
        node_name,
        controller_side,
        max_speed_fraction,
        chest_compensation_for_kinova,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__CONTROLLER_SIDE = controller_side
        self.__MAX_SPEED_FRACTION = max_speed_fraction
        self.__CHEST_COMPENSATION_FOR_KINOVA = chest_compensation_for_kinova

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__oculus_joystick = ControllerJoystick()
        self.__joystick_button_state = 0

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

        self.__dependency_status['teleoperation'] = False
        self.__dependency_status_topics['teleoperation'] = (
            rospy.Subscriber(
                '/my_gen3/teleoperation/is_initialized',
                Bool,
                self.__teleoperation_callback,
            )
        )

        # # Service provider:

        # # Service subscriber:
        self.__chest_home = rospy.ServiceProxy(
            '/chest_control/home',
            Empty,
        )
        self.__chest_stop = rospy.ServiceProxy(
            '/chest_control/stop',
            Empty,
        )

        self.__positional_control_chest_compensation = rospy.ServiceProxy(
            '/my_gen3/positional_control/enable_z_chest_compensation',
            SetBool,
        )
        self.__teleoperation_chest_compensation = rospy.ServiceProxy(
            f'/my_gen3/teleoperation/enable_z_chest_compensation',
            SetBool,
        )

        # # Topic publisher:
        self.__chest_velocity = rospy.Publisher(
            '/chest_control/velocity_fraction',
            Float32,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.__CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )

        # # Timers:
        # rospy.Timer(
        #     rospy.Duration(1.0 / 100),
        #     self.__some_function_timer,
        # )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __chest_control_callback(self, message):
        """Monitors /chest_control/is_initialized topic.
        
        """

        self.__dependency_status['chest_control'] = message.data

    def __teleoperation_callback(self, message):
        """Monitors /my_gen3/teleoperation/is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     response = True

    #     return response

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    # # Timer callbacks:
    # def __some_function_timer(self, event):
    #     """Calls <some_function> on each timer callback with 100 Hz frequency.

    #     """

    #     self.__some_function()

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
            if (
                key == 'teleoperation'
                and not self.__CHEST_COMPENSATION_FOR_KINOVA
            ):
                continue

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

                if self.__CHEST_COMPENSATION_FOR_KINOVA:
                    self.__teleoperation_chest_compensation(True)
                    self.__positional_control_chest_compensation(True)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __map_chest(self):
        """
        
        """

        chest_velocity = 0.0

        if abs(self.__oculus_joystick.position_y) > 0.05:  # Noisy joystick.
            chest_velocity = np.interp(
                round(self.__oculus_joystick.position_y, 4),
                [-1.0, 1.0],
                [-1.0, 1.0],
            )

        chest_velocity = np.clip(
            chest_velocity,
            -self.__MAX_SPEED_FRACTION,
            self.__MAX_SPEED_FRACTION,
        )

        velocity_message = Float32()
        velocity_message.data = chest_velocity
        self.__chest_velocity.publish(velocity_message)

    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Homing is activated.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__chest_home()
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 0

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
        self.__joystick_button_state_machine()
        self.__map_chest()

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
        'oculus_chest_mapping',
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
        param_name=f'{rospy.get_name()}/max_speed_fraction',
        default=1.0,
    )
    chest_compensation_for_kinova = rospy.get_param(
        param_name=f'{node_name}/enable_chest_compensation_for_kinova',
        default=False,
    )

    class_instance = OculusChestMapping(
        node_name=node_name,
        controller_side=controller_side,
        max_speed_fraction=max_speed_fraction,
        chest_compensation_for_kinova=chest_compensation_for_kinova,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
