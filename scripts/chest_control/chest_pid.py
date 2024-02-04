#!/usr/bin/env python
"""Implements chest position control using PID module.

Author (s):
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
    Float64,
)
from std_srvs.srv import (
    SetBool,
    Empty,
)

# # Third party messages and services:


class ChestPID:
    """
    
    """

    def __init__(
        self,
        node_name,
        max_speed_fraction,
        cutoff_speed_fraction,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__MAX_SPEED_FRACTION = max_speed_fraction
        self.__CUTOFF_SPEED_FRACTION = cutoff_speed_fraction

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__pid_enabled = True
        self.__current_position = None
        self.__goal_position = None
        self.__control_effort = 0.0

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

        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/enable_pid',
            SetBool,
            self.__enable_pid_handler,
        )
        # # Service subscriber:
        self.__chest_stop = rospy.ServiceProxy(
            '/chest_control/stop',
            Empty,
        )

        # # Topic publisher:
        self.__chest_velocity = rospy.Publisher(
            '/chest_control/velocity_fraction',
            Float32,
            queue_size=1,
        )

        self.__state = rospy.Publisher(
            f'{self.__NODE_NAME}/state',
            Float64,
            queue_size=1,
        )
        self.__setpoint = rospy.Publisher(
            f'{self.__NODE_NAME}/setpoint',
            Float64,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/chest_logger/current_position',
            Float32,
            self.__current_position_callback,
        )
        rospy.Subscriber(
            f'{self.__NODE_NAME}/goal_position',
            Float32,
            self.__goal_position_callback,
        )

        rospy.Subscriber(
            f'{self.__NODE_NAME}/control_effort',
            Float64,
            self.__control_effort_callback,
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

    # # Service handlers:
    def __enable_pid_handler(self, request):
        """

        """

        self.__pid_enabled = request.data

        success = True
        message = ''

        return success, message

    # # Topic callbacks:
    def __current_position_callback(self, message):
        """

        """

        self.__current_position = message.data

        if self.__goal_position == None and self.__is_initialized:
            self.__goal_position = message.data

    def __goal_position_callback(self, message):
        """

        """

        self.__goal_position = np.clip(message.data, 0, 0.44)

    def __control_effort_callback(self, message):
        """

        """

        self.__control_effort = message.data

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
        if (self.__dependency_initialized and self.__current_position != None):
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

    def __update_pid(self):
        """
        
        """

        # State:
        state_message = Float64()
        state_message.data = self.__current_position
        self.__state.publish(state_message)

        # Setpoint:
        setpoint_message = Float64()
        setpoint_message.data = self.__goal_position
        self.__setpoint.publish(setpoint_message)

        # Chest velocity output:
        chest_velocity = round(self.__control_effort, 2)

        if abs(chest_velocity) < self.__CUTOFF_SPEED_FRACTION:
            chest_velocity = 0

        chest_velocity = np.clip(
            chest_velocity,
            -self.__MAX_SPEED_FRACTION,
            self.__MAX_SPEED_FRACTION,
        )

        velocity_message = Float32()
        velocity_message.data = chest_velocity
        self.__chest_velocity.publish(velocity_message)

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

        if self.__pid_enabled and self.__goal_position != None:
            self.__update_pid()

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
        'chest_pid',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    max_speed_fraction = rospy.get_param(
        param_name=f'{rospy.get_name()}/max_speed_fraction',
        default=1.0,
    )
    cutoff_speed_fraction = rospy.get_param(
        param_name=f'{rospy.get_name()}/cutoff_speed_fraction',
        default=0.001,
    )

    class_instance = ChestPID(
        node_name=node_name,
        max_speed_fraction=max_speed_fraction,
        cutoff_speed_fraction=cutoff_speed_fraction,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
