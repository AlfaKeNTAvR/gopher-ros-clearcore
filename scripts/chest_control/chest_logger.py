#!/usr/bin/env python
"""Publishes status information about the robot chest component.

This module subscribes to a ROS topic 'logger_info', which publishes status
information of a robot chest component in the form of a JSON string. The JSON
string is deserialized to obtain velocity, position and status of the components
of the chest, which are then published to various ROS topics.

Author(s):
    1. Yveder Joseph (ygjoseph@wpi.edu), Worcester Polytechnic Institute (WPI),
       2023.
    2. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
"""

# # Standart libraries:
import rospy
import json

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    String,
    Float32,
)

# # Third party messages and services:
from gopher_ros_clearcore.srv import (SerialWrite)


class ChestLogger:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.

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
            'chest_serial': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['chest_serial'] = (
            rospy.Subscriber(
                f'/chest_serial/is_initialized',
                Bool,
                self.__chest_serial_callback,
            )
        )

        # # Service provider:

        # # Service subscriber:
        self.__serial_write = rospy.ServiceProxy(
            '/chest_serial/serial_write',
            SerialWrite,
        )

        # # Topic publisher:
        self.__current_position = rospy.Publisher(
            f'{self.__NODE_NAME}/current_position',
            Float32,
            queue_size=1,
        )
        self.__current_velocity = rospy.Publisher(
            f'{self.__NODE_NAME}/current_velocity',
            Float32,
            queue_size=1,
        )

        self.__is_homed = rospy.Publisher(
            f'{self.__NODE_NAME}/is_homed',
            Bool,
            queue_size=1,
        )
        self.__brake_status = rospy.Publisher(
            f'{self.__NODE_NAME}/brake_status',
            Bool,
            queue_size=1,
        )
        self.__drive_status = rospy.Publisher(
            f'{self.__NODE_NAME}/drive_status',
            Bool,
            queue_size=1,
        )
        # self.__upper_limit_status = rospy.Publisher(
        #     f'{self.__NODE_NAME}/upper_limit_status',
        #     Bool,
        #     queue_size=1,
        # )
        # self.__lower_limit_status = rospy.Publisher(
        #     f'{self.__NODE_NAME}/lower_limit_status',
        #     Bool,
        #     queue_size=1,
        # )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.__NODE_NAME}/logger_info',
            String,
            self.__logger_info_callback,
        )

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __chest_serial_callback(self, message):
        """Monitors /chest_serial/is_initialized topic.
        
        """

        self.__dependency_status['chest_serial'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __logger_info_callback(self, message):
        """Callback function for the ROS logger_info topic subscriber.

        This method is called every time a message is received on the
        'logger_info' topic. It deserializes the message (which is a JSON
        string), and publishes the position, velocity, and status information of
        the chest on various ROS topics.
        """

        try:
            if len(message.data) > 0:
                # Deserialize JSON String to Python dictionary.
                status = json.loads(message.data)

                # Object for publishing velocity.
                velocity = Float32()
                velocity.data = float(status['Motor']['CurrentVelocity']) / 100
                self.__current_velocity.publish(velocity)

                # Object for publishing position.
                position = Float32()
                position.data = float(status['Motor']['CurrentPosition']) / 1000
                self.__current_position.publish(position)

                # Object for publishing homing status:
                is_homed = Bool()
                is_homed.data = bool(status['Motor']['Homed'])
                self.__is_homed.publish(is_homed)

                # Object for publishing status of components of chest.
                self.__brake_status.publish(Bool(status['Brake']['Active']))
                self.__drive_status.publish(Bool(status['Motor']['Enabled']))
                # self.__upper_limit_status.publish(
                #     Bool(status['Limits']['UpperLimitReached'])
                # )
                # self.__lower_limit_status.publish(
                #     Bool(status['Limits']['LowerLimitReached'])
                # )

        except json.JSONDecodeError as e:
            # Handle JSON decoding error.
            rospy.logerr(f'{self.__NODE_NAME}: '
                         f'\nJSON decoding error: {e}')

        except Exception as e:
            # Handle other exceptions if necessary.
            rospy.logerr(f'{self.__NODE_NAME}: '
                         f'\nAn error occurred: {e}')

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

                # Enable logger (100 Hz).
                try:
                    self.__serial_write('logger_on_10_')

                except rospy.ServiceException as e:
                    rospy.logerr(
                        f'{self.__NODE_NAME}: '
                        f'\nService call failed: {e}'
                    )

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                # Disable logger.
                try:
                    self.__serial_write('logger_off_')

                except rospy.ServiceException as e:
                    rospy.logerr(
                        f'{self.__NODE_NAME}: '
                        f'\nService call failed: {e}'
                    )

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

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

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # Disable logger.
        self.__serial_write('logger_off_')

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'chest_logger',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    class_instance = ChestLogger(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
