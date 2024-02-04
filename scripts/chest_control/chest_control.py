#!/usr/bin/env python
"""Controls the robot chest component.

Initializes a ROS node 'chest_control' and sets up various ROS subscribers and
service providers for controlling a robot chest component. It receives velocity
and position messages from ROS topics 'velocity_fraction' and 'position',
respectively, and uses the messages to control the chest.

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
"""

# # Standart libraries:
import rospy
import time

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float32,
)
from std_srvs.srv import (
    SetBool,
    Empty,
)

# # Third party messages and services:
from gopher_ros_clearcore.msg import (Position)
from gopher_ros_clearcore.srv import (
    MovePosition,
    SerialWrite,
)


class ChestControl:
    """
    
    """

    def __init__(
        self,
        node_name,
        start_position,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__START_POSITION = start_position

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__last_velocity = 0
        self.__current_time = {
            'velocity': time.perf_counter(),
            'position': time.perf_counter(),
        }
        self.__previous_time = {
            'velocity': time.perf_counter(),
            'position': time.perf_counter(),
        }
        self.__chest_is_homed = False

        self.__current_chest_position = None
        self.__chest_at_start_position = False
        self.__start_position_state = 0

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
            'chest_logger': False,
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
        self.__dependency_status_topics['chest_logger'] = (
            rospy.Subscriber(
                f'/chest_logger/is_initialized',
                Bool,
                self.__chest_logger_callback,
            )
        )

        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/stop',
            Empty,
            self.__stop_handler,
        )
        rospy.Service(
            f'{self.__NODE_NAME}/home',
            Empty,
            self.__home_handler,
        )

        rospy.Service(
            f'{self.__NODE_NAME}/move_absolute_position',
            MovePosition,
            self.__absolute_position_handler,
        )
        rospy.Service(
            f'{self.__NODE_NAME}/move_relative_position',
            MovePosition,
            self.__relative_position_handler,
        )

        rospy.Service(
            f'{self.__NODE_NAME}/enable_drive',
            SetBool,
            self.__enable_drive_handler,
        )
        rospy.Service(
            f'{self.__NODE_NAME}/enable_brake',
            SetBool,
            self.__enable_brake_handler,
        )
        rospy.Service(
            f'{self.__NODE_NAME}/enable_debug',
            SetBool,
            self.__enable_debug_handler,
        )
        rospy.Service(
            f'{self.__NODE_NAME}/enable_logger',
            SetBool,
            self.__enable_logger_handler,
        )

        # # Service subscriber:
        self.__serial_write = rospy.ServiceProxy(
            '/chest_serial/serial_write',
            SerialWrite,
        )

        # # Topic publisher:

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.__NODE_NAME}/velocity_fraction',
            Float32,
            self.__velocity_callback,
        )
        rospy.Subscriber(
            f'{self.__NODE_NAME}/position',
            Position,
            self.__position_callback,
        )
        rospy.Subscriber(
            '/chest_logger/is_homed',
            Bool,
            self.__is_homed_callback,
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
    def __chest_serial_callback(self, message):
        """Monitors /chest_serial/is_initialized topic.
        
        """

        self.__dependency_status['chest_serial'] = message.data

    def __chest_logger_callback(self, message):
        """Monitors /chest_logger/is_initialized topic.
        
        """

        self.__dependency_status['chest_logger'] = message.data

    # # Service handlers:
    def __stop_handler(self, request):
        """Handler for the '/chest_control/stop' service.

        Sends a serial command to the ClearCore board to stop the motion of the
        chest component.

        """

        # Send 0 velocity to stop any motion.
        serial_command = 'vm_0.0_'
        self.__serial_write(serial_command)

        return []

    def __home_handler(self, request):
        """Handler for the '/chest_control/home' service.

        Sends a serial command to the ClearCore board to initiate the homing
        process of the chest component.

        """

        serial_command = ''

        # Start homing.
        serial_command = 'hm_'

        self.__serial_write(serial_command)

        return []

    def __absolute_position_handler(self, request):
        """Handles the '/chest_control/absolute_position' service request.

        Sends a serial command to the ClearCore board to move the chest
        component. to an absolute position with the specified speed fraction.

        """

        if not self.__is_initialized:
            return

        serial_command = f'am_{request.position * 1000}_{request.speed_fraction}_'
        self.__serial_write(serial_command)

        # Service response.
        response = True

        return response

    def __relative_position_handler(self, request):
        """Handles the '/chest_control/relative_position' service request.

        Sends a serial command to the ClearCore board to move the chest
        component. to a position relative to the current position with the
        specified speed fraction.

        """

        if not self.__is_initialized:
            return

        serial_command = f'rm_{request.position * 1000}_{request.speed_fraction}_'
        self.__serial_write(serial_command)

        # Service response.
        response = True

        return response

    def __enable_drive_handler(self, request):
        """Handler for the '/chest_control/enable_drive' service.

        Sends a serial command to the ClearCore board to enable or disable the
        drive motor of the chest component.

        """

        serial_command = ''

        if request.data:
            serial_command = 'ed_'  # Enable.

        elif not request.data:
            serial_command = 'dd_'  # Disable.

        self.__serial_write(serial_command)

        # Service response.
        success = True
        message = ''

        return success, message

    def __enable_brake_handler(self, request):
        """Handler for the '/chest_control/enable_brake' service.
        
        Sends a serial command to the ClearCore board to enable or disable the
        brake of the chest component.

        """

        serial_command = ''

        if request.data:
            serial_command = 'eb_'  # Enable.

        elif not request.data:
            serial_command = 'db_'  # Disable.

        self.__serial_write(serial_command)

        # Service response.
        success = True
        message = ''

        return success, message

    def __enable_debug_handler(self, request):
        """Handler for the '/chest_control/enable_debug' service.

        Sends a serial command to the ClearCore board to enable or disable the
        debug mode of the chest component.

        """

        serial_command = ''

        if request.data:
            serial_command = 'debug_on_'

        elif not request.data:
            serial_command = 'debug_off_'

        self.__serial_write(serial_command)

        # Service response.
        success = True
        message = ''

        return success, message

    def __enable_logger_handler(self, request):
        """Handler for the '/chest_control/enable_logger' service.

        Sends a serial command to the ClearCore board to enable or disable the
        logger mode of the chest component.

        """

        serial_command = ''

        if request.data:
            # Run logger at 100 Hz.
            serial_command = 'logger_on_10_'

        elif not request.data:
            serial_command = 'logger_off_'

        try:
            self.__serial_write(serial_command)
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.__NODE_NAME}: '
                         f'\nService call failed: {e}')

        # Service response.
        success = True
        message = ''

        return success, message

    # # Topic callbacks:
    def __velocity_callback(self, message):
        """Callback function for the 'z_chest_vel' topic.

        Sends the target velocity of the chest component to the ClearCore board
        over serial communication.

        """

        if not self.__is_initialized:
            return

        serial_command = ''

        velocity = message.data

        self.__current_time['velocity'] = time.perf_counter()

        # Send target velocity at 10 Hz rate:
        if (
            (
                self.__current_time['velocity']
                - self.__previous_time['velocity'] > 0.1
            ) and abs(velocity - self.__last_velocity) > 0.01
        ):
            self.__previous_time['velocity'] = self.__current_time['velocity']
            self.__last_velocity = velocity

            serial_command = f'vm_{velocity}_'
            self.__serial_write(serial_command)

    def __position_callback(self, message):
        """Callback function for the 'z_chest_pos' topic.

        Sends the target position and velocity of the chest component to the
        ClearCore board to control the chest movement.

        """

        if not self.__is_initialized:
            return

        serial_command = ''

        self.__current_time['position'] = time.perf_counter()

        # Rate 10 Hz:
        if (
            self.__current_time['position'] - self.__previous_time['position'] >
            0.1
        ):
            self.__previous_time['position'] = self.__current_time['position']

            serial_command = f'am_{message.position * 1000}_{message.speed_fraction}_'
            self.__serial_write(serial_command)

    def __is_homed_callback(self, message):
        """
        
        """

        self.__chest_is_homed = message.data

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
            self.__dependency_initialized and self.__chest_is_homed
            and self.__chest_at_start_position
        ):
            if not self.__is_initialized:
                rospy.loginfo(
                    (f'{self.__NODE_NAME}: '
                     f'the chest is homed.'),
                )
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

            if not self.__chest_is_homed:
                rospy.logwarn_throttle(
                    15,
                    (f'{self.__NODE_NAME}: '
                     f'the chest is not homed!'),
                )

        self.__node_is_initialized.publish(self.__is_initialized)

    def __start_position_state_machine(self):
        """
        
        """

        # State 0: Send to start position.
        if (
            self.__start_position_state == 0
            and not self.__chest_at_start_position
        ):
            # No start position provided.
            if self.__START_POSITION == -1:
                self.__chest_at_start_position = True

            else:
                # Send to start position.
                serial_command = f'am_{self.__START_POSITION * 1000}_{0.5}_'
                self.__serial_write(serial_command)
                rospy.logwarn(
                    f'{self.__NODE_NAME}: moving to the start position...',
                )

                self.__start_position_state = 1

        # State 1: Wait for the motion to finish.
        elif (
            self.__start_position_state == 1 and
            abs(self.__current_chest_position - self.__START_POSITION) <= 0.005
        ):
            self.__chest_at_start_position = True
            self.__start_position_state = 0

            rospy.loginfo(f'{self.__NODE_NAME}: at the start position.',)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if self.__chest_is_homed and not self.__chest_at_start_position:
            self.__start_position_state_machine()

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

        # Stop any chest motion.
        self.__serial_write('vm_0.0_')

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'chest_control',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )
    start_position = rospy.get_param(
        param_name=f'{rospy.get_name()}/start_position',
        default=-1,
    )

    class_instance = ChestControl(
        node_name=node_name,
        start_position=start_position,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
