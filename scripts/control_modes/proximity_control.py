#!/usr/bin/env python
"""

Author(s):

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


class ProximityControl:
    """
    
    """

    def __init__(
        self,
        node_name,
        robot_name,
        arm_center_of_workspace_z,
        activation_boundary_z,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__ROBOT_NAME = robot_name

        self.__ARM_CENTER_OF_WORKSPACE_Z = arm_center_of_workspace_z
        self.__ACTIVATION_BOUNDARY_Z = {
            'min': self.__ARM_CENTER_OF_WORKSPACE_Z - activation_boundary_z,
            'max': self.__ARM_CENTER_OF_WORKSPACE_Z + activation_boundary_z,
        }

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__z_height = {
            'gcs': self.__ARM_CENTER_OF_WORKSPACE_Z,
            'chest': None,
        }

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

        self.__dependency_status['positional_control'] = False
        self.__dependency_status_topics['positional_control'] = (
            rospy.Subscriber(
                f'/{self.__ROBOT_NAME}/positional_control/is_initialized',
                Bool,
                self.__positional_control_callback,
            )
        )

        # # Service provider:

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
            f'/{self.__ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
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
    def __chest_pid_callback(self, message):
        """Monitors /chest_pid/is_initialized topic.
        
        """

        self.__dependency_status['chest_pid'] = message.data

    def __positional_control_callback(self, message):
        """Monitors /{self.__ROBOT_NAME}/positional_control/is_initialized topic.
        
        """

        self.__dependency_status['positional_control'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __commanded_pose_callback(self, message):
        """
        
        """

        self.__z_height['gcs'] = message.position.z

    def __chest_current_position_callback(self, message):
        """
        
        """

        self.__z_height['chest'] = message.data

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

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __publish_chest_position(self):
        """
        
        """

        if (
            self.__z_height['gcs'] <= self.__ACTIVATION_BOUNDARY_Z['min']
            or self.__z_height['gcs'] >= self.__ACTIVATION_BOUNDARY_Z['max']
        ):

            # Distance from the arms' center of workspace to the current position in
            # Z in GCS.
            z_difference_gcs = (
                self.__z_height['gcs'] - self.__ARM_CENTER_OF_WORKSPACE_Z
            )

            # Change in Z for chest:
            self.__z_height['chest'] += z_difference_gcs
            self.__z_height['chest'] = np.clip(
                self.__z_height['chest'],
                0.0,
                0.44,
            )

            goal_position_message = Float32()
            goal_position_message.data = float(self.__z_height['chest'])
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
        'proximity_control',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    robot_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )
    arm_center_of_workspace_z = rospy.get_param(
        param_name=f'{rospy.get_name()}/arm_center_of_workspace_z',
        default=0.0,
    )
    activation_boundary_z = rospy.get_param(
        param_name=f'{rospy.get_name()}/activation_boundary_z',
        default=0.3,
    )

    class_instance = ProximityControl(
        node_name=node_name,
        robot_name=robot_name,
        arm_center_of_workspace_z=arm_center_of_workspace_z,
        activation_boundary_z=activation_boundary_z,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
