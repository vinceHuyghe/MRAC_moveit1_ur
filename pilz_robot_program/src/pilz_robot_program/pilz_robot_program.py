from copy import deepcopy
from math import pi
from operator import add
from typing import Any, Iterable, List, Optional, Tuple, Union

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander import MoveGroupCommander, MoveItCommanderException
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MotionSequenceItem, MotionSequenceRequest,
                             MoveItErrorCodes, OrientationConstraint,
                             PlanningOptions, PositionConstraint,
                             RobotTrajectory)
from moveit_msgs.srv import GetMotionSequence, GetMotionSequenceResponse
from shape_msgs.msg import SolidPrimitive
from tf import transformations


def get_tf_buffer() -> tf2_ros.Buffer:
    if not hasattr(get_tf_buffer, 'buffer'):
        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer)
        get_tf_buffer.buffer = buffer

    return get_tf_buffer.buffer


# Default velocities
_DEFAULT_CARTESIAN_VEL_SCALE = 0.1
_DEFAULT_JOINT_VEL_SCALE = 1.0

# Default acceleration
_DEFAULT_ACC_SCALE = 0.1
_DEFAULT_BASE_LINK = 'base_link'

# Tolerance for cartesian pose
_DEFAULT_POSITION_TOLERANCE = 2e-3
_DEFAULT_ORIENTATION_TOLERANCE = 1e-5

# axis sequence of euler angles
_AXIS_SEQUENCE = 'rzyz'


class _AbstractCmd:
    """Base class for all commands."""

    def __init__(self) -> None:
        # set robot state as empty diff in planning scene to start with current planning scene
        self._planning_options = PlanningOptions()
        self._planning_options.planning_scene_diff.robot_state.is_diff = True


Goal = Union[Pose, PoseStamped, Iterable[float]]


class BaseCmd(_AbstractCmd):
    """Base class for all single commands.
    :param goal: The goal of the motion, which can be given in joint (list or tuple of float, in the order of active
        joints in the planning group) or Cartesian space. For geometry_msgs/Pose you can specify the reference frame
        as extra parameter `reference_frame` (see below). If a PoseStamped is passed as goal, timestamp has to be zero
        and the `frame_id` from the Header is used instead of `reference_frame`
    :note:
        The geometry_msgs/Pose consists of position and orientation (quaternion). When creating an instance of
        geometry_msgs/Pose, the position and orientation can be left as uninitialized. The uninitialized position is
        considered as zero position. The uninitialized orientation is considered as keeping the current orientation
        unchanged. This difference is because uninitialized position can not be recognized when comparing with a zero
        position but uninitialized orientation can.
    :param relative: Has to be set to:
            * :py:obj:`False` if the goal states the target position as absolute position with regard to base coordinate
                              system.
            * :py:obj:`True` if the goal states the target position as offset relative to the current robot position.
            The orientation is added as offset to the euler-angles.
            The offset has to be stated with regard to the base coordinate system.
            E.g. to move the robot 0.1m up and tilt it 10degrees around the global z-axis use:
            ::
                Ptp(goal=Pose(position=Point(0., 0., 0.1), orientation=from_euler(math.radians(10), 0., 0.)),
                    vel_scale=0.4)
            Note the gimbal lock, if you pass a relative rotation to this function: If b==0, the values for a and c
            depend on each other and may thus give arbitrary euler angles, when converting back from quaternion to
            euler internally.
            The function assumes, you want to take the shorter rotation distance, so you should only pass
            rotations smaller than 180 degrees for relative movements.
    :param reference_frame: The frame of reference parameter allows to change the reference coordinate system for the
        passed goal position and orientation.
        Any published tf can be used as frame by reference. The reference_frame has to be a valid tf id string.
        Setting no reference_frame will use the move groups default frame
    :type relative: bool
    :type reference_frame: string
    """

    def __init__(
        self,
        goal: Goal = None,
        vel_scale: float = _DEFAULT_CARTESIAN_VEL_SCALE,
        acc_scale: float = _DEFAULT_ACC_SCALE,
        relative: bool = False,
        reference_frame: str = _DEFAULT_BASE_LINK,
    ) -> None:
        super().__init__()

        # Needs to be set by derived classes
        self._pipeline_id: Optional[str] = None
        self._planner_id: Optional[str] = None

        try:
            self._goal = tuple(goal)
        except TypeError:
            self._goal = goal

        if not isinstance(
            goal,
            (
                Pose,
                PoseStamped,
                tuple,
            ),
        ):
            raise ValueError(
                'Goal must be a Pose, PoseStamped or tuple of joint positions')

        self._vel_scale = vel_scale
        self._acc_scale = acc_scale
        self._relative = relative

        self._reference_frame = reference_frame

    def __str__(self) -> str:
        out_str = _AbstractCmd.__str__(self)
        out_str += ' vel_scale: ' + str(self._vel_scale)
        out_str += ' acc_scale: ' + str(self._acc_scale)
        out_str += ' reference: ' + str(self._reference_frame)
        return out_str

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, BaseCmd):
            return hash(self) == hash(other)
        return NotImplemented

    def __ne__(self, other: Any) -> bool:
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self) -> int:
        return hash(tuple(sorted([str(t) for t in self.__dict__.items()])))

    __repr__ = __str__

    def _cmd_to_request(self, move_group: MoveGroupCommander) -> MotionPlanRequest:
        """Transforms the given command to a MotionPlanRequest."""
        req = MotionPlanRequest()

        self._target_link = move_group.get_end_effector_link()

        self._planning_group = move_group.get_name()

        self._robot_reference_frame = move_group.get_planning_frame()
        self._active_joints = move_group.get_active_joints()
        self._start_joint_states = move_group.get_current_joint_values()

        self._tf_buffer = get_tf_buffer()

        # Convert current pose to the reference frame
        start = move_group.get_current_pose(
            end_effector_link=self._target_link)
        if self._reference_frame == self._robot_reference_frame:
            self._start_pose = start.pose
        else:
            self._start_pose = self._transform_frame(
                self._robot_reference_frame, start.pose, self._reference_frame)

        # Set general info
        req.pipeline_id = self._pipeline_id
        req.planner_id = self._planner_id
        req.group_name = self._planning_group
        req.max_velocity_scaling_factor = self._vel_scale
        req.max_acceleration_scaling_factor = self._acc_scale * \
            self._calc_acc_scale(1.0)
        req.allowed_planning_time = 1.0

        # Set an empty diff as start_state => the current state is used by the planner
        req.start_state.is_diff = True

        # Set goal constraint
        if self._goal is None:
            raise NameError('Goal is not given.')

        conversion_methods = [
            self._pose_to_constraint,
            self._pose_stamped_to_constraint,
            self._joint_values_to_constraint,
        ]
        error_list = []

        for method in conversion_methods:
            try:
                req.goal_constraints = method()
                break
            except (TypeError, AttributeError) as e:
                error_list.append(e)
                pass
        else:
            raise NotImplementedError(
                'Unknown type of goal: %s \nerrors: %s' % (
                    str(self._goal), str(error_list))
            )

        return req

    def _zero_header_time(self) -> None:
        if isinstance(self._goal, PoseStamped):
            self._goal.header.stamp = rospy.Time(0, 0)

    def _joint_values_to_constraint(self, joint_names=()):
        if not isinstance(
            self._goal,
            (
                tuple,
                list,
            ),
        ):
            raise TypeError(
                f'{self._goal} is not convertible into joint values.')

        joint_names = joint_names if len(
            joint_names) != 0 else self._active_joints
        joint_values = self._get_joint_pose()
        if len(joint_names) != len(joint_values):
            raise IndexError(
                'Given joint goal does not match the active joints ' +
                str(joint_names) + '.'
            )

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [
            JointConstraint(joint_name=name, position=value, weight=1)
            for name, value in zip(joint_names, joint_values)
        ]
        return [goal_constraints]

    def _pose_to_constraint(self):
        goal_pose = self._get_goal_pose()
        goal_constraints = Constraints()
        robot_reference_frame = self._robot_reference_frame
        goal_constraints.orientation_constraints.append(
            _to_ori_constraint(
                goal_pose, robot_reference_frame, self._target_link)
        )
        goal_constraints.position_constraints.append(
            _to_pose_constraint(
                goal_pose, robot_reference_frame, self._target_link)
        )
        return [goal_constraints]

    def _pose_stamped_to_constraint(self):
        self._reference_frame = (
            self._goal.header.frame_id if self._goal.header.frame_id != "" else _DEFAULT_BASE_LINK
        )
        self._zero_header_time()
        self._goal = self._goal.pose
        return self._pose_to_constraint()

    def _get_goal_pose(self) -> Pose:
        """Determines the goal pose for the given command."""
        if self._relative:
            self._goal = _pose_relative_to_absolute(
                self._start_pose, self._goal)

        if not self._reference_frame == _DEFAULT_BASE_LINK:
            return self._transform_frame(self._reference_frame, self._goal, self._robot_reference_frame)

        # in case of uninitialized orientation, set the goal orientation as current
        if _is_quaternion_initialized(self._goal.orientation):
            return self._goal
        else:
            return Pose(position=self._goal.position, orientation=self._start_pose.orientation)

    def _get_joint_pose(self) -> Goal:
        """Determines the joint goal for the given command."""
        goal_joint_state = (
            self._goal if not self._relative else map(
                add, self._goal, self._start_joint_states)
        )
        return goal_joint_state

    def _transform_frame(self, pose_frame: str, goal_pose_custom_ref: Pose, target_frame: str) -> Pose:
        """
        Transforms a pose from a custom reference frame to one in robot reference frame.
        :param pose_frame: is the custom reference frame of the pose.
        :param goal_pose_custom_ref: pose in the custom reference frame.
        :return: A goal pose in robot reference frame.
        """
        if not _is_quaternion_initialized(goal_pose_custom_ref.orientation):
            goal_pose_custom_ref.orientation = Quaternion(w=1)

        if pose_frame == target_frame:
            return goal_pose_custom_ref

        stamped = PoseStamped()
        stamped.header.frame_id = pose_frame
        stamped.pose = goal_pose_custom_ref

        return self._tf_buffer.transform_full(
            stamped, target_frame, rospy.Time(0), 'world', rospy.Duration(1)
        ).pose

    @staticmethod
    def _calc_acc_scale(vel_scale: float) -> float:
        raise NotImplementedError


class Ptp(BaseCmd):
    """Represents a single point-to-point (Ptp) command.
    A :py:class:`Ptp` command allows the user to quickly move the robot from its current position to a specified point
    in space (goal). The trajectory taken to reach the goal is defined by the underlying planning
    algorithms and cannot not be defined by the user.
    :param vel_scale: The velocity scaling factor allows to limit the highest possible axis velocity.
        The velocity scaling factor is a scalar. The value is applied to all axes.
        The value is given in percentage of the maximal velocity of an axis and has to be
        in range: (0,1]. The allowed axis velocity for each axis is calculated as follows:
            allowed axis velocity = vel_scale * maximal axis velocity
    :param acc_scale: The acceleration scaling factor allows to limit the highest possible axis acceleration.
        The acceleration scaling factor is a scalar value. The value is applied to all axes.
        The value is given in percentage of the maximal acceleration of an axis and
        has to be in range: (0,1]. The allowed axis acceleration for each axis is calculated as follows:
            allowed axis acceleration = vel_scale * maximal axis acceleration
        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:
            acc_scale = vel_scale * vel_scale
    """

    def __init__(
        self,
        vel_scale: float = _DEFAULT_JOINT_VEL_SCALE,
        acc_scale: Optional[float] = None,
        *args,
        **kwargs,
    ) -> None:
        acc_scale_final = acc_scale if acc_scale is not None else Ptp._calc_acc_scale(
            vel_scale)
        kwargs.update(acc_scale=acc_scale_final, vel_scale=vel_scale)
        super().__init__(*args, **kwargs)

        self._pipeline_id = 'pilz_industrial_motion_planner'
        self._planner_id = 'PTP'

    def __str__(self) -> str:
        out_str = super().__str__()
        if self._relative:
            out_str += ' relative: True'
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += ' Cartesian goal:\n' + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += ' joint goal: ' + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale: float) -> float:
        return vel_scale * vel_scale


class Lin(BaseCmd):
    """Represents a linear command.
    A :py:class:`Lin` command allows the user to move the robot from its current position to a specified point
    in space (goal). The trajectory taken to reach the goal is a straight line (in Cartesian space).
    :param vel_scale: The velocity scaling factor allows to limit the highest possible cartesian velocity
        of the TCP frame. The velocity scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian velocity and has to be
        in range: (0,1]. The allowed cartesian velocity of the TCP frame is calculated as follows:
            allowed cartesian velocity = vel_scale * maximal cartesian velocity
    :param acc_scale: The acceleration scaling factor allows to limit the highest possible cartesian acceleration
        of the TCP frame. The acceleration scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian acceleration and has to be
        in range: (0,1]. The allowed cartesian acceleration of the TCP frame is calculated as follows:
            allowed cartesian acceleration = acc_scale * maximal cartesian acceleration
        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:
            acc_scale = vel_scale
    """

    def __init__(
        self,
        vel_scale: float = _DEFAULT_CARTESIAN_VEL_SCALE,
        acc_scale: Optional[float] = None,
        *args,
        **kwargs,
    ) -> None:

        acc_scale_final = acc_scale if acc_scale is not None else Lin._calc_acc_scale(
            vel_scale)
        kwargs.update(acc_scale=acc_scale_final)

        super().__init__(*args, **kwargs)

        self._pipeline_id = 'pilz_industrial_motion_planner'
        self._planner_id = 'LIN'

    def __str__(self) -> str:
        out_str = BaseCmd.__str__(self)
        if self._relative:
            out_str += ' relative: True'
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += ' Cartesian goal:\n' + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += ' joint goal: ' + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale: float) -> float:
        return vel_scale


class Circ(BaseCmd):
    """Represents a circular command. A :py:class:`Circ` command allows the user to move the robot from its
    current position to a specified point in space (goal).
    The trajectory taken to reach the goal represents a circle (in Cartesian space). The circle is defined by the
    current position of the robot, the specified interim/center point and the goal position.
    :note:
        The circle can be completely defined by stating a interim `or` an center position.
        However, only one of both should be stated.
    :param interim: Position in cartesian space (geometry_msgs/Point),
        which lies on the circle on which the robot is supposed to move.
        The position has to lie between the current position of the robot and the goal position.
        The interim position indicates in which direction of the circle the robot is supposed to move.
    :param center: The center point (stated in Cartesian space) of the circle on which the robot is supposed to move.
        If the center point is given, the robot moves in the direction of the smallest angle to the goal.
    :param vel_scale: The velocity scaling factor allows to limit the highest possible cartesian velocity
        of the TCP frame. The velocity scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian velocity and has to be
        in range: (0,1]. The allowed cartesian velocity of the TCP frame is calculated as follows:
            allowed cartesian velocity = vel_scale * maximal cartesian velocity
    :param acc_scale: The acceleration scaling factor allows to limit the highest possible cartesian acceleration
        of the TCP frame. The acceleration scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian acceleration and has to be
        in range: (0,1]. The allowed cartesian acceleration of the TCP frame is calculated as follows:
            allowed cartesian acceleration = acc_scale * maximal cartesian acceleration
        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:
            acc_scale = vel_scale
    """

    def __init__(
        self,
        interim=None,
        center=None,
        vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE,
        acc_scale=None,
        *args,
        **kwargs,
    ) -> None:

        acc_scale_final = acc_scale if acc_scale is not None else Circ._calc_acc_scale(
            vel_scale)
        kwargs.update(acc_scale=acc_scale_final)

        super(Circ, self).__init__(vel_scale=vel_scale, *args, **kwargs)

        if center and not isinstance(center, Point):
            raise ValueError('center must be a Point')

        if interim and not isinstance(interim, Point):
            raise ValueError('interim must be a Point')

        if not isinstance(
            self._goal,
            (
                Pose,
                PoseStamped,
            ),
        ):
            raise ValueError('Goal must be a Pose or PoseStamped')

        self._pipeline_id = 'pilz_industrial_motion_planner'
        self._planner_id = 'CIRC'
        self._interim = interim
        self._center = center

    def __str__(self) -> str:
        out_str = super(Circ, self).__str__()

        if isinstance(self._goal, Pose) and self._goal is not None:
            out_str += ' goal:\n' + str(self._goal)
        if self._interim is not None:
            out_str += '\ninterim:\n' + str(self._interim)
        if self._center is not None:
            out_str += '\ncenter:\n' + str(self._center)
        return out_str

    __repr__ = __str__

    def _cmd_to_request(self, move_group: MoveGroupCommander) -> MotionPlanRequest:
        req = super(Circ, self)._cmd_to_request(move_group)

        if self._center is not None and self._interim is not None:
            raise NameError(
                'Both center and interim are set for circ command!')

        if self._center is None and self._interim is None:
            raise NameError(
                'Both center and interim are not set for circ command!')

        # Set the position constraint
        path_point = Pose()
        if self._center is not None:
            req.path_constraints.name = 'center'
            path_point.position = self._center
        else:
            req.path_constraints.name = 'interim'
            path_point.position = self._interim

        if self._reference_frame:
            path_point = self._transform_frame(
                self._reference_frame, path_point, self._robot_reference_frame)

        position_constraint = _to_pose_constraint(
            path_point, self._robot_reference_frame, self._target_link, float(
                '+inf')
        )

        req.path_constraints.position_constraints = [position_constraint]

        return req

    @staticmethod
    def _calc_acc_scale(vel_scale: float) -> float:
        return vel_scale


class _SequenceSubCmd:
    def __init__(self, cmd: BaseCmd, blend_radius: float = 0.0) -> None:
        self.cmd = cmd
        self.blend_radius = blend_radius

    def __str__(self) -> str:
        out_str = self.__class__.__name__
        out_str += ' - ' + str(self.cmd)
        out_str += '\nblend radius: ' + str(self.blend_radius)
        return out_str

    __repr__ = __str__


class Sequence(_AbstractCmd):
    """Represents an overall Sequence command. A :py:class:`Sequence` consists of one or more
    robot motion commands. All commands in a sequence are planned first. After all
    commands in a sequence are planned, they are executed.
    If the blending radius between two or more commands is greater than zero, the commands are blended
    together, in other words, the robot will not stop at the end of each command. To allow a smooth transition from
    one trajectory to the next (in case of blending), the original trajectories are altered slightly
    within the sphere defined by the blending radius.
    :note: In case the blend radius is zero, the robot executes the robot motion commands as if they are sent
           separately.
    :note: The robot always stops between gripper and non-gripper commands.
    :note: Gripper commands cannot be blended together.
    :note: In case the planning of a command in a sequence fails, non of the commands in the sequence are executed.
    """

    def __init__(self) -> None:
        super().__init__()
        # List of tuples containing commands and blend radii
        self.items: List[_SequenceSubCmd] = []

    def append(self, cmd: BaseCmd, blend_radius: float = 0.0) -> None:
        """Adds the given robot motion command to the sequence.
        :param cmd: The robot motion command which has to be added to the sequence.
            The blending happens between the specified command and the command following the specified command
            if a non-zero blend_radius is defined. Otherwise, if the blend radius is zero, the commands will
            execute consecutively.
            The blend radius preceding a gripper command is always ignored. The blend radius stated with a gripper
            command is also ignored.
        :type cmd: :py:class:`BaseCmd`
        :param blend_radius: The blending radius states how much the robot trajectory can deviate from the
            original trajectory (trajectory without blending) to blend the robot motion from one trajectory to the next.
            The blending happens inside a sphere with a radius specified by the blending radius. When the trajectory
            leaves the sphere the trajectory is back on the original trajectory.
        :type blend_radius: float
        :note:
            The last command of the sequence has to have zero blending radius which can be achieved
            by omitting the last blend radius argument.
        """
        self.items.append(_SequenceSubCmd(cmd, blend_radius))

    def insert(self, index: int, cmd: BaseCmd, blend_radius: float = 0.0) -> None:
        """Inserts the given robot motion command to the sequence at the specified index.
        :param cmd: The robot motion command which has to be added to the sequence.
            The blending happens between the specified command and the command following the specified command
            if a non-zero blend_radius is defined. Otherwise, if the blend radius is zero, the commands will
            execute consecutively.
            The blend radius preceding a gripper command is always ignored. The blend radius stated with a gripper
            command is also ignored.
        :type cmd: :py:class:`BaseCmd`
        :param blend_radius: The blending radius states how much the robot trajectory can deviate from the
            original trajectory (trajectory without blending) to blend the robot motion from one trajectory to the next.
            The blending happens inside a sphere with a radius specified by the blending radius. When the trajectory
            leaves the sphere the trajectory is back on the original trajectory.
        :type blend_radius: float
        :note:
            The last command of the sequence has to have zero blending radius which can be achieved
            by omitting the last blend radius argument.
        """

        self.items.insert(index, _SequenceSubCmd(cmd, blend_radius))

    def _get_sequence_request(self, move_group: MoveGroupCommander) -> List[MotionSequenceItem]:
        items = []

        for item in self.items:
            # Create and fill request
            curr_sequence_req = MotionSequenceItem()
            curr_sequence_req.blend_radius = item.blend_radius

            # Fill MotionPlanRequest
            curr_sequence_req.req = item.cmd._cmd_to_request(move_group)

            items.append(curr_sequence_req)

        return items

    def __str__(self) -> str:
        out_str = _AbstractCmd.__str__(self)
        out_str += ':\n'
        for item in self.items:
            out_str += str(item)
            out_str += '\n'
        return out_str

    __repr__ = __str__


class MoveSequenceFailed(MoveItCommanderException):
    pass


class SequencePlanningError(MoveItCommanderException):
    # Map numerical code to some human readable text
    _error_code_map = {
        getattr(MoveItErrorCodes, attr): attr
        for attr in dir(MoveItErrorCodes)
        if attr[0].isalpha() and attr[0].isupper()
    }

    def __init__(self, error_code: int) -> None:
        self.error_code = error_code
        self.message = self._error_code_map[error_code]
        super().__init__(self.message)


class MoveGroupSequence:
    _trajectories: List[RobotTrajectory] = []
    _target_joint_values = Optional[List[float]]

    JOINT_ERROR_THRESHOLD = 0.001
    JOINT_MEAN_ERROR_THRESHOLD = 0.0005

    def __init__(self, move_group: MoveGroupCommander) -> None:
        rospy.wait_for_service('plan_sequence_path', 30)
        self._service = rospy.ServiceProxy(
            'plan_sequence_path', GetMotionSequence)
        self._move_group = move_group

    def _reset(self) -> None:
        self._trajectories = []
        self._target_joint_values = None

    def plan(self, command: _AbstractCmd) -> Tuple[bool, List[RobotTrajectory], float, int]:
        self._reset()

        if isinstance(command, Sequence):
            items = command._get_sequence_request(self._move_group)
        elif isinstance(command, BaseCmd):
            items = [MotionSequenceItem(
                req=command._cmd_to_request(self._move_group))]
        else:
            raise ValueError('Unsupported command: ', command)

        response: GetMotionSequenceResponse = self._service(
            request=MotionSequenceRequest(items=items)
        )

        success = response.response.error_code.val == MoveItErrorCodes.SUCCESS
        if not success:
            raise SequencePlanningError(response.response.error_code.val)

        self._trajectories = response.response.planned_trajectories

        return success, self._trajectories, response.response.planning_time, response.response.error_code.val

    def execute(self, trajectories: Optional[List[RobotTrajectory]] = None) -> None:
        to_execute = trajectories or self._trajectories

        for trajectory in to_execute:
            self._move_group.execute(trajectory, wait=True)

            if not self._reached_pose(trajectory.joint_trajectory.points[-1].positions):
                raise MoveSequenceFailed(
                    "The robot did not reach it's target pose")

    def _reached_pose(self, target_joint_values: List[float]) -> bool:
        distance = [
            abs(target - current)
            for target, current in zip(
                target_joint_values, self._move_group.get_current_joint_values()
            )
        ]

        if [i for i in distance if i >= self.JOINT_ERROR_THRESHOLD]:
            return False

        return sum(distance) / len(target_joint_values) < self.JOINT_MEAN_ERROR_THRESHOLD


def _to_ori_constraint(
    pose, reference_frame, link_name, orientation_tolerance=_DEFAULT_ORIENTATION_TOLERANCE
):
    """Returns an orientation constraint suitable for ActionGoal's."""
    ori_con = OrientationConstraint()
    ori_con.header.frame_id = reference_frame
    ori_con.link_name = link_name
    ori_con.orientation = pose.orientation
    ori_con.absolute_x_axis_tolerance = orientation_tolerance
    ori_con.absolute_y_axis_tolerance = orientation_tolerance
    ori_con.absolute_z_axis_tolerance = orientation_tolerance
    ori_con.weight = 1
    return ori_con


def _to_pose_constraint(
    pose, reference_frame, link_name, position_tolerance=_DEFAULT_POSITION_TOLERANCE
):
    """Returns an position constraint suitable for ActionGoal's."""
    pos_con = PositionConstraint()
    pos_con.header.frame_id = reference_frame
    pos_con.link_name = link_name
    pos_con.constraint_region.primitive_poses.append(pose)
    pos_con.weight = 1

    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions.append(position_tolerance)

    pos_con.constraint_region.primitives.append(region)

    return pos_con


def _is_quaternion_initialized(quaternion):
    """Check if the quaternion is initialized"""
    return quaternion != Quaternion()  # check, if all fields are zero


def _pose_relative_to_absolute(current_pose, relative_pose):
    """Add the offset relative_pose to current_pose and return an absolute goal pose"""
    goal_pose = deepcopy(current_pose)

    # translation
    goal_pose.position.x += relative_pose.position.x
    goal_pose.position.y += relative_pose.position.y
    goal_pose.position.z += relative_pose.position.z

    # rotation
    a_cur, b_cur, c_cur = transformations.euler_from_quaternion(
        [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ],
        axes=_AXIS_SEQUENCE,
    )

    a, b, c = transformations.euler_from_quaternion(
        [
            relative_pose.orientation.x,
            relative_pose.orientation.y,
            relative_pose.orientation.z,
            relative_pose.orientation.w,
        ],
        axes=_AXIS_SEQUENCE,
    )

    # choose shorter distance for relative movement:
    # if we set b -> -b and rotate a and c by 180 degrees,
    # we obtain the same rotation.
    # But we have to make sure, to stay within the -pi,pi range.
    # For b we explicitly allow (-pi,pi) here as well, since the angles
    # are an offset to the current angle, so that b may be negative.
    a2, b2, c2 = min([pi + a, -pi + a], key=abs), - \
        b, min([pi + c, -pi + c], key=abs)
    if abs(a) + abs(b) + abs(c) > abs(a2) + abs(b2) + abs(c2):
        a, b, c = a2, b2, c2

    goal_pose.orientation = from_euler(a + a_cur, b + b_cur, c + c_cur)

    return goal_pose


def from_euler(a, b, c):
    """Convert euler angles into a `geometry.msg.Quaternion`.
    Pass euler angles a, b, c in intrinsic ZYZ convention (in radians).
    Use this function to fill pose values for :py:class:`Ptp` / :py:class:`Lin` commands:
    ::
        r.move(Ptp(goal=Pose(position=Point(0.6, -0.3, 0.2), orientation=from_euler(0, pi, 0))))
    :param a: rotates around the z-axis.
    :param b: rotates around the new y-axis.
    :param c: rotates around the new z-axis.
    :note:
        e.g. (0, pi, 0) orients the tool downwards,
        (pi/2., pi/2., 0) horizontal west
    """
    return Quaternion(*transformations.quaternion_from_euler(a, b, c, axes=_AXIS_SEQUENCE))
