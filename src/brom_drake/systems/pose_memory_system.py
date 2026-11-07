from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.systems.framework import (
    Context,
    InputPort,
    LeafSystem,
    OutputPort,
    State,
)


class PoseMemorySystem(LeafSystem):
    """
    *Description*

    This LeafSystem is a "sample-and-hold" for poses (i.e., AbstractValue
    objects containing RigidTransform values).

    - While the ``hold`` input is False, the system passes ``pose_in`` through
      to ``pose_out`` and remembers it.
    - While the ``hold`` input is True, the system outputs the last pose that it
      remembered before ``hold`` became True (ignoring ``pose_in``).

    The remembered pose is stored in the system's (abstract) state, so it is reset
    whenever a new Context is created. It is updated at the start of every simulation step
    (and when the simulator is initialized).

    *Diagram*

    The LeafSystem's input and output ports can be illustrated with the following block: ::

                            |---------------|
        pose_in ----------->|               |
        (RigidTransform)    | Pose          |
                            | Memory        | ----> pose_out
        hold -------------->| System        |       (RigidTransform)
        (bool)              |               |
                            |---------------|

    *Usage*

    Holding the desired pose of a gripper in the world frame, once a
    finite state machine says to stop tracking a trajectory: ::

        memory_system = builder.AddSystem(PoseMemorySystem())
        builder.Connect(trajectory_source.get_output_port(), memory_system.get_pose_in_port())
        builder.Connect(fsm.GetOutputPort("hold_gripper_pose"), memory_system.get_hold_port())
    """

    def __init__(self):
        LeafSystem.__init__(self)

        # Define Input Ports
        self._pose_in_port = self.DeclareAbstractInputPort(
            "pose_in", AbstractValue.Make(RigidTransform())
        )
        self._hold_port = self.DeclareAbstractInputPort(
            "hold", AbstractValue.Make(False)
        )

        # Define State (i.e., the remembered pose)
        self._remembered_pose_index = self.DeclareAbstractState(
            AbstractValue.Make(RigidTransform())
        )

        # Remember the input pose at initialization and at the start of each step
        self.DeclareInitializationUnrestrictedUpdateEvent(self.UpdateRememberedPose)
        self.DeclarePerStepUnrestrictedUpdateEvent(self.UpdateRememberedPose)

        # Define Output Port
        self._pose_out_port = self.DeclareAbstractOutputPort(
            "pose_out",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcOutputPose,
        )

    def UpdateRememberedPose(self, context: Context, state: State):
        """
        *Description*

        This callback function remembers the current input pose,
        unless the system is holding.
        """
        # Setup
        hold: bool = self._hold_port.Eval(context)

        if hold:
            return  # Keep the remembered pose

        # Otherwise, remember the current input pose
        pose_in: RigidTransform = self._pose_in_port.Eval(context)

        assert (
            type(pose_in) is RigidTransform
        ), f'Expected pose_in port to contain "RigidTransform" objects, but received type {type(pose_in)}'

        state.get_mutable_abstract_state(self._remembered_pose_index).set_value(pose_in)

    def CalcOutputPose(self, context: Context, output: AbstractValue):
        """
        *Description*

        This callback function outputs the input pose (if not holding)
        or the remembered pose (if holding).
        """
        # Setup
        hold: bool = self._hold_port.Eval(context)

        if hold:
            pose_out = context.get_abstract_state(
                self._remembered_pose_index
            ).get_value()
        else:
            pose_out = self._pose_in_port.Eval(context)

        output.set_value(pose_out)

    def get_hold_port(self) -> InputPort:
        return self._hold_port

    def get_pose_in_port(self) -> InputPort:
        return self._pose_in_port

    def get_pose_out_port(self) -> OutputPort:
        return self._pose_out_port
