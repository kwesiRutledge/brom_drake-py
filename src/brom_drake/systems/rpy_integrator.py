"""
Extended Integrator System

This module provides an integrator system that extends the basic Drake Integrator
with additional functionality for integration without wrap-around.
"""

from pydrake.systems.framework import LeafSystem, BasicVector, InputPort


class RPYIntegrator(LeafSystem):
    """
    **Description**

    A system that integrates roll, pitch, and yaw velocity signals to produce
    roll, pitch, and yaw position estimates without angle wrap-around.

    This system is useful when you want to track cumulative rotation angles
    beyond the typical [-π, π] range, allowing for unbounded angle tracking
    (e.g., roll = 10π is valid).

    **Notes**

    - This system integrates the time derivatives of RPY angles (ṙoll, ṗitch, ẏaw).
    - If you have body-frame angular velocities (ωx, ωy, ωz), you must convert them
      to RPY rates first using the appropriate transformation matrix.
    - Initial conditions can be set through the context's continuous state.

    **Example**

    .. code-block:: python

        from pydrake.all import DiagramBuilder
        import numpy as np

        builder = DiagramBuilder()

        # Create the integrator
        rpy_integrator = builder.AddSystem(RPYIntegrator())

        # Connect current RPY to set initial state
        # builder.Connect(current_rpy_source.get_output_port(),
        #                 rpy_integrator.get_input_port(0))

        # Connect velocity source to integrator
        # builder.Connect(velocity_source.get_output_port(),
        #                 rpy_integrator.get_input_port(1))
    """

    def __init__(self):
        """
        Initializes the RPY integrator system.
        """
        LeafSystem.__init__(self)

        # Declare input port for current RPY (used to set initial state)
        self._rpy_port: InputPort = self.DeclareVectorInputPort("rpy", BasicVector(3))

        # Declare input port for RPY velocities
        self._rpy_velocity_port: InputPort = self.DeclareVectorInputPort(
            "rpy_velocity", BasicVector(3)
        )

        # Declare continuous state for integrated RPY values
        self.DeclareContinuousState(3)

        # Declare output port for integrated RPY
        self.DeclareVectorOutputPort("rpy", BasicVector(3), self.CalcOutput)

    def get_rpy_input_port(self):
        """
        Returns the input port for current RPY values (used to set initial state).
        """
        return self._rpy_port

    def get_rpy_velocity_input_port(self):
        """
        Returns the input port for RPY velocity values.
        """
        return self._rpy_velocity_port

    def SetDefaultState(self, context, state):
        """
        Set the default initial state for the integrator from the RPY input port.
        """
        # Get the initial RPY from the input port
        initial_rpy = self._rpy_port.Eval(context)
        state.get_mutable_continuous_state().SetFromVector(initial_rpy)

    def DoCalcTimeDerivatives(self, context, derivatives):
        """
        Calculate the time derivatives (which are just the input velocities).
        """
        # Get the input velocities (from port 1)
        rpy_velocity = self._rpy_velocity_port.Eval(context)

        # Set the derivatives equal to the input velocities
        derivatives.get_mutable_vector().SetFromVector(rpy_velocity)

    def CalcOutput(self, context, output):
        """
        Output the current integrated RPY values.
        """
        # Get the current state (integrated RPY)
        rpy = context.get_continuous_state_vector().CopyToVector()

        # Set the output
        output.SetFromVector(rpy)
