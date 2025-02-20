import networkx as nx
import numpy as np
from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    LeafSystem,
    PortDataType,
)
from typing import List, Union

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm.fsm_transition_condition import (
    FSMTransitionCondition, FSMTransitionConditionType
)
from brom_drake.utils.leaf_systems.network_fsm.fsm_edge_definition import FSMEdgeDefinition
from brom_drake.utils.leaf_systems.network_fsm.errors import (
    EdgeContainsNoConditionsError,
    EdgeContainsNoLabelsError,
    MultipleConnectedComponentsError,
    NumberOfStartNodesError,
    OutputPortNotInitializedError,
)

class NetworkXFSM(LeafSystem):
    def __init__(
        self,
        fsm_graph: nx.DiGraph,
    ):
        LeafSystem.__init__(self)

        # Prepare to Create FSM from Graph
        self.fsm_graph = fsm_graph

        # Check if the FSM is valid
        exceptions_during_check = self.supports_this_digraph(self.fsm_graph)
        if exceptions_during_check is not None:
            raise exceptions_during_check

        # Collect all edge definitions from the graph
        self.edge_definitions = self.collect_edge_definitions(self.fsm_graph)

        # Create the Input and Output Ports required by the graph
        # NOTE: We will create more than just these ports.
        self.input_port_dict = {}
        self.derive_input_ports_from_graph()

        self.output_port_dict = {}
        self.derive_output_ports_from_graph()

        # Initialize the system in the initial state
        self.t_start_of_current_state = 0.0
        self.current_state = None
        self.initialize_fsm_state()

    def advance_state_if_necessary(self, context: Context, debug_flag: bool = False):
        """
        Description
        -----------
        Checks the current values of all relevant inputs
        and advances the state if necessary according to the
        conditions established in the FSM graph.
        """
        # Setup
        s_t = self.current_state
        input_port_dict = self.input_port_dict
        edge_definitions = self.edge_definitions

        # Collect all edge definitions for the current state
        s_t_edge_definitions = [
            edge_definition
            for edge_definition in edge_definitions
            if edge_definition.src == s_t
        ]

        # print(f"Current state: {s_t}")
        # print(f"Edge definitions for state {s_t}: {s_t_edge_definitions}")

        # Iterate through all edge definitions to find the next state (if it exists)
        edge_conditions_satisfied = [False for _ in range(len(s_t_edge_definitions))]
        for ii, edge_definition in enumerate(s_t_edge_definitions):
            # Check all conditions in the edge definition
            all_conditions_met_for_ii = True
            for jj, condition in enumerate(edge_definition.conditions):
                # Check the condition
                condition_ii_met = self.evaluate_transition_condition(
                    condition, context,
                )

                # If the condition is not met,
                # then mark the edge as not satisfied and break the inner loop.
                if not condition_ii_met:
                    all_conditions_met_for_ii = False
                    break

            # If all conditions are met, then mark the edge as satisfied
            edge_conditions_satisfied[ii] = all_conditions_met_for_ii

        if debug_flag:
            print(f"Edge conditions satisfied: {edge_conditions_satisfied}")

        # Check the number of conditions that are satisfied
        num_conditions_satisfied = sum(edge_conditions_satisfied)
        if num_conditions_satisfied == 0:
            # If no conditions are satisfied, then stay in the same state
            next_state = s_t
        elif num_conditions_satisfied == 1:
            # If exactly one condition is satisfied, then transition to the next state
            next_state = s_t_edge_definitions[edge_conditions_satisfied.index(True)].dst

            # And update transition time
            self.t_start_of_current_state = context.get_time()
        else:
            # If more than one condition is satisfied, then raise an error
            raise ValueError(
                f"More than one condition is satisfied for state {s_t} in the FSM graph."
            )
            # TODO(kwesi): make this error more verbose to explain to people how to create
            #              mutually exclusive conditions for transition.
        
        # Update the current state
        self.current_state = next_state

    def CalcFSMState(self, context: Context, fsm_state: BasicVector):
        """
        Description
        -----------
        This function calculates the current state of the FSM.
        It uses the various Edge Definitions to determine the next state,
        based on the current state and the input values.
        """
        # Call advance function
        self.advance_state_if_necessary(context)

        # Set the FSM state
        fsm_state.SetFromVector(
            np.array([self.current_state])
        )

    def collect_edge_definitions(
        self,
        fsm_graph: nx.DiGraph,
        debug_flag: bool = False,
    ) -> List[FSMEdgeDefinition]:
        """
        Description
        -----------
        This function collects the edge definitions from the FSM graph.
        It uses the data from the graph to create an internal representation
        (FSMEdgeDefinition) for each edge in the graph.
        """
        # Setup
        edge_definitions = []

        # Iterate through all edges of the FSM graph
        # and identify:
        #   - The input ports that are required for each edge
        for ii, edge_ii in enumerate(fsm_graph.edges):
            src_ii, dst_ii = edge_ii 
            if debug_flag:
                print(f"Edge: {src_ii} -> {dst_ii}")

            edge_data = fsm_graph.edges[edge_ii]

            # Get the conditions for each edge
            ii_th_conditions = edge_data["conditions"]
            
            # Create the FSMEdgeDefinition object
            edge_definition = FSMEdgeDefinition(
                conditions=ii_th_conditions,
                src=src_ii,
                dst=dst_ii,
            )

            # Add the edge definition to the list
            edge_definitions.append(edge_definition)

        return edge_definitions

    def derive_input_ports_from_graph(self, debug_flag: bool = False):
        """
        Description
        -----------
        This function creates the input ports for the FSM.
        """
        # Setup
        fsm_graph = self.fsm_graph

        # Iterate through all edges of the FSM graph
        # and identify:
        #   - The input ports that are required for each edge
        input_port_definitions = []
        for ii, edge_ii in enumerate(fsm_graph.edges):
            edge_data = fsm_graph.edges[edge_ii]
            ii_th_conditions = edge_data["conditions"]

            # Get the input port names
            for jj, condition_jj in enumerate(ii_th_conditions):
                if debug_flag:
                    print(f"Condition {jj} of edge {ii} has:\n" +
                          f"- Input port \"{condition_jj.input_port_name}\"\n" +
                          f"- Condition value \"{condition_jj.condition_value}\"" +
                          f"- Condition type \"{condition_jj.type}\"")

                if not condition_jj.requires_input_port():
                    if debug_flag:
                        print(f"Condition {jj} does not require an input port.")

                    continue

                input_port_name = condition_jj.input_port_name
                if input_port_name not in [port_defn[0] for port_defn in input_port_definitions]:
                    input_port_definitions.append(
                        (input_port_name, condition_jj.condition_value)
                    )

        # After collecting all input port names, create the input ports
        for ii, input_port_definition in enumerate(input_port_definitions):
            # Debug messages
            if debug_flag:
                print(f"Found definition for port \"{input_port_definition}\" with example value.")

            # Extract values from the input port definition
            port_name_ii, example_value_ii = input_port_definition

            # Check to see if port exists already
            if port_name_ii in self.input_port_dict:
                continue

            # Create the port
            if type(example_value_ii) == np.ndarray:
                # Compute size of the input port
                input_port_size = len(example_value_ii)

                # Create port
                self.input_port_dict[port_name_ii] = self.DeclareVectorInputPort(
                    port_name_ii,
                    input_port_size,
                )
            elif type(example_value_ii) == bool:
                # Create port
                self.input_port_dict[port_name_ii] = self.DeclareAbstractInputPort(
                    port_name_ii,
                    AbstractValue.Make(example_value_ii),
                )
            else:
                # Raise an error
                raise ValueError(
                    f"Input port type \"{type(example_value_ii)}\" not supported by" + \
                    " NetworkXFSM.create_input_ports()."
                )

    def derive_output_ports_from_graph(self, debug_flag: bool = False):
        """
        Description
        -----------
        This method uses the fsm_graph to infer what output ports
        should be created for the FSM.

        Note
        ----
        For speed, this assumes that the graph has already been checked
        for validity.
        Also, this method assumes that all necessary output ports can be
        inferred from the initial node.
        """
        # Setup
        fsm_graph = self.fsm_graph

        # Get the initial node
        start_nodes = [
            node
            for node in fsm_graph.nodes
            if len(list(fsm_graph.predecessors(node))) == 0
        ]
        initial_node = start_nodes[0]
        initial_node_data = fsm_graph.nodes[initial_node]

        # Create output ports
        for output_port_definition in initial_node_data["outputs"]:
            # Extract values from the output port definition
            port_name_ii = output_port_definition.output_port_name
            output_value_ii = output_port_definition.output_port_value

            # Check to see if port exists already
            if port_name_ii in self.output_port_dict:
                continue

            # Create the port
            if type(output_value_ii) == np.ndarray:
                # Compute size of the output port
                output_port_size = len(output_value_ii)

                # Define dummy function for output port
                def dummy_output_function_ii(context: Context, output: BasicVector):
                    self.advance_state_if_necessary(context)
                    output.SetFrom(output_value_ii)

                # Create port
                self.output_port_dict[port_name_ii] = self.DeclareVectorOutputPort(
                    port_name_ii,
                    output_port_size,
                    dummy_output_function_ii,
                )
            elif type(output_value_ii) == bool:
                # Create dummy function for output port
                def dummy_output_function_ii(context: Context, output: AbstractValue):
                    self.advance_state_if_necessary(context)
                    output.SetFrom(AbstractValue.Make(output_value_ii))

                # Create port
                self.output_port_dict[port_name_ii] = self.DeclareAbstractOutputPort(
                    port_name_ii,
                    lambda: AbstractValue.Make(output_value_ii),
                    dummy_output_function_ii,
                )
            elif (type(output_value_ii) == int) or (type(output_value_ii) == float):
                # Create a np.array from the scalar value
                output_value_ii = np.array([output_value_ii])

                # Create dummy function for output port
                def dummy_output_function_ii(context: Context, output: BasicVector):
                    self.advance_state_if_necessary(context)
                    output.SetFrom(output_value_ii)

                # Create vector port for new output
                self.output_port_dict[port_name_ii] = self.DeclareVectorOutputPort(
                    port_name_ii,
                    1,
                    dummy_output_function_ii,
                )
            else:
                # Raise an error
                raise ValueError(
                    f"Output port type \"{type(output_value_ii)}\" not supported by" + \
                    " NetworkXFSM.create_output_ports()."
                )

    def evaluate_transition_condition(self, condition: FSMTransitionCondition, context: Context) -> bool:
        """
        Description
        -----------
        This function evaluates the condition for a given input port value.
        """
        # Setup

        # port_is_abstract = self.input_port_dict[input_port_name].get_data_type() == PortDataType.kAbstractValued
        # if port_is_abstract:
        #     input_port_value = input_port_value.get_value()

        # Evaluate the condition
        if condition.type == FSMTransitionConditionType.kAfterThisManySeconds:
            # Check the amount of time elapsed
            elapsed_time = context.get_time() - self.t_start_of_current_state
            return elapsed_time >= condition.condition_value
        else:
            # Evaluate the comparison
            input_port_name = condition.input_port_name
            input_port_value = self.input_port_dict[input_port_name].Eval(context)

            return condition.evaluate_comparison(input_port_value)


    def initialize_fsm_state(self):
        """
        Description
        -----------
        This function initializes the FSM state by:
        - Creating the output port for the fsm state,
        - Setting the initial state of the FSM (as an internal variable)
        """
        # Setup
        fsm_graph = self.fsm_graph

        # Get the initial node
        start_nodes = [
            node
            for node in fsm_graph.nodes
            if len(list(fsm_graph.predecessors(node))) == 0
        ]
        initial_node = start_nodes[0]

        # Set the initial state
        self.current_state = initial_node

        # Create the output port for the FSM state
        self.DeclareVectorOutputPort(
            "fsm_state",
            1,
            self.CalcFSMState,
        )

    @staticmethod
    def supports_this_digraph(
        digraph: nx.DiGraph,
        debug_flag: bool = False,
    ) -> Union[None, ValueError]:
        """
        Description
        -----------
        This function checks if the given directed graph is a valid FSM.

        Parameters
        ----------
        digraph : nx.DiGraph
            The directed graph to check.

        Returns
        -------
        recognized_exception: Union[None, ValueError]
            Either an error if the graph is not a valid FSM, or None if the graph is valid.
        """

        # Check if the graph is connected
        connected_components = list(nx.weakly_connected_components(digraph))
        if debug_flag:
            print(f"Found {len(connected_components)} connected components.")
        
        if len(connected_components) > 1:
            return MultipleConnectedComponentsError(connected_components)


        # Check if the graph has a single start node
        start_nodes = [
            node
            for node in digraph.nodes
            if len(list(digraph.predecessors(node))) == 0
        ]
        if debug_flag:
            print(f"Found {len(start_nodes)} start nodes.")

        if len(start_nodes) != 1:
            return NumberOfStartNodesError(start_nodes)

        # Check that all edges have a unique label
        # TODO: List all edges and verify that each edge has at least one label
        for edge in digraph.edges:
            # print(edge)
            # print(digraph.edges[edge])
            if digraph.edges[edge] == {}:
                return EdgeContainsNoLabelsError(edge)
            
            conditions = digraph.edges[edge]["conditions"]
            if len(conditions) == 0:
                return EdgeContainsNoConditionsError(edge)

        # Check that the set of output ports (defined by the labels of the initial node)
        # contains ALL the output ports defined in the FSM
        initial_node = start_nodes[0]
        initial_node_data = digraph.nodes[initial_node]
        output_ports_defined = initial_node_data["outputs"]
        output_ports_defined = [output_defn.output_port_name for output_defn in output_ports_defined]
        if debug_flag:
            print(f"Output ports defined by the initial node: {output_ports_defined}")

        # Check that all output ports are defined in the FSM
        for node in digraph.nodes:
            node_data = digraph.nodes[node]
            if "outputs" not in node_data:
                continue

            output_ports = node_data["outputs"]
            output_ports = [output_defn.output_port_name for output_defn in output_ports]
            for output_port in output_ports:
                if output_port not in output_ports_defined:
                    return OutputPortNotInitializedError(node, output_port, output_ports_defined)

        # All checks passed
        return None