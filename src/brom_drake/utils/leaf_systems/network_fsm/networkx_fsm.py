import logging
import networkx as nx
import numpy as np
from pathlib import Path
from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    DiscreteValues,
    LeafSystem,
    PortDataType,
)
from typing import Callable, Dict, List, Union

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
from brom_drake.utils.leaf_systems.network_fsm.config import NetworkXFSMConfig
from brom_drake.directories import DEFAULT_NETWORKX_FSM_DIR

class NetworkXFSM(LeafSystem):
    def __init__(
        self,
        fsm_graph: nx.DiGraph,
        config: NetworkXFSMConfig = NetworkXFSMConfig(),
    ):
        LeafSystem.__init__(self)

        # Setup
        self.config = config

        # Prepare to Create FSM from Graph
        self.fsm_graph = fsm_graph

        # Configure the logger
        self.logger = self.create_logger()

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

        self.last_output_value = {}
        self.initialize_last_output_value_map()

        self.output_port_dict = {}
        self.derive_output_ports_from_graph()

        # Initialize the system in the initial state
        self.t_start_of_current_state = 0.0
        self.current_state_index = None
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
        s_t = context.get_discrete_state_vector().GetAtIndex(0)
        input_port_dict = self.input_port_dict
        edge_definitions = self.edge_definitions

        # Collect all edge definitions for the current state
        s_t_edge_definitions = [
            edge_definition
            for edge_definition in edge_definitions
            if edge_definition.src == s_t
        ]

        if debug_flag:
            self.log(f"Current state: {s_t}", level=logging.DEBUG)
            self.log(f"Edge definitions for state {s_t}: {s_t_edge_definitions}", level=logging.DEBUG)

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
            self.log(
                f"Edge conditions satisfied: {edge_conditions_satisfied}",
                level=logging.DEBUG,
            )

        # Check the number of conditions that are satisfied
        num_edges_that_are_triggered = sum(edge_conditions_satisfied)
        if (num_edges_that_are_triggered >= 1) and debug_flag:
            self.log(
                f"Number of conditions satisfied: {num_edges_that_are_triggered} at time {context.get_time()}",
                level=logging.DEBUG,
            )

        if num_edges_that_are_triggered == 0:
            # If no conditions are satisfied, then stay in the same state
            next_state = s_t
        elif num_edges_that_are_triggered == 1:
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
        
        if debug_flag:
            print(f"Transitioning from state {s_t} to state {next_state} at time {context.get_time()}")

        # Update the current state
        # self.current_state = next_state
        context.SetDiscreteState(
            np.array([next_state])
        )

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

        s_t = context.get_discrete_state_vector()

        # Set the FSM state
        fsm_state.SetFrom(s_t)

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

        # Announce the beginning of the edge collection
        self.logger.info(
            "Collecting edge definitions from the FSM graph...",
        )

        # Iterate through all edges of the FSM graph
        # and identify:
        #   - The input ports that are required for each edge
        for ii, edge_ii in enumerate(fsm_graph.edges):
            src_ii, dst_ii = edge_ii 
            self.log(f"- Edge: {src_ii} -> {dst_ii}", level=logging.INFO)

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

    def create_logger(self) -> logging.Logger:
        """
        Description
        -----------
        This function configures the logger for the FSM.
        It sets the logger to log messages at the INFO level.
        """
        # Setup
        config = self.config

        # Create logger
        logger = logging.getLogger(config.name + " logger")

        # Create a console handler
        if config.show_logs_in_terminal:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.WARNING)

            # Create a formatter and set it for the handler
            formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(name)s | %(message)s')
            console_handler.setFormatter(formatter)

            # Add the handler to the logger
            logger.addHandler(console_handler)

        # Create a file handler
        if config.log_file_name is not None:
            # Create directory if it doesn't exist
            networkx_fsm_log_dir = Path(DEFAULT_NETWORKX_FSM_DIR)
            networkx_fsm_log_dir.mkdir(parents=True, exist_ok=True)

            # Create a file handler
            file_handler = logging.FileHandler(
                filename=DEFAULT_NETWORKX_FSM_DIR + "/" + config.log_file_name,
                mode='w'
            )
            file_handler.setLevel(logging.INFO) # Set to DEBUG to get REALLY verbose logs

            # Create a formatter and set it for the handler
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            file_handler.setFormatter(formatter)

            # Add the handler to the logger
            logger.addHandler(file_handler)

        # Avoid duplicate logs
        logger.propagate = False

        # Make sure the logger responds to all messages of level DEBUG and above
        logger.setLevel(logging.DEBUG)  # Set to DEBUG to capture all messages

        return logger

    def create_output_port_function(
        self,
        port_name_ii: str,
        create_abstract_port: bool = False
    ) -> Callable[[Context, AbstractValue], None]:
        """
        Description
        -----------
        This function creates the output port function for the FSM.
        """
        # Setup

        # Create the output port function
        update_map_ii = self.derive_output_update_map(port_name_ii) # (This describes when to update the value of port_ii)

        if create_abstract_port:
            def dummy_output_function_ii(context: Context, output: AbstractValue):
                self.advance_state_if_necessary(context)
                s_t = context.get_discrete_state_vector().GetAtIndex(0)

                # Check to see if the state value has changed
                if s_t in update_map_ii:
                    # Update the output value
                    self.last_output_value[port_name_ii] = update_map_ii[s_t]

                self.logger.debug(
                    f"Output value for port {port_name_ii} is {self.last_output_value[port_name_ii]} at time {context.get_time()}",
                    )
                output.SetFrom(AbstractValue.Make(self.last_output_value[port_name_ii]))
        else:
            def dummy_output_function_ii(context: Context, output: BasicVector):
                self.advance_state_if_necessary(context)
                s_t = context.get_discrete_state_vector().GetAtIndex(0)

                # Check to see if the state value has changed
                if s_t in update_map_ii:
                    # Update the output value
                    self.last_output_value[port_name_ii] = update_map_ii[s_t]
                    
                output.SetFrom(self.last_output_value[port_name_ii])


        return dummy_output_function_ii

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
                # Log what we've found
                self.logger.info(f"Condition {jj} of edge {ii} has:\n" +
                        f"- Input port \"{condition_jj.input_port_name}\"\n" +
                        f"- Condition value \"{condition_jj.condition_value}\"" +
                        f"- Condition type \"{condition_jj.type}\"",
                        )

                if not condition_jj.requires_input_port():
                    self.log(f"Condition {jj} does not require an input port.", level=logging.INFO)

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
        for output_index_ii, output_port_definition in enumerate(initial_node_data["outputs"]):
            # Extract values from the output port definition
            port_name_ii = output_port_definition.output_port_name
            output_value_ii = output_port_definition.output_port_value

            # Check to see if port exists already
            if port_name_ii in self.output_port_dict:
                continue

            # Create the port
            update_map_ii = self.derive_output_update_map(port_name_ii) # (This describes when to update the value of port_ii)
            if type(output_value_ii) == np.ndarray:
                # Compute size of the output port
                output_port_size = len(output_value_ii)

                # Create port
                self.output_port_dict[port_name_ii] = self.DeclareVectorOutputPort(
                    port_name_ii,
                    output_port_size,
                    self.create_output_port_function(port_name_ii, create_abstract_port=False),
                )
            elif (type(output_value_ii) == bool) or (type(output_value_ii) == str):
                self.output_port_dict[port_name_ii] = self.DeclareAbstractOutputPort(
                    port_name_ii,
                    lambda: AbstractValue.Make(output_value_ii),
                    self.create_output_port_function(port_name_ii, create_abstract_port=True),
                )
            elif (type(output_value_ii) == int) or (type(output_value_ii) == float):
                # Create a np.array from the scalar value
                output_value_ii = np.array([output_value_ii])

                # Create vector port for new output
                self.output_port_dict[port_name_ii] = self.DeclareVectorOutputPort(
                    port_name_ii,
                    1,
                    self.create_output_port_function(port_name_ii, create_abstract_port=False),
                )
            else:
                # Raise an error
                raise ValueError(
                    f"Output port type \"{type(output_value_ii)}\" not supported by" + \
                    " NetworkXFSM.create_output_ports()."
                )

    def derive_output_update_map(
        self,
        port_name_ii: str
    ) -> Dict[int, Union[str, int]]:
        """
        Description
        -----------
        This method constructs a small mapping of WHEN to change the value of
        port_name_ii.
        The key to each element in the mapping is a value of the state (i.e.,
        on this state, change the value of port_name_ii to this value) and
        the value is the value that the state should be changed to when we reach there.

        Arguments
        ---------
        port_name_ii: str
            The name of the port for which this mapping applies.
        """
        # Setup
        fsm_graph = self.fsm_graph

        # Collect all nodes relevant to this port
        relevant_nodes = []
        for node_index_ii in fsm_graph.nodes:
            node_ii = fsm_graph.nodes[node_index_ii]
            if "outputs" not in node_ii:
                continue # Skip this node if it doesn't have an "outputs" key

            if port_name_ii in [ output_port_definition.output_port_name for output_port_definition in node_ii["outputs"] ]:
                relevant_nodes.append(node_index_ii)

        # Iterate through each node in the relevant list and create the mapping
        update_map = {}
        for node_ii in relevant_nodes:
            # Get the output port definition
            output_port_definitions = fsm_graph.nodes[node_ii]["outputs"]
            
            # Find the output port for our target port
            output_value_ii = None
            for output_port_definition in output_port_definitions:
                if output_port_definition.output_port_name != port_name_ii:
                    continue

                # Otherwise, we have the right output port
                output_value_ii = output_port_definition.output_port_value

            # Create the mapping
            update_map[node_ii] = output_value_ii

        return update_map

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

    def initialize_last_output_value_map(self):
        """
        Description
        -----------
        This function takes the initial node and then uses
        all of the output definitions for that node to define
        the initial values for the `last_output_value` map.
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

        # Create the last output value map
        for output_port_definition in initial_node_data["outputs"]:
            # Extract values from the output port definition
            port_name_ii = output_port_definition.output_port_name
            output_value_ii = output_port_definition.output_port_value

            # Create the last output value map
            self.last_output_value[port_name_ii] = output_value_ii

        

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

        # Declare Discrete State for the FSM State
        self.current_state_index = self.DeclareDiscreteState(1)
        def discrete_state_init(context: Context, discrete_values: DiscreteValues):
            """
            Initialization function for the current state of the fsm
            """
            discrete_values.SetFrom(
                DiscreteValues(BasicVector(np.array([initial_node])))
            )
        self.DeclareInitializationDiscreteUpdateEvent(discrete_state_init)

        # Create the output port for the FSM state
        self.DeclareVectorOutputPort(
            "fsm_state",
            1,
            self.CalcFSMState,
        )

    def log(self, message: str, level: int = logging.INFO):
        """
        Description
        -----------
        This function logs a message to the logger.
        """
        self.logger.log(level, message)

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