from os.path import join, exists
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from contextlib import contextmanager
from .simple_substitution import SimpleSubstitution, flatten
from .group import Group
from . import console
from .gazebo import only_show_args, silent_exec, GazeboBridge, ros_gz_prefix, gz_launch_setup
from typing import Text, List, Iterable, Tuple, Union

NODE_REMAPS = LAUNCH_ARGS = 1
NODE_PARAMS = 2
XACRO_ARGS = 3


def stringify(arg):
    if isinstance(arg, (str, Substitution, list)):
        return arg
    return str(arg)


def adapt_type(params, target):
    # detect type of passed params and adapts to launch API
    # NODE_PARAMS expects a list with 1 or several dict
    # NODE_REMAPS and LAUNCH_ARGS expect a list of (key,value) tuples
    # XACRO_ARGS expect a single string of key:=value
    if type(params) == dict:
        if target == NODE_PARAMS:
            return [params]
        elif target in (NODE_REMAPS, LAUNCH_ARGS):
            # such arguments do not support raw numerical values
            return [(key, stringify(val)) for key, val in params.items()]
        else:
            # xacro arguments are key:=value / only str or Substitution
            out = SimpleSubstitution()
            for key, val in params.items():
                out += [' ', key]
                if val is not None:
                    out += [':=', stringify(val)]
            return out

    if type(params) in (list, tuple):

        if all(type(elem) == dict for elem in params):
            # to dict
            return adapt_type(dict(key_val for elem in params for key_val in elem.items()), target)

        if all(type(elem) in (tuple,list) and len(elem) == 2 for elem in params):
            # (key, val) pairs
            if target in (NODE_REMAPS, LAUNCH_ARGS):
                # no-op
                return params
            return adapt_type(dict(params), target)

    return params


class SimpleLauncher:

    def __init__(self, namespace = None, use_sim_time = None, scope_included_files = False):
        '''
        Initializes entities in the given workspace
        If use_sim_time is True or False, creates a `use_sim_time` launch argument with this value as the default and forwards it to all nodes
        If use_sim_time is 'auto', then SimpleLauncher will set it to True if the /clock topic is advertized (case of an already running simulation)
        scope_included_files will make this launch file include other ones with scoped arguments, so that modifying them does not override the ones of this launch file
        '''
        # group tree
        self.__groups = [Group(namespace)]
        self.__cur_group = self.__groups[0]
        self.sim_time = None
        self.gz_axes = ('x','y','z','yaw','pitch','roll')
        self.__context = None
        self.scope_included_files = scope_included_files

        if use_sim_time is None:
            return

        # deal with use_sim_time
        if isinstance(use_sim_time, bool):
            self.sim_time = self.declare_arg('use_sim_time', use_sim_time,
                             description = 'Force use_sim_time parameter of instanciated nodes')
        elif use_sim_time != 'auto':
            console.warn("`use_sim_time` should be None, a Boolean, or 'auto' to rely on the /clock topic, ignoring")
        elif only_show_args():
            console.warn("This launch file will forward use_sim_time to all nodes if /clock is advertized at the time of the launch")
            return
        else:
            # detect if any /clock topic is advertized
            self.sim_time = False
            clock_info = silent_exec('ros2 topic info /clock').splitlines()
            for line in clock_info:
                if line.startswith('Publisher count'):
                    self.sim_time = int(line.split()[-1]) > 0
                    break
            if self.sim_time:
                console.info("SimpleLauncher(use_sim_time='auto'): found a /clock topic, forwarding use_sim_time:=True to all nodes")
            else:
                console.info("SimpleLauncher(use_sim_time='auto'): no /clock topic found, forwarding use_sim_time:=False to all nodes")

        # good candidate to replace tedious mechanics of use_sim_time
        # but will not be forwared to e.g. events and other stuff
        #self.add_action(SetParameter('use_sim_time', self.sim_time))

    def declare_arg(self, name, default_value = None, **kwargs):
        '''
        Add an argument to the launch file
        '''
        if self.__has_context():
            console.error(f'declaring a launch argument "{name}" while inside an opaque function\nyou should declare the arguments before the function')

        def to_string_nested(elem):
            if isinstance(elem, str):
                return elem
            elif not isinstance(elem, Iterable):
                return str(elem)
            elem = list(elem)
            for i,item in enumerate(elem):
                elem[i] = to_string_nested(item)
            return elem

        default_value = to_string_nested(default_value)

        if 'choices' in kwargs:
            kwargs['choices'] = to_string_nested(kwargs['choices'])

        self.__groups[0].add_action(DeclareLaunchArgument(
            name,
            default_value=default_value,
            **kwargs))

        return self.arg(name)

    @staticmethod
    def ros_version():
        '''
        Returns the name of the currently sourced ros version (e.g. $ROS_VERSION)
        '''
        from os import environ
        return environ['ROS_DISTRO']

    def arg(self, name: str):
        '''
        Retrieve an argument, should be a string otherwise -s will crash
        '''
        return self.__try_perform(SimpleSubstitution(LaunchConfiguration(self.__try_perform(name))))

    def arg_map(self, *names):
        '''
        Retrieves several arguments as a dict
        '''
        if len(names) == 1 and isinstance(names[0], (list, tuple)):
            # deprecated syntax, names was a tuple
            names = names[0]
        return dict((name, self.arg(name)) for name in names)

    def launch_description(self, opaque_function = None):
        '''
        Returns the setup launch description
        This value can be directly returned from generate_launch_description()
        It can also be stored in order to add custom entities

        If opaque_function is not None, then returns the generate_launch_description() wrapper around the passed function
        '''

        if opaque_function is None:
            # classical call without opaque function
            main_actions = self.__cur_group.close()
            return main_actions if self.__has_context() else LaunchDescription(main_actions)

        if self.__has_context():
            # opaque function but we already have context
            console.error('Calling SimpleLauncher.launch_description with opaque function within an opaque function makes it really opaque')

        from launch.actions import OpaqueFunction

        # the actual generate_launch_description function, just wrapping the passed opaque function
        def generate_launch_description():
            def wrapped_opaque_function(context):
                self.__context = context
                return opaque_function()

            return LaunchDescription(self.__cur_group.close() + [OpaqueFunction(function = wrapped_opaque_function)])

        return generate_launch_description

    def __has_context(self) -> bool:
        return self.__context is not None

    def __try_perform(self, substitution):
        '''
        Returns the performed value if the context is defined, otherwise the substitution
        '''
        if isinstance(substitution, Text) or not self.__has_context():
            return substitution

        from ast import literal_eval
        performed = substitution.perform(self.__context)
        try:
            return literal_eval(performed)
        except ValueError:
            if performed.lower() in ('true', 'false'):
                return literal_eval(performed.title())
        except SyntaxError:
            pass
        return performed

    def py_eval(self, *elems):
        '''
        Evaluates the Python expression
        '''
        # add a small check on lower case when combining conditions
        # TODO make it robust to forgotten spaces in and/or/not without relying on str.replace
        expr = SimpleSubstitution('eval("',elems,'", {"true": True, "false": False})')
        return self.__try_perform(SimpleSubstitution(PythonExpression(expr)))

    def find(self, package, file_name=None, file_dir = None):
        '''
        Retrieve the path to a file within its share directory.

        * package -- name of the package, if None then assumes an absolute file path
        * file_name -- name of the file to find (if None, returns the path to this package)
        * file_dir -- package directory containing the file (if None, will search the file)

        If any argument is neither string nore None, assumes use of parameters and returns the corresponding Substitution
        '''

        # resolve package
        package_dir = get_package_share_directory(package) if package else None

        if file_name is None:
            return package_dir

        # deal with non-resolvable package - cannot find anything in there
        def is_substitution(elem):
            return elem is not None and not isinstance(elem, str)
        
        if is_substitution(package) or is_substitution(file_name) or is_substitution(file_dir):
            return self.path_join(package_dir, file_dir, file_name)

        # below this point all arguments are strings or None
        if package_dir is None:
            return file_name

        # do not look for it, it's (said to be) there
        if file_dir is not None:
            return join(package_dir, file_dir, file_name)

        # look for it
        from os import walk
        for root, dirs, files in walk(package_dir, topdown=False):
            if file_name in files:
                return join(package_dir, root, file_name)

        # not there
        console.error(f'Could not find file {file_name} in package {package}')

    @contextmanager
    def group(self, ns=None, if_arg=None, unless_arg=None,
              if_condition=None, unless_condition=None,
              when = None):
        '''
        Group the next nodes / entities into
         - another namespace
         - a if / unless condition depending on some argument
         - a raw if / unless condition that may come out of an expression
         - an event condition (from simple_launch import OnExit, OnStart
        '''
        if self.__cur_group.is_container():
            console.error(f'Container "{self.__cur_group.is_container()}": Group blocks cannot be nested inside container blocks. Use OpaqueFunction or reverse the logic.')

        # we can only deal with 1 condition
        conditions = (if_arg, unless_arg, if_condition, unless_condition)
        if len(conditions) - conditions.count(None) > 1:
            console.error(f'group blocks cannot have more than 1 condition (has {len(conditions) - conditions.count(None)}')

        # get condition and cast it
        condition = None
        if if_arg is not None:
            condition = IfCondition(LaunchConfiguration(if_arg))
        elif unless_arg is not None:
            condition = UnlessCondition(LaunchConfiguration(unless_arg))
        elif if_condition is not None:
            condition = IfCondition(SimpleSubstitution(if_condition))
        elif unless_condition is not None:
            condition = UnlessCondition(SimpleSubstitution(unless_condition))

        self.__groups.append(Group(ns = ns, condition = condition, parent = self.__cur_group, when = when))
        self.__cur_group = self.__groups[-1]

        try:
            yield self
        finally:
            self.__cur_group = self.__cur_group.close()

    @contextmanager
    def container(self, name, namespace = '', existing = False, package='rclcpp_components', executable='component_container', **container_args):
        '''
        Opens a Composition group to add nodes
        If existing is True, then loads nodes in the (supposely) existing container
        '''

        self.__groups.append(Group(parent = self.__cur_group, container = name))
        self.__cur_group = self.__groups[-1]

        try:
            yield self
        finally:
            self.__cur_group, composed = self.__cur_group.close()

            # store ComposableNodes inside a Container
            if existing:
                self.add_action(
                    LoadComposableNodes(
                        composable_node_descriptions = composed,
                        target_container=name))
            else:
                self.add_action(
                    ComposableNodeContainer(
                        name=name,
                        namespace=namespace,
                        package=package,
                        executable=executable,
                        composable_node_descriptions = composed,
                        **container_args))

    def add_action(self, action):
        '''
        Adds an Action (Node, ComposableNodes, Group, etc.) at the current group level
        '''
        self.__cur_group.add_action(action)
        return action

    def node(self, package = None, executable = None, plugin = None, **node_args):
        '''
        Add a node to the launch tree. If use_sim_time was used then the use_sim_time parameter will be set if not explicitely given

        * package -- name of the package
        * executable (classical node) -- name of the node within the package, if None then assumes the node has the name of the package
        * plugin (inside a composition group) -- name of the composed node plugin within the package
        * node_args -- any other args passed to the node constructor
        '''
        as_composable = self.__cur_group.is_container()

        if plugin is None and as_composable:
            console.error('Indicate the plugin name when adding a composable node to a container')

        if executable is None and not as_composable:
            executable = package

        for key,target in (('parameters',NODE_PARAMS),('remappings',NODE_REMAPS)):
            if key in node_args:
                node_args[key] = adapt_type(node_args[key], target)

        if 'arguments' in node_args and isinstance(node_args['arguments'], str):
            node_args['arguments'] = node_args['arguments'].split()

        if self.sim_time is not None:
            if 'parameters' in node_args:

                if type(node_args['parameters'][0]) == dict:
                    if 'use_sim_time' not in node_args['parameters'][0]:
                        node_args['parameters'][0]['use_sim_time'] = self.sim_time
                elif type(node_args['parameters'][0]) == str:
                    # yaml-file, check if it contains use_sim_time
                    config_file = node_args['parameters'][0]
                    if exists(config_file):
                        with open(config_file) as f:
                            config = f.read().splitlines()
                        if not any(line.strip().startswith('use_sim_time') for line in config):
                            node_args['parameters'].append({'use_sim_time': self.sim_time})
                else:
                    console.neutral(f'node {package}/{executable} has an unkown config file, setting use_sim_time may fail')
                    node_args['parameters'] += [{'use_sim_time': self.sim_time}]
            else:
                node_args['parameters'] = [{'use_sim_time': self.sim_time}]

        if as_composable:
            # check plugin name - add package if needed
            if '::' not in plugin:
                plugin = '{}::{}'.format(package, plugin)
            return self.add_action(ComposableNode(package=package, plugin=plugin, **node_args))

        # basic node
        return self.add_action(Node(package=package, executable=executable, **node_args))

    def log_info(self, msg):
        '''
        Adds a LogInfo
        '''
        return self.add_action(LogInfo(msg = msg))

    def include(self, package=None, launch_file=None, launch_dir=None, launch_arguments=None):
        '''
        Include another launch file
        '''
        launch_file = self.find(package, launch_file, launch_dir)

        inclusion = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments=adapt_type(launch_arguments, LAUNCH_ARGS))

        if self.scope_included_files:
            # run included launch in a group to avoid polluting my scope
            from launch.actions import GroupAction
            return self.add_action(GroupAction([inclusion]))
        # just use default include behavior
        return self.add_action(inclusion)

    def call_service(self, server, request = None, verbosity = '', **kwargs):
        '''
        Calls the service at server address after checking its type.
        Request is a dictionary that is forwarded to service request fields, assuming they match
        verbosity is none or 'req', 'res' or 'reqres' to get information on service call
        '''
        params = {'simple_launch.server': server, 'simple_launch.verbosity': verbosity}
        if request is not None:
            params.update(request)
        return self.node('simple_launch', 'call_service', parameters = params, **kwargs)

    def set_parameters(self, node_name, parameters: dict = {}, verbosity = '', **kwargs):
        '''
        Sets the requested parameters for this node
        verbosity is none or 'req', 'res' or 'reqres' to get information on service call
        '''
        params = {'simple_launch.node': node_name, 'simple_launch.keys': list(parameters.keys()), 'simple_launch.verbosity': verbosity}
        params.update(parameters)
        return self.node('simple_launch', 'set_parameters',
                  parameters = params,
                  **kwargs)

    def rviz(self, config_file = None, warnings = False):
        '''
        Runs RViz with the given config file and warning level
        '''
        args = [] if config_file is None else ['-d', config_file]
        if not warnings:
            args += ['--ros-args', '--log-level', 'FATAL']
        self.node('rviz2', 'rviz2', arguments = args)

    def robot_description(self, package=None, description_file=None, description_dir=None, xacro_args=None):
        '''
        Returns the robot description after potential xacro parse if the file ends with xacro or xacro_args are defined
        '''
        description_file = self.find(package, description_file, description_dir)

        # just read description file if the path is string and file is URDF
        if type(description_file) == str and description_file.endswith('urdf') and xacro_args is None:
            with open(description_file) as f:
                urdf_xml = f.read()
            return urdf_xml

        # go for xacro output, compatible with launch parameters
        cmd = SimpleSubstitution('xacro ', description_file)
        if xacro_args is not None:
            cmd += adapt_type(xacro_args, XACRO_ARGS)
        return SimpleSubstitution("'", Command(cmd,on_stderr='warn'), "'")

    def robot_state_publisher(self, package=None, description_file=None, description_dir=None,
                              xacro_args=None, **node_args):
        '''
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * description_file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * node_args -- any additional node arguments such as remappings or parameters
        '''

        urdf_xml = self.robot_description(package, description_file, description_dir, xacro_args)

        if 'parameters' in node_args:
            # already some parameters, change to list of dictionaries
            node_args['parameters'] = adapt_type(node_args['parameters'], NODE_PARAMS)
            + [{'robot_description': urdf_xml}]
        else:
            node_args['parameters'] = [{'robot_description': urdf_xml}]

        # Launch the robot state publisher with the desired URDF
        self.node("robot_state_publisher", **node_args)

    def joint_state_publisher(self, use_gui = True, **node_args):
        '''
        Adds a joint_state_publisher / joint_state_publisher_gui with passed arguments as parameters
        Assumes some robot_description topic is published inside the namespace
        '''
        if type(use_gui) == bool:
            use_gui = str(use_gui)

        self.node('joint_state_publisher', parameters = node_args, condition=UnlessCondition(use_gui))
        self.node('joint_state_publisher_gui', parameters = node_args, condition=IfCondition(use_gui))

# Gazebo / Ignition methods

    def declare_gazebo_axes(self, **axes):
        '''
        Declares classical Gazebo axes as launch arguments
        If axes is void then declares all 6 axes with default value 0
        Otherwise declares the given axes with the given defaults
        '''
        if len(axes):
            self.gz_axes = [axis for axis,_ in axes.items() if axis in self.gz_axes]
            for axis in self.gz_axes:
                self.declare_arg(axis, default_value=axes[axis])
            return
        # no axis specified,
        self.gz_axes = ('x','y','z','yaw','pitch','roll')
        for axis in self.gz_axes:
            self.declare_arg(axis, default_value=0.)

    def gazebo_axes_args(self):
        '''
        Generate arguments corresponding to Gazebo spawner
        '''
        axes = {'x': 'x', 'y': 'y', 'z': 'z', 'roll': 'R', 'pitch': 'P', 'yaw': 'Y'}
        args = flatten([['-'+tag, self.arg(axis)] for axis,tag in axes.items() if axis in self.gz_axes])
        return [stringify(arg) for arg in args]

    def create_gz_bridge(self, bridges: Union[GazeboBridge,List[Union[GazeboBridge,Tuple]]], name = 'gz_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge with the passed GazeboBridge instances
        The bridge has a default name if not specified
        If any bridge is used for sensor_msgs/Image, ros_{gz,ign}_image will be used instead
        '''
        # adapt types
        if isinstance(bridges, GazeboBridge):
            bridges = [bridges]
        if len(bridges) == 0:
            return

        # lazy list of bridges
        for idx,bridge in enumerate(bridges):
            if not isinstance(bridge, GazeboBridge):
                bridges[idx] = GazeboBridge(*bridge)

        ros_gz = 'ros_' + ros_gz_prefix()

        # add camera_info for image bridges
        im_bridges = [bridge for bridge in bridges if bridge.is_image]

        for bridge in im_bridges:

            gz_head, gz_tail = bridge.gz_topic.split_tail()
            ros_head, ros_tail = bridge.ros_topic.split_tail()

            if not all(isinstance(tail, Text) and 'image' in tail for tail in (ros_tail, gz_tail)):
                continue

            cam = []
            for tail in (gz_tail, ros_tail):
                idx = tail.rfind('image')
                cam.append(tail[:idx] + 'camera_info')

            bridges.append(GazeboBridge(gz_head + [cam[0]], ros_head + [cam[1]], 'sensor_msgs/CameraInfo', GazeboBridge.gz2ros))

        std_config = sum([bridge.yaml() for bridge in bridges if not bridge.is_image], [])

        if std_config.has_elems():
            # use YAML-based configuration, handles Gazebo topics that are invalid to ROS
            from tempfile import NamedTemporaryFile
            dst = NamedTemporaryFile().name

            self.add_action(ExecuteProcess(cmd=['echo ', SimpleSubstitution('"',std_config,'"'), ' >> ', dst],
                                    shell=True, name=SimpleSubstitution(name, '_config')))
            self.node(f'{ros_gz}_bridge', 'parameter_bridge', name=name,
                        parameters = {'config_file': dst})

        if len(im_bridges):
            # use remapping to ROS topics
            remappings = []
            for bridge in im_bridges:
                for ext in ('', '/compressed', '/compressedDepth', '/theora'):
                    remappings.append((SimpleSubstitution(bridge.gz_topic,ext), SimpleSubstitution(bridge.ros_topic,ext)))

            self.node(f'{ros_gz}_image', 'image_bridge', name=SimpleSubstitution(name, '_image'),
                      arguments = [bridge.gz_topic for bridge in im_bridges],
                      remappings = remappings)

    def gz_world_tf(self, world_frame = None):
        '''
        Runs a static_transform_publisher to connect `world` and Gazebo world name, if different from `world`
        '''
        if world_frame is None:
            world_frame = GazeboBridge.world()
        if world_frame != 'world':
            self.node('tf2_ros', 'static_transform_publisher',
                      name = 'gz_world_tf',
                      arguments = ['--frame-id', 'world','--child-frame-id', world_frame])

    def create_gz_clock_bridge(self, name = 'gz_clock_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge for the /clock topic
        Typically used in the launch file that runs the simulation before spawning things in
        '''
        self.create_gz_bridge(GazeboBridge.clock(), name)

    def gz_launch(self, world_file, gz_args = None, full_world = None, save_after = 5.):
        '''
        Wraps gz_sim_launch to be Ignition/GzSim agnostic
        default version is Fortress (6), will use GZ_VERSION if present
        if `full_world` is given as a raw `string` then the resulting SDF will be saved, including base world and spawned entities
        '''
        if isinstance(full_world, str):
            if exists(full_world):
                launch_file, launch_arguments = gz_launch_setup(full_world, gz_args)
            else:
                launch_file, launch_arguments = gz_launch_setup(world_file, gz_args)
                self.save_gz_world(full_world, save_after)
        else:
            launch_file, launch_arguments = gz_launch_setup(world_file, gz_args)
        return self.include(launch_file = launch_file, launch_arguments = launch_arguments)

    def save_gz_world(self, dst, after = 5.):
        '''
        Saves the current world under dst
        Resolves any spawned URDF through their description parameter and converts to SDF
        '''
        from .events import When
        with self.group(when = When(delay = after)):
            self.node('simple_launch', 'generate_gz_world', arguments = [dst])

    def spawn_gz_model(self, name, topic = 'robot_description', model_file = None, spawn_args = []):
        '''
        Spawns a model into Gazebo under the given name, from the given topic or file
        Additional spawn_args can be given to specify e.g. the initial pose
        '''

        if model_file is not None:
            spawn_args = flatten(spawn_args + ['-file',model_file,'-name', name])
        else:
            spawn_args = flatten(spawn_args + ['-topic',topic,'-name', name])

        pkg = 'ros_ign_gazebo' if ros_gz_prefix() == 'ign' else 'ros_gz_sim'
        self.node(package = pkg, executable = 'create', arguments=spawn_args)

    # deprecated section

    def name_join(self, *elems):
        return self.__try_perform(SimpleSubstitution(elems))

    def path_join(self, *pathes):
        ret = SimpleSubstitution()
        for elem in pathes:
            ret /= elem
        return self.__try_perform(ret)

    # backward compatibility for new syntax
    service = call_service
    entity = add_action
