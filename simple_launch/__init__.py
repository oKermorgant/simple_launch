from os.path import join, exists
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from contextlib import contextmanager
from .simple_substitution import SimpleSubstitution, flatten
from .gazebo import only_show_args, silent_exec, GazeboBridge
from typing import Text

NODE_REMAPS = LAUNCH_ARGS = 1
NODE_PARAMS = 2
XACRO_ARGS = 3


def adapt_type(params, target):
    # detect type of passed params and adapts to launch API
    # NODE_PARAMS expects a list with 1 or several dict
    # NODE_REMAPS and LAUNCH_ARGS expect a list of (key,value) tuples
    # XACRO_ARGS expect a single string of key:=value

    def stringify(arg):
        if isinstance(arg, (str, Substitution, list)):
            return arg
        return str(arg)

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

    def __init__(self, namespace = '', use_sim_time = None):
        '''
        Initializes entities in the given workspace
        If use_sim_time is True or False, creates a `use_sim_time` launch argument with this value as the default and forwards it to all nodes
        If use_sim_time is 'auto', then SimpleLauncher will set it to True if the /clock topic is advertized (case of an already running simulation)
        '''
        self.entities = [[]]
        self.index = 0
        self.ns_graph = {0: -1}
        self.composed = False
        self.sim_time = None
        self.gz_axes = ('x','y','z','yaw','pitch','roll')
        self.__context = None

        if namespace:
            self.entity(PushRosNamespace(namespace))

        if use_sim_time is None:
            return

        # deal with use_sim_time
        if isinstance(use_sim_time, bool):
            self.declare_arg('use_sim_time', use_sim_time,
                             description = 'Force use_sim_time parameter of instanciated nodes')
            self.auto_sim_time(self.arg('use_sim_time'))
        elif use_sim_time != 'auto':
            raise(RuntimeWarning("\033[93mSimpleLauncher: `use_sim_time` should be None, a Boolean, or 'auto' to rely on the /clock topic\033[0m"))
        elif only_show_args():
            print('\033[93mThis launch file will forward use_sim_time to all nodes if /clock is advertized at the time of the launch\033[0m')
            return
        else:
            self.auto_sim_time()

    def auto_sim_time(self, force = None):
        '''
        Forces use_sim_time for all nodes.
        If force is None then checks if /clock is being published and sets use_sim_time if this is the case
        '''
        if force is None:
            self.sim_time = False
            clock_info = silent_exec('ros2 topic info /clock')
            for line in clock_info:
                if line.startswith('Publisher count'):
                    self.sim_time = int(line.split()[-1]) > 0
                    break
            if self.sim_time:
                print("\033[92mSimpleLauncher(use_sim_time='auto'): found a /clock topic, forwarding use_sim_time:=True to all nodes\033[0m")
            else:
                print("\033[92mSimpleLauncher(use_sim_time='auto'): no /clock topic found, forwarding use_sim_time:=False to all nodes\033[0m")
        else:
            self.sim_time = force

    def declare_arg(self, name, default_value = None, **kwargs):
        '''
        Add an argument to the launch file
        '''
        if self.has_context():
            raise Exception('simple_launch: declaring a launch argument while inside an opaque function')

        self.entity(DeclareLaunchArgument(
            name,
            default_value=str(default_value),
            **kwargs))

        return self.arg(name)

    @staticmethod
    def ros_version():
        '''
        Returns the name of the currently sourced ros version (e.g. $ROS_VERSION)
        '''
        from os import environ
        return environ['ROS_DISTRO']

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

    def arg(self, name):
        '''
        Retrieve an argument
        '''
        if type(name) != str:
            return self.try_perform(name)
        return self.try_perform(SimpleSubstitution(LaunchConfiguration(name)))

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
            return self.entities[0] if self.has_context() else LaunchDescription(self.entities[0])

        if self.has_context():
            raise Exception('Calling SimpleLauncher.launch_description with opaque function within an opaque function')

        from launch.actions import OpaqueFunction

        # the actual generate_launch_description function, just wrapping the passed opaque function
        def generate_launch_description():
            def wrapped_opaque_function(context):
                self.__context = context
                return opaque_function()

            return LaunchDescription(self.entities[0] + [OpaqueFunction(function = wrapped_opaque_function)])

        return generate_launch_description

    def has_context(self):
        return self.__context is not None

    def try_perform(self, substitution):
        '''
        Returns the performed value if the context is defined, otherwise the substitution
        '''
        if isinstance(substitution, Text) or not self.has_context():
            return substitution

        from ast import literal_eval
        performed = substitution.perform(self.__context)
        try:
            return literal_eval(substitution.perform(self.__context))
        except ValueError:
            if performed.lower() in ('true', 'false'):
                return literal_eval(performed.title())
            return performed

    def py_eval(self, *elems):
        '''
        Evaluates the Python expression
        '''
        return self.try_perform(SimpleSubstitution(PythonExpression(elems)))

    def name_join(self, *elems):
        return self.try_perform(SimpleSubstitution(elems))

    def gazebo_axes_args(self):
        '''
        Generate arguments corresponding to Gazebo spawner
        '''
        axes = {'x': 'x', 'y': 'y', 'z': 'z', 'roll': 'R', 'pitch': 'P', 'yaw': 'Y'}
        return [['-'+tag, self.arg(axis)] for axis,tag in axes.items() if axis in self.gz_axes]

    def path_join(self, *pathes):
        ret = SimpleSubstitution()
        for elem in pathes:
            ret /= elem
        return self.try_perform(ret)

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
            return elem is not None and type(elem) != str
        
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
        raise Exception(f'Could not find file {file_name} in package {package}')

    def group_level_down(self):
        self.entities.append([])
        self.ns_graph[len(self.entities)-1] = self.index
        self.index = len(self.entities)-1

    def group_level_up(self):
        new_entities = self.entities[self.index]
        self.index = self.ns_graph[self.index]
        return new_entities

    @contextmanager
    def group(self, ns=None, if_arg=None, unless_arg=None, if_condition=None, unless_condition=None):
        '''
        Group the next nodes / entities into
         - another namespace
         - a if / unless condition depending on some argument
         - a raw if / unless condition that may come out of an expression
        '''
        self.group_level_down()

        if ns is not None:
            self.entity(PushRosNamespace(ns))

        try:
            yield self
        finally:

            new_entities = self.group_level_up()

            condition = None

            # we can only deal with 1 condition
            conditions = (if_arg, unless_arg, if_condition, unless_condition)
            if len(conditions) - conditions.count(None) > 1:
                raise Exception(f'SimpleLauncher groups cannot have more than 1 condition (has {len(conditions) - conditions.count(None)}')

            # get condition
            if if_arg is not None:
                condition = IfCondition(self.arg(if_arg))
            elif unless_arg is not None:
                condition = UnlessCondition(self.arg(unless_arg))
            elif if_condition is not None:
                condition = IfCondition(SimpleSubstitution(if_condition))
            elif unless_condition is not None:
                condition = UnlessCondition(SimpleSubstitution(unless_condition))
            # add new entities as sub-group
            self.entity(GroupAction(new_entities, condition=condition))

    @contextmanager
    def container(self, name, namespace = '', existing = False, package='rclcpp_components', executable='component_container', **container_args):
        '''
        Opens a Composition group to add nodes
        If existing is True, then loads nodes in the (supposely) existing container
        '''
        self.group_level_down()

        self.composed = True
        try:
            yield self
        finally:

            self.composed = False
            new_entities = self.group_level_up()

            # store ComposableNodes inside a Container
            if existing:
                self.entity(
                    LoadComposableNodes(
                        composable_node_descriptions = new_entities,
                        target_container=name
                        ))
            else:
                self.entity(
                ComposableNodeContainer(
                name=name,
                namespace=namespace,
                package=package,
                executable=executable,
                composable_node_descriptions=new_entities,
                **container_args))

    def entity(self, entity):
        '''
        Directly adds a user-created Entity (Node, ComposableNodes, etc.)
        '''
        self.entities[self.index].append(entity)

    def node(self, package, executable = None, plugin = None, **node_args):
        '''
        Add a node to the launch tree. If auto_sim_time was used then the use_sim_time parameter will be set if not explicitely given

        * package -- name of the package
        * executable (classical node) -- name of the node within the package, if None then assumes the node has the name of the package
        * plugin (inside a composition group) -- name of the composed node plugin within the package
        * node_args -- any other args passed to the node constructor
        '''
        if executable is None and not self.composed:
            executable = package
        if plugin is None and self.composed:
            raise Exception('Indicate the plugin name when adding a composable node')

        for key,target in (('parameters',NODE_PARAMS),('remappings',NODE_REMAPS)):
            if key in node_args:
                node_args[key] = adapt_type(node_args[key], target)

        if self.sim_time is not None:
            if 'parameters' in node_args:
                if type(node_args['parameters'][0]) == dict and 'use_sim_time' not in node_args['parameters'][0]:
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
                    print('simple_launch: skipping use_sim_time for node', f'{package}/{executable}', 'cannot check if already here')
#                    #node_args['parameters'] += [{'use_sim_time': self.sim_time}]
            else:
                node_args['parameters'] = [{'use_sim_time': self.sim_time}]

        if not self.composed:
            self.entity(Node(package=package, executable=executable, **node_args))
        else:
            # check plugin name - add package if needed
            if '::' not in plugin:
                plugin = '{}::{}'.format(package, plugin)
            self.entity(ComposableNode(package=package, plugin=plugin, **node_args))

    def include(self, package=None, launch_file=None, launch_dir=None, launch_arguments=None):
        '''
        Include another launch file
        '''
        launch_file = self.find(package, launch_file, launch_dir)
        self.entity(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments=adapt_type(launch_arguments, LAUNCH_ARGS)))

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

    def robot_state_publisher(self, package=None, description_file=None, description_dir=None, xacro_args=None, prefix_gz_plugins=False,
                              namespaced_tf = False, **node_args):
        '''
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.

        If the file ends with 'xacro', or any path element is defined from an Argument, or xacro_args are passed, runs xacro on this file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * description_file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * prefix_gz_plugins -- will forward any frame_prefix to frame names published by Gazebo plugins
        * namespaced_tf -- equivalent to remapping /tf and /tf_static to local namespace
        * node_args -- any additional node arguments such as remappings
        '''

        urdf_xml = self.robot_description(package, description_file, description_dir, xacro_args)

        frame_prefix = ""
        if 'parameters' in node_args:
            node_args['parameters'] = adapt_type(node_args['parameters'], NODE_PARAMS)
            if 'frame_prefix' in node_args['parameters'][0]:
                frame_prefix = node_args['parameters'][0]['frame_prefix']
        else:
            node_args['parameters'] = []
        frame_prefix = SimpleSubstitution("'", frame_prefix, "'")

        if prefix_gz_plugins:
            urdf_xml = Command(SimpleSubstitution(['ros2 run simple_launch frame_prefix_gazebo',
                                                         ' -d ', urdf_xml,
                                                         ' --frame_prefix ', frame_prefix]))

        node_args['parameters'] += [{'robot_description': urdf_xml}]

        if namespaced_tf:
            remaps = {'/tf':'tf', '/tf_static':'tf_static'}
            if 'remappings' in node_args:
                if type(node_args['remappings']) == dict:
                    node_args['remappings'].update(remaps)
                else:
                    node_args['remappings'] += [remaps]
            else:
                node_args['remappings'] = remaps

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

    @staticmethod
    def gz_prefix():
        return 'ign' if SimpleLauncher.ros_version() < 'humble' else 'gz'

    def create_gz_bridge(self, bridges: list[GazeboBridge], name = 'gz_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge with the passed GazeboBridge instances
        The bridge has a default name if not specified
        If any bridge is used for sensor_msgs/Image, ros_{gz,ign}_image will be used instead
        '''
        if type(bridges) not in (list, tuple):
            bridges = [bridges]
        if len(bridges) == 0:
            return

        gz = self.gz_prefix()
        ros_gz = f'ros_{gz}'

        # add camera_info for image bridges
        im_bridges = [bridge for bridge in bridges if bridge.is_image]

        for bridge in im_bridges:
            gz_head, gz_tail = bridge.gz_topic.split_tail()
            ros_head, ros_tail = bridge.ros_topic.split_tail()

            if not all(isinstance(tail, Text) and '/image' in tail for tail in (ros_tail, gz_tail)):
                continue

            cam = []
            for tail in (gz_tail, ros_tail):
                idx = tail.rfind('/image')
                cam.append(tail[:idx] + '/camera_info')

            bridges.append(GazeboBridge(gz_head + [cam[0]], ros_head + [cam[1]], 'sensor_msgs/CameraInfo', GazeboBridge.gz2ros))

        std_config = sum([bridge.yaml(gz) for bridge in bridges if not bridge.is_image], [])

        if std_config.has_elems():
            # use YAML-based configuration, handles Gazebo topics that are invalid to ROS
            from tempfile import NamedTemporaryFile
            dst = NamedTemporaryFile().name

            self.entity(ExecuteProcess(cmd=['echo ', SimpleSubstitution('"',std_config,'"'), ' >> ', dst],
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

    def create_gz_clock_bridge(self, name = 'gz_clock_bridge'):
        '''
        Create a ros_gz_bridge::parameter_bridge for the /clock topic
        Typically used in the launch file that runs the simulation before spawning things in
        '''
        self.create_gz_bridge(GazeboBridge.clock(), name)

    def gz_launch(self, gz_args = None):

        if self.gz_prefix() == 'gz':
            launch_file = self.find('ros_gz_sim', 'gz_sim.launch.py')
            launch_args = 'gz_args'
        else:
            launch_file = self.find('ros_ign_gazebo', 'ign_gazebo.launch.py')
            launch_args = 'ign_args'
        if gz_args is None:
            self.include(launch_file = launch_file)
        else:
            self.include(launch_file = launch_file, launch_arguments = {launch_args: gz_args})

    def spawn_gz_model(self, name, topic = 'robot_description', model_file = None, spawn_args = [], only_new = True):
        '''
        Spawns a model into Gazebo under the given name, from the given topic or file
        Additional spawn_args can be given to specify e.g. the initial pose
        '''
        if model_file is not None:
            spawn_args = flatten(spawn_args + ['-file',model_file,'-name', name])
        else:
            spawn_args = flatten(spawn_args + ['-topic',topic,'-name', name])

        # spawn if not already there
        pkg = 'ros_ign_gazebo' if self.ros_version() < 'humble' else 'ros_gz_sim'
        node = Node(package = pkg, executable = 'create', arguments=spawn_args)

        if only_new:
            if self.has_context():
                if not GazeboBridge.has_model(name):
                    self.entity(node)
            else:
                with self.group(unless_condition = GazeboBridge.has_model(name)):
                    self.entity(node)
        else:
            self.entity(node)
