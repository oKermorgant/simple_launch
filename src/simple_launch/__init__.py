from os.path import basename, exists, join, splitext, exists, sep

from imp import load_source

from ros2run.api import get_executable_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from contextlib import contextmanager
import sys

def regular_path_elem(path):
    return path is None or type(path) == str

NODE_REMAPS = LAUNCH_ARGS = 1
NODE_PARAMS = 2
XACRO_ARGS = 3

def adapt_type(params, target):
    # detect type of passed params and adapts to launch API
    # NODE_PARAMS expects a list with 1 or several dict
    # NODE_REMAPS and LAUNCH_ARGS expect a list of (key,value) tuples
    # XACRO_ARGS 
    
    if type(params) == dict:        
        if target == NODE_PARAMS:
            return [params]
        elif target in (NODE_REMAPS, LAUNCH_ARGS):        
            # launch arguments do not support raw Booleans
            return [(key, str(val) if isinstance(val, bool) else val) for key, val in params.items()]
        else:
            # xacro arguments are key:=value / only str or Substitution
            if type(params) == str:
                # user has passed raw args
                return [' ', params]
            else:
                # args as a dict
                out = []
                for key, val in params.items():
                    out += [' ', key]
                    if val is not None:
                        out += [':=', val]
                return [arg if isinstance(arg, str) or isinstance(arg, Substitution) else str(arg)
                       for arg in SimpleLauncher.flatten(out)]
        
    if type(params) in (list, tuple):
                
        if all(type(elem)==dict for elem in params):
            # to dict
            return adapt_type(dict(key_val for elem in params for key_val in elem.items()), target)
        
        if all(type(elem) in (tuple,list) and len(elem) == 2 for elem in params):
            # (key, val) pairs
            return adapt_type(dict(params), target)    
    
    return params

def only_show_args():
    '''
    Returns True if the launch file was launch only to show its arguments
    '''
    return any(show_arg in sys.argv for show_arg in ('-s', '--show-args', '--show-arguments'))

def silent_exec(cmd):
    '''
    Executes the given command and returns the output
    Returns an empty list if any error
    '''
    from subprocess import check_output, STDOUT
    if isinstance(cmd, str):
        from shlex import split
        cmd = split(cmd)
    try:
        return check_output(cmd, stderr=STDOUT).decode().splitlines()
    except:
        return []    

class IgnitionBridge:
    ign2ros = '['
    ros2ign = ']'
    bidirectional = '@'
    models = None
    
    @staticmethod
    def read_models():
        if IgnitionBridge.models is not None:
            return
        if only_show_args():
            # set a dummy world model, we are not runnign anyway
            IgnitionBridge.models = ['','world [default]']
            print('\033[93mThis launch file will request information on a running Ignition instance at the time of the launch\033[0m')
            return
            
        models = silent_exec('ign model --list')
        if any(line.startswith('Requesting') for line in models):
            IgnitionBridge.models = [line.strip('- ') for line in models]
            print('\033[92mIgnitionBridge: connected to a running Ignition instance\033[0m')
        else:
            print('\033[91mIgnitionBridge: could not find any Ignition instance, launch will probably fail\033[0m')
        
    def __init__(self, ign_topic, ros_topic, msg, direction):
        '''
        Create a bridge instance to be passed to SimpleLauncher.create_ign_bridge        
        '''
        
        # find corresponding ign message 
        # from https://github.com/ignitionrobotics/ros_ign/blob/foxy/ros_ign_bridge/README.md
        msg_map = {'std_msgs/msg/Bool': 'ignition.msgs.Boolean',
 'std_msgs/msg/Empty': 'ignition.msgs.Empty',
 'std_msgs/msg/Float32': 'ignition.msgs.Float',
 'std_msgs/msg/Float64': 'ignition.msgs.Double',
 'std_msgs/msg/Header': 'ignition.msgs.Header',
 'std_msgs/msg/Int32': 'ignition.msgs.Int32',
 'std_msgs/msg/String': 'ignition.msgs.StringMsg',
 'geometry_msgs/msg/Quaternion': 'ignition.msgs.Quaternion',
 'geometry_msgs/msg/Vector3': 'ignition.msgs.Vector3d',
 'geometry_msgs/msg/Point': 'ignition.msgs.Vector3d',
 'geometry_msgs/msg/Pose': 'ignition.msgs.Pose',
 'geometry_msgs/msg/PoseStamped': 'ignition.msgs.Pose',
 'geometry_msgs/msg/Transform': 'ignition.msgs.Pose',
 'geometry_msgs/msg/TransformStamped': 'ignition.msgs.Pose',
 'geometry_msgs/msg/Twist': 'ignition.msgs.Twist',
 'nav_msgs/msg/Odometry': 'ignition.msgs.Odometry',
 'rosgraph_msgs/msg/Clock': 'ignition.msgs.Clock',
 'sensor_msgs/msg/BatteryState': 'ignition.msgs.BatteryState',
 'sensor_msgs/msg/CameraInfo': 'ignition.msgs.CameraInfo',
 'sensor_msgs/msg/FluidPressure': 'ignition.msgs.FluidPressure',
 'sensor_msgs/msg/Imu': 'ignition.msgs.IMU',
 'sensor_msgs/msg/Image': 'ignition.msgs.Image',
 'sensor_msgs/msg/JointState': 'ignition.msgs.Model',
 'sensor_msgs/msg/LaserScan': 'ignition.msgs.LaserScan',
 'sensor_msgs/msg/MagneticField': 'ignition.msgs.Magnetometer',
 'sensor_msgs/msg/PointCloud2': 'ignition.msgs.PointCloudPacked',
 'tf2_msgs/msg/TFMessage': 'ignition.msgs.Pose_V',
 'trajectory_msgs/msg/JointTrajectory': 'ignition.msgs.JointTrajectory'}
        
        if '/msg/' not in msg:
            msg = msg.replace('/', '/msg/')
            
        if msg not in msg_map:
            print(f'Cannot build a ros <-> ign bridge for message "{msg}": unknown type')
            return
        
        if not IgnitionBridge.valid(direction):
            print(f'Cannot build ros <-> ign bridge with direction "{direction}": should be in {{[,],@}}')
            return
                        
        # ros2 run ros_ign_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
        # auto-detect relative or absolute topic
        ign_abs_topic = ign_topic
                        
        if not( isinstance(ign_topic, str) and ign_topic.startswith('/') ):
            ign_abs_topic = SimpleLauncher.name_join(IgnitionBridge.model_prefix(ign_topic))
        
        self.tag = SimpleLauncher.name_join(ign_topic,'@',msg,direction,msg_map[msg])
        self.remapping = SimpleLauncher.name_join(ign_topic,':=', ros_topic)
    
    @staticmethod
    def valid(direction):
        return direction in (IgnitionBridge.ign2ros, IgnitionBridge.ros2ign, IgnitionBridge.bidirectional)    
    
    @staticmethod
    def world():
        IgnitionBridge.read_models()
        return IgnitionBridge.models[1].replace(']', '[').split('[')[1]
    
    @staticmethod
    def model_prefix(model):
        return SimpleLauncher.name_join(f"/world/{IgnitionBridge.world()}/model/",
                            model)
        
    @staticmethod
    def clock():
        return IgnitionBridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', IgnitionBridge.ign2ros)
    
    @staticmethod
    def has_model(model):
        IgnitionBridge.read_models()
        return SimpleLauncher.py_eval("'", model, "' in ", str(IgnitionBridge.models))

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
        
        if namespace:
            self.entity(PushRosNamespace(namespace))
        
        if use_sim_time is None:
            return
        
        # deal with use_sim_time
        if isinstance(use_sim_time, bool):
            self.declare_arg('use_sim_time', use_sim_time,
                             'Force use_sim_time parameter of instanciated nodes')
            self.auto_sim_time(self.arg('use_sim_time'))
        elif use_sim_time != 'auto':
            raise("\033[93mSimpleLauncher: `use_sim_time` should be None, a Boolean, or 'auto' to rely on the /clock topic\033[0m")
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
        
    def declare_arg(self, name, default_value = None, description = None):
        '''
        Add an argument to the launch file
        '''
        self.entity(DeclareLaunchArgument(
            name,
            default_value=str(default_value),
            description=description))
        
    def declare_gazebo_axes(self):
        '''
        Declares classical Gazebo axes as launch arguments
        '''        
        for axis in ('x','y','z','roll','pitch','yaw'):
            self.declare_arg(axis, default_value=0.)

    def arg(self, name):
        '''
        Retrieve an argument
        '''
        if type(name) != str:
            return name
        return LaunchConfiguration(name)
            
    def arg_map(self, names, to_update={}):
        '''
        Retrieves several arguments as a dict
        Updates passed dictionary if any
        '''    
        return {**to_update, **dict((name, self.arg(name)) for name in names)}
    
    def launch_description(self):
        '''
        Returns the setup launch description
        This value can be directly returned from generate_launch_description()
        It can also be stored in order to add custom entities
        '''
        return LaunchDescription(self.entities[0])
    
    @staticmethod
    def flatten(nested):
        '''
        Take a list with possibly sub-(sub-(...))-lists elements and make it to a 1-dim list
        '''
        return sum([SimpleLauncher.flatten(elem) if type(elem)==list else [elem] for elem in nested],[])
    
    @staticmethod
    def py_eval(*elems):
        '''
        Evaluates the Python expression
        '''
        return PythonExpression(elems)
    
    @staticmethod
    def py_eval_str(*elems):
        '''
        Evaluates the Python expression as a string
        '''
        return SimpleLauncher.name_join("'",PythonExpression(elems),"'")
    
    @staticmethod
    def name_join(*elems):
        return SimpleLauncher.flatten([TextSubstitution(text=elem) if type(elem)==str else elem for elem in elems if elem is not None])
    
    def gazebo_axes_args(self):
        '''
        Generate arguments corresponding to Gazebo spawner
        '''
        axes={'x': 'x', 'y': 'y', 'z': 'z', 'roll': 'R', 'pitch': 'P', 'yaw': 'Y'}
        return [['-'+tag, self.arg(axis)] for axis,tag in axes.items()]    
    
    @staticmethod
    def path_join(*pathes):
        ret = [TextSubstitution(text=sep)]*(2*len(pathes)-1)
        ret[::2] = pathes
        return SimpleLauncher.flatten(ret)
     
    @staticmethod
    def find(package, file_name, file_dir = None):
        '''
        Retrieve the path to a file within its share directory.

        * package -- name of the package, if None then assumes an absolute file path
        * file_name -- name of the file to find
        * file_dir -- package directory containing the file (if None, will search the file)
        
        If any argument is neither string nore None, assumes use of parameters and returns the corresponding Substitution
        '''
        
        # resolve package
        package_dir = package and get_package_share_directory(package) or None
        
        # deal with non-resolvable package - cannot find anything in there
        if not regular_path_elem(package) or not regular_path_elem(file_name) or not regular_path_elem(file_dir):
            return SimpleLauncher.path_join(package_dir, file_dir, file_name)
        
        # below this point all arguments are strings or None
        if package_dir == None:
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
        raise Exception('Could not find file {} in package {}'.format(file_name, package))
    
    def group_level_down(self):
        self.entities.append([])
        self.ns_graph[len(self.entities)-1] = self.index
        self.index = len(self.entities)-1
        
    def group_level_up(self):
        new_entities = self.entities[self.index]
        self.index = self.ns_graph[self.index]
        return new_entities
    
    '''
    # unused as of now
    def abs_node_ns(self, node_args = {}):
        
        ns = []
        if 'namespace' in node_args:
            ns.insert(0, node_args['namespace'])
            
        index = self.index    
        while index in self.ns_graph:
            if len(self.entities[index]):
                head = self.entities[index][0]
                if(type(head) == PushRosNamespace):
                    ns.insert(0, head.namespace)
            index = self.ns_graph[index]
        return self.name_join(["'"], self.path_join(*ns), ["'"])
    '''
    
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
            # get condition
            if if_arg is not None:
                condition = IfCondition(self.arg(if_arg))
            elif unless_arg is not None:
                condition = UnlessCondition(self.arg(unless_arg))
            elif if_condition is not None:
                condition = IfCondition(if_condition)
            elif unless_condition is not None:
                condition = UnlessCondition(unless_condition)
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
                if 'use_sim_time' not in node_args['parameters'][0]:
                    node_args['parameters'][0]['use_sim_time'] = self.sim_time
            else:
                node_args['parameters'] = [{'use_sim_time':  self.sim_time}]
                
        if not self.composed:
            self.entity(Node(package=package, executable=executable, **node_args))
        else:
            # check plugin name - add package if needed
            if '::' not in plugin:
                plugin = '{}::{}'.format(package, plugin)
            self.entity(ComposableNode(package=package, plugin=plugin, **node_args))
        
    def include(self, package, launch_file, launch_dir=None, launch_arguments=None):
        '''
        Include another launch file
        '''
        launch_file = self.find(package, launch_file, launch_dir)
        self.entity(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments=adapt_type(launch_arguments, LAUNCH_ARGS)))
        
    def robot_description(self, package=None, description_file=None, description_dir=None, xacro_args=None):
        '''
        Returns the robot description after potential xacro parse if the file ends with xacro or xacro_args are defined
        '''
        description_file = self.find(package, description_file, description_dir)
        
        # just read description file if the path is string and file is URDF
        if type(description_file) == str and description_file.endswith('urdf') and xacro_args is None:
            with open(description_file) as f:
                urdf_xml = f.read()
            return self.name_join("'", urdf_xml, "'")
        
        # go for xacro output, compatible with launch parameters
        if type(description_file) == str:
            cmd = ['xacro ' + description_file]
        else:
            cmd = ['xacro '] + description_file
        if xacro_args is not None:  
            cmd += adapt_type(xacro_args, XACRO_ARGS)
        return self.name_join("'",Command(SimpleLauncher.name_join(*cmd)),"'")
        
    def robot_state_publisher(self, package=None, description_file=None, description_dir=None, xacro_args=None, prefix_gz_plugins=False, namespaced_tf = False, **node_args):
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
        frame_prefix = self.name_join("'", frame_prefix, "'")
                
        if prefix_gz_plugins:
            urdf_xml = Command(SimpleLauncher.name_join(['ros2 run simple_launch frame_prefix_gazebo',
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
        
    def create_ign_bridge(self, bridges, name = None):
        '''
        Create a ros_ign_bridge::parameter_bridge with the passed IgnitionBridge instances
        The bridge has a default name if not specified        
        '''
        if type(bridges) not in (list, tuple):
            bridges = [bridges]
        bridge_args = []
        remappings = []
        for bridge in bridges:
            if bridge is not None:
                bridge_args.append(bridge.tag)
                remappings += ['-r',bridge.remapping]
                                
        self.node('ros_ign_bridge','parameter_bridge', name=name, 
                  arguments=bridge_args + ['--ros-args'] + remappings,
                  parameters = {'args': bridge_args})

    def spawn_ign_model(self, name, topic = 'robot_description', spawn_args = [], only_new = True):
        '''
        Spawns a model into Ignition under the given name, from the given topic
        Additional spawn_args can be given to specify e.g. the initial pose
        '''
        spawn_args += ['-topic',topic,'-name', name]
        
        # spawn if not already there
        if only_new:
            with self.group(unless_condition = IgnitionBridge.has_model(name)):
                self.node('ros_ign_gazebo','create', arguments=spawn_args)
        else:
            self.node('ros_ign_gazebo','create', arguments=spawn_args)
