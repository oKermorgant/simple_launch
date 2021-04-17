from os.path import basename, exists, join, splitext, exists, sep

from imp import load_source

from ros2run.api import get_executable_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from contextlib import contextmanager
import sys

def regular_path_elem(path):
    return path is None or type(path) == str



class SimpleLauncher:
    def __init__(self, namespace = ''):
        '''
        Initializes entities in the given workspace
        '''
        self.entities = [[]]
        self.index = 0
        self.composed = False
        
        if namespace:
            self.entity(PushRosNamespace(namespace))
        
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
       
    @contextmanager
    def group(self, ns=None, if_arg=None, unless_arg=None):
        '''
        Group the next nodes / entities into
         - another namespace
         - a if / unless condition depending on some argument
        '''
        # save current entity index
        prev_index = self.index

        self.entities.append([])
        self.index = len(self.entities)-1
        if ns is not None:
            self.entities[-1].append(PushRosNamespace(ns))
            
        try:
            yield self
        finally:
                        
            new_entities = self.entities[self.index]
            # restore state
            self.index = prev_index
            
            condition = None
            # get condition
            if if_arg is not None:
                condition = IfCondition(self.arg(if_arg))
            elif unless_arg is not None:
                condition = UnlessCondition(self.arg(unless_arg))
            # add new entities as sub-group
            self.entity(GroupAction(new_entities, condition=condition))
            
    @contextmanager
    def container(self, name, namespace = '', existing = False, **container_args):
        '''
        Opens a Composition group to add nodes
        If existing is True, then loads nodes in the (supposely) existing container
        '''
        prev_index = self.index

        self.entities.append([])
        self.index = len(self.entities)-1
        self.composed = True
        try:
            yield self
        finally:
            
            # restore state 
            new_entities = self.entities[self.index]
            self.index = prev_index
            self.composed = False
                        
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
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=new_entities,
                **container_args))
            
    def entity(self, entity):
        '''
        Directly adds a user-created Entity (Node, ComposableNodes, etc.)
        '''
        self.entities[self.index].append(entity)
            
    def node(self, package, executable = None, plugin = None, **node_args):
        '''
        Add a node to the launch tree.
        
        * package -- name of the package
        * executable (classical node) -- name of the node within the package, if None then assumes the node has the name of the package
        * plugin (inside a composition group) -- name of the composed node plugin within the package
        * node_args -- any other args passed to the node constructor
        ''' 
        if executable is None and not self.composed:
            executable = package
        if plugin is None and self.composed:
            raise Exception('Indicate the plugin name when adding a composable node')
        
        if 'arguments' in node_args:
            args = SimpleLauncher.flatten([node_args['arguments']])
            for i,arg in enumerate(args):
                if type(arg)==str:
                    args[i] = [TextSubstitution(text=kw) for kw in arg.split() if kw]
            node_args['arguments'] = SimpleLauncher.flatten(args)
        if 'parameters' in node_args:
            if type(node_args['parameters']) == dict:
                node_args['parameters'] =  [node_args['parameters']]
            elif type(node_args['parameters']) == list and all(type(elem)==dict for elem in node_args['parameters']):
                node_args['parameters']=[dict(kv for d in node_args['parameters'] for kv in d.iteritems())]
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
            AnyLaunchDescriptionSource(launch_file), launch_arguments=launch_arguments))
        
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
        if type(description_file) == str:
            cmd = ['xacro ' + description_file]
        else:
            cmd = ['xacro '] + description_file
        if xacro_args is not None:            
            if type(xacro_args) == str:
                # user has passed raw args
                cmd.append(' ' + xacro_args)
            else:
                # args as a dict
                for key, val in xacro_args.items():
                    cmd += [' ', key]
                    if val is not None:
                        cmd += self.flatten([':=',val])
        return self.name_join("'",Command(SimpleLauncher.name_join(*cmd)),"'")
        
    def robot_state_publisher(self, package=None, description_file=None, description_dir=None, xacro_args=None, **node_args):
        '''
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.
        
        If the file ends with 'xacro', or any path element is defined from an Argument, or xacro_args are passed, runs xacro on this file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * description_file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * node_args -- any additional node arguments such as remappings 

        '''
        urdf_xml = self.robot_description(package, description_file, description_dir, xacro_args)
        
        if 'parameters' in node_args:
            node_args['parameters'].update({'robot_description': urdf_xml})
        else:
            node_args['parameters'] = {'robot_description': urdf_xml}
            
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
        
