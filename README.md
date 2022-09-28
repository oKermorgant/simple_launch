# simple_launch
This package provides a Python class to help writing ROS 2 launch files.

The motivation behind this package is that the ROS 2 launch Python syntax may not be suited for many simple cases such as running basic nodes, spawning a `robot_state_publisher`, and grouping nodes in namespaces or components..

**Thanks**: this package is inspired by [ros2_launch_util](https://github.com/bponsler/ros2_launch_util).

The entry point is the `SimpleLauncher` class, which has several capabilities.

## Basic syntax

### Namespace and argument parser initialization

`sl = SimpleLauncher(namespace = '', use_sim_time = None)`

- will initialize all nodes relative to the given namespace
- if `use_sim_time` is a Boolean, creates a `use_sim_time` launch argument with this value as the default and forwards it to all nodes, unless explicitely specified when running the node
- if `use_sim_time` is `'auto'`, then `SimpleLauncher` will set it to `True` if the `/clock` topic is advertized (case of an already running simulation). **This may have side effects if the `/clock` topic is advertized but you want to use this launch file with system clock**.
- if `use_sim_time` is `None` (default) then no particular value is forwarded to the nodes

### Node registration

`sl.node(package, executable, **node_args)` where

- `package` is the node package
- `executable` is the name of the executable
- `node_args` are any additional `Node` arguments given as a list

### Launch file include

`sl.include(package, launch_file, launch_dir = None, launch_arguments=None)` where

- `package` is the package of the included launch file
- `launch_file` is the name of the launch file
- `launch_dir` is its directory inside the package share (`None` to have it found)
- `launch_arguments` is a list of arguments to pass to the included launch file

### Robust types for parameters

In the launch API, differents types are expected for:

- node parameters: a list of dictionaries
- node remappings: a list of (`key`, `value`) pairs
- included launch arguments: a list of (`key`, `value`) pairs
- xacro arguments: a concatenation of `key:=value` strings

The `sl.include`, `sl.node` and `xacro_args` calls allow using any type (the simplest being a single dictionary)  and will convert to the one expected by the API.

### `SimpleSubstution` class

Most all methods listed below return an instance of `SimpleSubstitution` that wraps any Substitution, but that provides concatenation (`+`) and path concatenation (`/`) operators. It is still a `Substitution`, not a raw Python type.

## Launch arguments

The helper class allows declaring launch arguments and getting them in return:

### Declare a launch argument

`sl.declare_arg(name, default_value, description = None)`: declare and returns the argument

### Retrieve a launch argument

`sl.arg(name)`: returns the argument `name` as a `SimpleSubstitution`

### Retrieve several arguments as a dictionary

`sl.arg_map('robot', 'x', 'y')`: returns `{'robot': <robot arg value>, 'x': <x arg value>, 'y': <y arg value>}`

## Node groups

Groups are created through the `with sl.group():` syntax and accepts both a namespace and/or an if/unless condition:

### By namespace

```
  with sl.group(ns=sub_namespace):
    sl.node(package, executable)
```

### From a condition

```
  with sl.group(if_condition=True):
    sl.node(package, executable)

  with sl.group(unless_condition=<some expression>):
    sl.node(package, executable)
```
Only one condition can be set in a group, nested condition mst be combined first, or used in nested groups.

### From conditional arguments

```
  with sl.group(if_arg='use_gui'):
    sl.node(package, executable)

  with sl.group(unless_arg='dont_do'):
    sl.node(package, executable)
```

`if_arg` / `unless_arg` is expected to be the name of a launch argument. These two lines are equivalent:

```
  with sl.group(if_arg='use_gui'):
  with sl.group(if_condition=sl.arg('use_gui')):
```
If `if_arg` / `unless_arg` is not a string then it is considered as a `if_condition` / `unless_condition`.

### Creating containers

This syntax adds the `composition/composition::Talker` as a ComposableNode

```
  with sl.container(name='my_container', output='screen'):
    sl.node(package='composition', plugin='Talker', name='talker')
```

Use the `executable` and `package` parameters if you want to use executors other than `rclcpp_components`'s `component_container`:

```
  with sl.container(name='my_container', output='screen', executable='component_container_isolated'):
```

## `use_sim_time`

The current `use_sim_time` setting can be retrieved through `sl.sim_time` that may be:

- `None`, if `use_sim_time` was not set in the `SimpleLauncher` constructor
- a raw Boolean, if `use_sim_time` was set to `'auto'`, depending on the `/clock` topic being advertized
- a Boolean launch argument, if `use_sim_time` was set to `True` or `False`

In all cases, if the `use_sim_time` parameter is explicitely given to a node, it will be used instead of the `SimpleLauncher` instance one.


## Wrapper around `OpaqueFunction`

Most of the use cases can be dealt with substitutions and `with sl.group` blocs.
In order to design more imperative launch files, the [`OpaqueFunction`](https://discourse.ros.org/t/simplifying-launch-argument-declaration-and-initialization-in-launch-files/24204) approach can be used. The main drawback is that potential errors are harder to track.

To do this with `simple_launch`:
  - the `SimpleLauncher` instance and the argument declaration should be done in the main body of your launch file.
  - then, define a function (e.g. `launch_setup`) that takes no argument, where the logic of the launch file resides.
  - all arguments obtained through `sl.arg` will be basic Python types, obtained from performing the substitutions.
  - finally just export `generate_launch_description = sl.launch_description(opaque_function = launch_setup)`.

Compare [`example_launch.py`](example/example_launch.py) and [`example_opaque_launch.py`](example/example_opaque_launch.py) to see the two approaches on the same logic.

Note that inside an `OpaqueFunction` the if/unless idom reduces to a basic if/else:

```
# with substitutions
with sl.group(if_arg='some_condition'):
  # do stuff
with sl.group(unless_arg='some_condition'):
  # do other stuff

# with opaque function
if sl.arg('some_condition'):
  # do stuff
else:
  # do other stuff
```


## Interaction with Gazebo / Ignition

*Note: Ignition being renamed to Gazebo, all tools in this section use Gazebo / gz names*

### Spawn a model

The `sl.spawn_gz_model(name, topic, model_file = None, spawn_args = [], only_new = True)` functions allows easily spawing a model from its `robot_description`:

- `name` is the name this model will get in Gazebo
- `topic` is the topic to obtain the model from, default is `robot_description` (relative to the current namespace)
- `model_file` is the path to the (urdf or sdf) file. If defined then this will spawn this model and ignore the topic
- `only_new` if True, will not spawn the model if it already exists in a running Gazebo instance
- `spawn_args` are any additional spawn arguments, e.g. the initial pose

**example:** `sl.spawn_gz_model('my_robot', model_file = sl.find('my_pkg', 'my_model.urdf'))`

### Declare initial pose

Calling `sl.declare_gazebo_axes()` will declare all 6 parameters `(x,y,z,roll,pitch,yaw)` with null default values.
If any axis is given (e.g. `sl.declare_gazebo_axes(yaw = 3.14)` then only this parameter will be declared.

Such parameters can be retrieved through `sl.gazebo_axes_args()`. As a consequence, it is easy to spawn a model with:
```
sl.declare_gazebo_axes()
sl.spawn_gz_model(name, spawn_args = sl.gazebo_axes_args())
```

### Gazebo sim

The `GazeboBridge` class has a few static methods to interact with a **running Gazebo**. Namely:

- `GazeboBridge.world()` returns the current world name
- `GazeboBridge.model_prefix(model)` builds the Gazebo topic relative to the given model `/world/<world>/model/<model>`
- `GazeboBridge.has_model(model)` returns `True` of `False` depending on the passed model existing in Gazebo already

### Gazebo bridge

The `GazeboBridge` class allows easily creating bridges when using Gazebo. Gazebo has to be already running in order to get information on the simulation scene.

An instance is created with: `bridge = GazeboBridge(<gazebo_topic>, <ros_topic>, <ros_message>, direction)` where `direction` is either:

- `GazeboBridge.gz2ros` for Gazebo -> ROS
- `GazeboBridge.ros2gz` for ROS -> Gazebo
- `GazeboBridge.bidirectional` for both

The Gazebo message type is deduced from the ros message type. Remapping will be set to the given `ros_topic`.

The SimpleLauncher instance can then run all created bridges with: `sl.create_gz_bridge([bridges], <node_name>)`, as illustrated in the examples at this end of this document.
If some bridges involve `sensor_msgs/Image` then a dedicated `ros_ign_image` bridge will be used.

## Other shortcuts

### String / substitution concatenation

The following syntax builds the `SimpleSubstitution` corresponding to `<robot name>.xacro`:

`file_name = sl.arg('robot') + '.xacro'`

*deprecated*: `sl.name_join(sl.arg('robot'), '.xacro')`

### Path concatenation

The following syntax builds the `SimpleSubstitution` corresponding to `<package_path>/urdf/<robot name>.xacro`:

```
file_name = sl.arg('robot') + '.xacro'
urdf_file = os.path.join(get_package_share_directory(package),'urdf')/file_name
```
Obviously if all the path elements are raw strings, you should use `os.path.join` all along.

*deprecated*: `sl.path_join(get_package_share_directory(package), sl.arg('robot'), '.xacro')`


### Find a share file

`path = sl.find(package, file_name, file_dir = None)` where:

- `package` is the name of the package or `None` if `file_name` is already an absolute path
- `file_name` is the name of the file to find
- `file_dir` is the path inside the package

If `file_dir` is `None` then the `find` function will actually look for the file inside the package share, assuming that `package` and `file_name` are raw strings.

### Robot state publisher

`sl.robot_state_publisher(package, description_file, description_dir=None, xacro_args=None, prefix_gz_plugins=False, **node_args)` where

- `description_file` is a URDF or xacro file
- `description_dir` is the sub-directory of the file. If omitted, let the script search for the file assuming it is a raw string
- `xacro_args` are passed to xacro
- `prefix_gz_plugins` is used only if a `frame_prefix` parameter is given to `robot_state_publisher`. In this case it will forward the frame prefix to Gazebo-published messages that include frame names
- `node_args` are any additional arguments for `robot_state_publisher` (typically remapping)

### Joint state publisher

`sl.joint_state_publisher(use_gui, **node_args)`: fires up a `joint_state_publisher`, with or without the gui.

### Fallback to low-level syntax

If any unavailable functionality is needed, the `sl.entity(entity)` function adds any passed `Entity` at the current namespace / conditional / composition level.


## Examples

Examples can be found in the corresponding folder.


### Launch with arguments

Here is a file spawning a `robot_state_publisher` and its `joint_state_publisher`, where:

- `prefix`, `x` and `y` are launch arguments that are passed to xacro
- `use_gui` tells whether `joint_state_publisher` should run the gui

```
from simple_launch import SimpleLauncher

def generate_launch_description():
    '''
    Launch description for a single robot - runs the two nodes in their own namespace
    '''
    sl = SimpleLauncher()

    sl.declare_arg('prefix', default_value = '', description='name of the robot (+ tf prefix)')
    sl.declare_arg('x', default_value = 0, description='x-offset of the robot')
    sl.declare_arg('y', default_value = 0, description='y-offset of the robot')
    sl.declare_arg('use_gui', default_value = True, description='Use JSP gui')

    xacro_args = sl.arg_map(('prefix', 'x', 'y'))
    xacro_args['prefix'] = [xacro_args['prefix'], ':']

    with sl.group(ns=sl.arg('prefix')):
        sl.robot_state_publisher('simple_launch', 'turret.xacro', xacro_args = xacro_args)
        sl.joint_state_publisher(sources_list = ['source_joints'], use_gui = sl.arg('use_gui'))

    return sl.launch_description()
```

### Conditional blocks and includes

The file below fires up either `robot1` or `robot2` (or both) and also has a boolean argument to spawn `RViz2`:

```
from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()

    # conditional args
    sl.declare_arg('robot1', default_value=True, description='use robot 1')
    sl.declare_arg('robot2', default_value=True, description='use robot 2')
    sl.declare_arg('no_robot2', default_value=False, description='cancel use of robot 2')
    sl.declare_arg('rviz', default_value=True, description='Bringup RViz2')

    # numeric args
    sl.declare_arg('robot2_x', default_value=1, description='x-offset of robot 2')
    sl.declare_arg('robot2_y', default_value=1, description='y-offset of robot 2')

    with sl.group(if_arg='robot1'):
        sl.include('simple_launch', 'included_launch.py',
                   launch_arguments = {'prefix': 'robot1'})

    with sl.group(if_arg='robot2'):

        with sl.group(unless_arg='no_robot2'):

            args = {'prefix': 'robot2', 'x':sl.arg('robot2_x'), 'y': sl.arg('robot2_y')}
            sl.include('simple_launch', 'included_launch.py', launch_arguments=args)

    with sl.group(if_arg='rviz'):
        rviz_config = sl.find('simple_launch', 'turret.rviz')
        sl.node('rviz2', 'rviz2', arguments = ['-d', rviz_config])

    return sl.launch_description()
```

### Conditions with OpaqueFunction

The file below does the same as the previous one, but using an `OpaqueFunction`:

```
from simple_launch import SimpleLauncher

# declare simple launcher and the launch arguments in the main body
sl = SimpleLauncher()

# conditional args
sl.declare_arg('robot1', default_value=True, description='use robot 1')
sl.declare_arg('robot2', default_value=True, description='use robot 2')
sl.declare_arg('no_robot2', default_value=False, description='cancel use of robot 2')
sl.declare_arg('rviz', default_value=True, description='Bringup RViz2')

# numeric args
sl.declare_arg('robot2_x', default_value=1, description='x-offset of robot 2')
sl.declare_arg('robot2_y', default_value=1, description='y-offset of robot 2')

# string args
sl.declare_arg('included', default_value = 'included_launch')


# define the opaque function, context will be wrapped in the SimpleLauncher instance
def launch_setup():

    # we can use raw if as `robot1` argument is performed to a Boolean
    if sl.arg('robot1'):
        sl.include('simple_launch', 'included_launch.py', launch_arguments = {'prefix': 'robot1'})

    # and even combine conditions
    if sl.arg('robot2') and not sl.arg('no_robot2'):

            args = {'prefix': 'robot2', 'x':sl.arg('robot2_x'), 'y': sl.arg('robot2_y')}
            # summing up args and strings
            sl.include('simple_launch', sl.arg('included') + '.py', launch_arguments=args)

    if sl.arg('rviz'):
        rviz_config = sl.find('simple_launch', 'turret.rviz')
        sl.node('rviz2', 'rviz2', arguments = ['-d', rviz_config])

    return sl.launch_description()


# wrap the opaque_function in the launch description
# /!\ no `def generate_launch_description():`

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
```


### Composition

The file below is another way to write the [composition launch example](https://index.ros.org/doc/ros2/Tutorials/Composition/#composition-using-launch-actions):

```
from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()

    with sl.container(name='my_container', output='screen'):
        sl.node(package='composition', plugin='Talker', name='talker')
        sl.node(package='composition', plugin='Listener', name='listener')

    return sl.launch_description()
```

### auto sim time

Here we run Ignition and force all other nodes to `use_sim_time:=True`, unless this file is included from another one with `use_sim_time:=False`.
This is unlikely as this launch file spawns a simulator.

```
from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():

    # all nodes in this launch file will use_sim_time:=True
    sl = SimpleLauncher(use_sim_time=True)

    # run Gazebo + clock bridge
    sl.include('ros_ign_gazebo','ign_gazebo.launch.py',launch_arguments={'''some sdf world'''}})
    sl.create_gz_clock_bridge()

    # run other nodes with sim time

    return sl.launch_description()

```

### Robot description and conditionnal Gazebo bridge

The file below only runs by default a `robot_state_publisher` with `use_sim_time=False`.
However, if it is included from another file with `use_sim_time:=True` then it also spawns the robot into Gazebo and run two bridges for joint states and pose.

```
from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=False)

    # namespace is a launch argument, not a Python string
    sl.declare_arg('robot', default_value = 'robot1')
    robot = sl.arg('robot')

    with sl.group(ns = robot):
        # robot_state_publisher is always run
        sl.robot_state_publisher('my_description', 'my_robot.xacro')

        with sl.group(if_condition = sl.sim_time):
            # only execute this group if use_sim_time was set to True

            # spawn in Ignition at default pose if not already here
            # uses GazeboBridge.has_model(robot) under the hood and calls ros_ign_gazebo::create
            sl.spawn_gz_model(robot)

            # create a bridge for joint states @ /world/<world>/model/<robot>/joint_state
            # note the relative ROS topic 'joint_states' that is actually namespaced
            gz_js_topic = sl.name_join(GazeboBridge.model_prefix(robot), '/joint_state')
            js_bridge = GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)

            # pose publisher bridge @ /model/<robot>
            pose_bridge = GazeboBridge(sl.name_join('/model/', robot, '/pose'),
                                            'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros)

            # create bridge node with these two topics with default name gz_bridge
            sl.create_gz_bridge([js_bridge, pose_bridge])

    return sl.launch_description()

```
