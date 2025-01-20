# simple_launch
This package provides a Python class to help writing ROS 2 launch files.

The motivation behind this package is that the ROS 2 launch Python syntax may not be suited for many simple cases such as running basic nodes, spawning a `robot_state_publisher`, and grouping nodes in namespaces or components.

**Thanks**: this package is inspired by [ros2_launch_util](https://github.com/bponsler/ros2_launch_util).

The entry point is the `SimpleLauncher` class, which has several capabilities.

## Basic syntax

### Namespace and argument parser initialization

`sl = SimpleLauncher(namespace = '', use_sim_time = None, scope_included_files = False)`

- will initialize all nodes relative to the given namespace
- if `use_sim_time` is a Boolean, creates a `use_sim_time` launch argument with this value as the default and forwards it to all nodes, unless explicitely specified when running the node
- if `use_sim_time` is `'auto'`, then `SimpleLauncher` will set it to `True` if the `/clock` topic is advertized (case of an already running simulation). **This may have side effects if the `/clock` topic is advertized but you want to use this launch file with system clock**.
- if `use_sim_time` is `None` (default) then no particular value is forwarded to the nodes
- if `scope_included_files` is `False` (default) then including another launch file that shares the same argument but while passing another value for this argument, will also [modify the value of this argument after the inclusion](https://robotics.stackexchange.com/questions/98997/ros2-foxy-python-launch-argument-scope-when-nesting-launch-files). This is the default behavior of `launch` but can be undesired. Inside an `OpaqueFunction` the behavior is to be scoped anyway (the argument value is resolved) and `scope_included_files` has no effect.

### Node registration

`sl.node(package, executable, **node_args)` where

- `package` is the node package
- `executable` is the name of the executable
- `node_args` are any additional `Node` arguments

### Launch file include

`sl.include(package, launch_file, launch_dir = None, launch_arguments=None)` where

- `package` is the package of the included launch file
- `launch_file` is the name of the launch file
- `launch_dir` is its directory inside the package share (`None` to have it found)
- `launch_arguments` is a dictionary of arguments to pass to the included launch file

### Call a service at launch

This line runs a temporary client that waits for a service and calls it when available:

`sl.call_service(server, request = None, verbosity = '')` where

- `server` is the path to some service (possibly namespaced). The service type is deduced when it becomes available.
- `request` is a dictionary representing the service request. If `None` or incomplete, will use the service request default values.
- `verbosity` let the underlying node describe what it is doing: `'req'` for request info, `'res'` for response info or both with `'reqres'`

If any request parameter is `__ns` it will be changed to the current namespace.


### Setting parameters

This line runs a temporary client that waits for a node and changes its parameters when available:

`sl.set_parameters(node_name, parameters: dict = {}, verbosity = '')` where

- `node_name` is the name of the node (possibly namespaced)
- `parameters` is a dictionary of (name, value) parameters to be set
- `verbosity` let the underlying node describe what it is doing: `'req'` for request info, `'res'` for response info or both with `'reqres'`

This calls the `set_parameters` service of the node with the passed types. Possible errors may happen if the parameters do not exist or are of a different type.

### Robust types for parameters

In the launch API, differents types are expected for:

- node parameters: a list of dictionaries
- node remappings: a list of (`key`, `value`) pairs
- included launch arguments: a list of (`key`, `value`) pairs
- xacro arguments: a concatenation of `key:=value` strings

The `sl.include`, `sl.node` and `xacro_args` calls allow using any type (the simplest being a single dictionary)  and will convert to the one expected by the API.


## Launch arguments

`simple_launch` allows declaring launch arguments and getting them in return.

### Declare a launch argument

`sl.declare_arg(name, default_value, description = None)`: declare and returns the argument

Contrary to the base API, the default value is a raw Python type.

### Retrieve a launch argument

`sl.arg(name)`: returns the argument `name` as a `SimpleSubstitution`

### Retrieve several arguments as a dictionary

`sl.arg_map('robot', 'x', 'y')`: returns `{'robot': <robot arg value>, 'x': <x arg value>, 'y': <y arg value>}`

Typical when forwarding some launch arguments to a node or an included launch file.

## Groups or scopes

Groups are created through the `with sl.group():` syntax and accept, a namespace an if/unless condition and an event.

Actions that are added in a scope inherit from all previous defined groups.

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

- Only one condition can be set in a group, nested condition must be combined first, or used in nested groups.
- Combining conditions coming from launch arguments can be done with `sl.py_eval` as shown below.

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


### From events

The `when` argument wraps events from the `launch.event_handlers` module. It combines an event and a delay (0 by default)

```
from simple_launch.events import When, OnProcessStart, OnProcessExit, OnProcessIO

  my_node = sl.node(...)   # reference node

  with sl.group(when = When(my_node, OnProcessStart, 1.)):
      sl.node(...)  # will run 1 s after main node starts

  with sl.group(when = When(my_node, OnProcessExit)):
      sl.node(...)  # will run as soon as the main node exists

  with sl.group(when = When(my_node, OnProcessIO, io = 'stdout'):
      # OnProcessIO events need a function changing the event into an action
      sl.add_action(lambda event: LogInfo(msg = 'Node says "{}"'.format(
                      event.text.decode().strip())))
      # several functions can be used if needed, they will be combined in a single one
      sl.add_action(lambda event: LogInfo(msg = 'Once again, node says "{}"'.format(
                      event.text.decode().strip())))

  with sl.group(when = When(delay = 2.)):
      sl.node(...)  # will run after 2 sec
```


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

***It is currently impossible to have group blocks within a container block, as containers can only accept `ComposableNode`s***. A `GroupAction` containing e.g. `PushRosNamespace` and a `ComposableNode` is not itself a `ComposableNode`.


## `use_sim_time`

The current `use_sim_time` setting can be retrieved through `sl.sim_time` that may be:

- `None`, if `use_sim_time` was not set in the `SimpleLauncher` constructor
- a raw Boolean, if `use_sim_time` was set to `'auto'`, depending on the `/clock` topic being advertized
- a Boolean launch argument, if `use_sim_time` was set to `True` or `False`

In all cases, if the `use_sim_time` parameter is explicitely given to a node, it will be used instead of the `SimpleLauncher` instance one.


## `OpaqueFunction` with implicit `.perform(context)`

Most of the use cases can be dealt with substitutions and `with sl.group` blocks.
In order to design more imperative launch files, the [`OpaqueFunction`](https://discourse.ros.org/t/simplifying-launch-argument-declaration-and-initialization-in-launch-files/24204) approach can be used. The main drawback is that potential errors are harder to track.

To do this with `simple_launch`:

  - the `SimpleLauncher` instance and the argument declaration should be done in the main body of your launch file.
  - then, define a function (e.g. `launch_setup`) that takes no argument, where the logic of the launch file resides.
  - all arguments obtained through `sl.arg` will be basic Python types, obtained from performing the substitutions.
  - finally just export `generate_launch_description = sl.launch_description(opaque_function = launch_setup)`.

Compare [`example_launch.py`](example/example_launch.py) and [`example_opaque_launch.py`](example/example_opaque_launch.py) to see the two approaches on the same logic.

Note that inside an `OpaqueFunction` the if/unless idiom reduces to a basic if/else:

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

## Other one-liners

Methods listed below return instances of `SimpleSubstitution` that represent any Substitution, but that provides concatenation (`+`) and path concatenation (`/`) operators. It is still a `Substitution`, not a raw Python type. If run from an `OpaqueFunction` the underlying Python variable is returned.

### String / substitution concatenation

The following syntax builds the `SimpleSubstitution` corresponding to `<robot arg>.xacro`:

`file_name = sl.arg('robot') + '.xacro'`

*deprecated*: `sl.name_join(sl.arg('robot'), '.xacro')`

### Path concatenation

The following syntax builds the `SimpleSubstitution` corresponding to `<package_path>/urdf/<robot arg>.xacro`:

```
file_name = sl.arg('robot') + '.xacro'
urdf_file = os.path.join(get_package_share_directory(package),'urdf')/file_name
```
Obviously if all the path elements are raw strings, you should use `os.path.join` all along.

*deprecated*: `sl.path_join(get_package_share_directory(package), sl.arg('robot'), '.xacro')`


### Find a share file

`path = sl.find(package, file_name = None, file_dir = None)` where:

- `package` is the name of the package or `None` if `file_name` is already an absolute path
- `file_name` is the name of the file to find
- `file_dir` is the path inside the package

If `file_dir` is `None` but `package` and `file_name` are raw strings then the `find` function will actually look for the file inside the package share, using `os.walk`.

If `file_name` is `None` then the function just returns the path to the package share directory (e.g. `get_package_share_directory(package)`)

### Robot state publisher

It is quite common to run a `robot_state_publisher` from a `urdf` or `xacro` file. The line below runs it at the current namespace / condition level:

`sl.robot_state_publisher(package, description_file, description_dir=None, xacro_args=None, **node_args)` where

- `description_file` is a URDF or xacro file
- `description_dir` is the sub-directory of the file. If omitted, let the script search for the file assuming it is a raw string
- `xacro_args` is a dictionary of arguments to forward to xacro
- `node_args` are any additional arguments for `robot_state_publisher` (remappings / parameters)

### Python expressions

`sl.py_eval` will evaluate the given arguments as a Python expression, possibly performed if in an Opaque Function.

```
# RGB color as a list of [0-255] integers
sl.declare_arg('color', [255,0,0])
# same color as a string of [0-1] numbers (URDF format), note the padding commas to get a string
xacro_color = "'" + sl.py_eval("' '.join(str(c/255) for c in ", sl.arg('color'), ')') + "'"
```

### Conditions

`sl.py_eval` can be used to combine conditions. It is robust to lower case `true` or `false` and will return a `SimpleSubstitution`.

```
sl.declare_arg('some_condition', True)
opposed = sl.py_eval('not ', sl.arg('some_condition'))
```

Note that `IfCondition` and `UnlessCondition` cannot be combined, [only the underlying `Substitution`s can](https://answers.ros.org/answers/414006/revisions/).

### Joint state publisher

`sl.joint_state_publisher(use_gui, **node_args)`: fires up a `joint_state_publisher`, with or without the gui, in the current namespace.

### Rviz

`sl.rviz(config_file = None, warnings = False)`: runs RViz on the given configuration file. If `warnings` is `False` (default) then runs with `log-level FATAL` in order to avoid many messages in the console.

Classical use case: `sl.rviz(sl.find('my_package', 'some_rviz_config.rviz'))`


### Fallback to low-level syntax

If any unavailable functionality is needed, the `sl.add_action(action)` function adds any passed `Action` at the current namespace / conditional / event level.

## Interaction with Gazebo / Ignition

*Note: Ignition being renamed to Gazebo, all tools in this section use Gazebo / gz names*

An effort was made to be robust to Ignition versus Gazebo uses, i.e. *ign* prefix is used for `foxy` and `galactic` while *gz* prefix is used from `humble`. `GZ_VERSION` and `IGNITION_VERSION` environment variables are also used to identify which version should be preferred.

### Launch Gazebo

The Gazebo launch file corresponding to the current ROS 2 distribution is launched with
```
sl.gz_launch(world_file, gz_arguments, full_world = None, save_after = 5.)
```
Namely, it will redirect to either `ros_ign_gazebo/ign_gazebo.launch.py` (`foxy`, `galactic`) or `ros_gz_sim/gz_sim.launch.py` (`humble`+).
The given `gz_arguments`, if any, will be forwarded either as the `ign_args` or `gz_args`, accordingly.

If the world file can be parsed then `SimpleLaunch` will detect its name and forward it to `GazeboBridge` functions.

If `full_world` is a raw string then the world will be saved into this file after the delay. This will include any URDF that was spawned in between.

### Spawn a model

The following function allows easily spawing a model from its `robot_description`:

`sl.spawn_gz_model(name, topic, model_file = None, spawn_args = [], only_new = True)`

- `name` is the name this model will get in Gazebo
- `topic` is the topic to obtain the model from, default is `robot_description` (relative to the current namespace)
- `model_file` is the path to the (urdf or sdf) file. If defined then this will spawn this model and ignore the topic
- `only_new` if True, will not spawn the model if it already exists in a running Gazebo instance
- `spawn_args` are any additional spawn arguments, e.g. the initial pose

**example:** `sl.spawn_gz_model('my_robot', model_file = sl.find('my_pkg', 'my_model.urdf'))`

### Declare initial pose

Calling `sl.declare_gazebo_axes()` will declare all 6 parameters `(x,y,z,roll,pitch,yaw)` with `0` as default values.
If any axis is given (e.g. `sl.declare_gazebo_axes(yaw = 3.14)` then only this parameter will be declared.

Such parameters can be retrieved through `sl.gazebo_axes_args()`. As a consequence, it is easy to spawn a model with:

```
sl.declare_gazebo_axes()
sl.robot_description(...)
sl.spawn_gz_model(name, spawn_args = sl.gazebo_axes_args())
```

### Save current world

If the simulation is reset after some models have been spawned, they will disappear. A convenient way to avoid this is to generate the full SDF world from a running simulation. The Gazebo GUI is able to export the current world but cannot resolve models that were spawned from e.g. a `robot_description` topic.

The script `generate_gz_world` will generate a SDF corresponding to the current simulation. If some models have been spawned from URDF and topics, the script will get the corresponding description and insert it into the SDF, assuming robot `name` has its description under `/name/robot_description`. It can also be called from a launch file with (see Gazebo example):

```
# you might want to add a delay to be sure all models have been spawned
sl.save_gz_world(full_world.sdf, after = 1.)
```

Such a self-contained SDF world file is then compatible with a reset of the simulation. See Gazebo example for launch file that checks if the full world is available.


### Gazebo bridge

The `GazeboBridge` class allows easily creating bridges when using Gazebo.

An instance is created with: `bridge = GazeboBridge(<gazebo_topic>, <ros_topic>, <ros_message>, direction, <gz_message> = None)` where `direction` is either:

- `GazeboBridge.gz2ros` for Gazebo -> ROS
- `GazeboBridge.ros2gz` for ROS -> Gazebo
- `GazeboBridge.bidirectional` for both

The Gazebo message type is [deduced from the ROS message type](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge) if not set. Remapping will be set to the given `ros_topic`.

The SimpleLauncher instance can then run a node dealing with declared bridges, as illustrated in the examples at this end of this document.

- ```sl.create_gz_bridge([bridges], <node_name>)```

If some bridges involve `sensor_msgs/Image` then a dedicated `ros_gz_image` bridge will be used. The corresponding `camera_info` topic will be also bridged.



### Interaction with Gazebo sim

Some bridges (e.g. `joint_states`) need to have information on the world name. This name can only be obtained while Gazebo is running, or by having a priori knowledge of the world file.

The `GazeboBridge` class has a few static methods to get information on the simulated world, namely:

- `GazeboBridge.world()` returns the current world name
- `GazeboBridge.set_world_name(world)` imposes the world name
- `GazeboBridge.model_prefix(model)` builds the Gazebo topic relative to the given model `/world/<world>/model/<model>`

They can be used under these conditions:

- `sl.gz_launch` was called first (in the same launch setup) and the world file could be parsed, in this case the world name from the file is used
- or `GazeboBridge.set_world_name(world)` was called first (in the same launch setup), in this case this world name is used
- if none of the above and a **running Gazebo** instance exists, in this case `GazeboBridge` will request information on the world

If none of these conditions hold, the launch file will not be able to get information on the world, and launch fill probably fail.

### Built-in bridges

A common instance of the bridge is the clock. This one can be:

- created with `GazeboBridge.clock()`: returns a `GazeboBridge` instance, not added to any node yet
- or run directly with `sl.create_gz_clock_bridge()` (actually runs `sl.create_gz_bridge(GazeboBridge.clock())`)

Another common but tedious bridge instance is the joint state topic of a given model. This topic includes, in Gazebo, the name of the world *and* of the model. Assuming the world name is known, the corresponding bridge can be created with:

- `GazeboBridge.joint_states_bridge(model)`
- actually runs `GazeboBridge('/world/<world>/model/<model>/joint_state', 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)`

### World TF publisher

If `/tf` is used with Gazebo, then the root frame used by Gazebo depends on the world name. This frame is usually `world` on the ROS side.
A trivial `static_transform_publisher` is run to connect these two frames if they have different names:

- `sl.gz_world_tf(world_frame = None)`: run a `static_transform_publisher` from `world` to `world_frame`. If `None` then detect the one currently used by Gazebo.

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

    xacro_args = sl.arg_map('prefix', 'x', 'y')
    xacro_args['prefix'] += '/'  # can sum substitutions and strings

    with sl.group(ns=sl.arg('prefix')):
        sl.robot_state_publisher('simple_launch', 'turret.xacro', xacro_args = xacro_args)
        sl.joint_state_publisher(use_gui = sl.arg('use_gui'))

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


# tell SimpleLauncher to rely on the opaque_function in the launch description
# /!\ no `def generate_launch_description():`

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
```


### Combining conditions

The file below shows how to use `sl.py_eval` to combine conditions. We have to build a valid Python expression, not forgetting the spaces around `and`, `or`, `not`, etc.

```
from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()

    cond1 = sl.declare_arg('cond1', True)
    cond2 = sl.declare_arg('cond2', False)

    for logic in ('and', 'or'):

        # builds <cond1> <logic> <cond2> Python expression
        combined = sl.py_eval(cond1, f' {logic} ', cond2)

        sl.log_info([f'{logic} condition is ', combined])

        with sl.group(if_condition = combined):
            sl.node('demo_nodes_cpp', 'talker', name = f'talker_{logic}')

        # also works with sl.node(..., condition = IfCondition(combined))

    return sl.launch_description()
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

### Events and parameters

This [example file](example/event_tutorial_launch.py) is another way to write the [event launch example](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html). If does a little more work and is a little smarter, as the passed `new_background_r` is applied as soon as it has a valid value.


### auto sim time

Here we run Gazebo and force all other nodes to `use_sim_time:=True`, unless this file is included from another one with `use_sim_time:=False`.
This is unlikely as this launch file spawns a simulator.

```
from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():

    # all nodes in this launch file will use_sim_time:=True
    sl = SimpleLauncher(use_sim_time=True)

    # run Gazebo + clock bridge
    sl.gz_launch(<path/to/some/sdf/world>, '-r')
    sl.create_gz_clock_bridge()

    # run other nodes with sim time

    return sl.launch_description()

```

### Robot description and conditionnal Gazebo bridge

The file below only runs by default a `robot_state_publisher` with `use_sim_time:=False`.
However, if it is included from another file with `use_sim_time:=True` then it also spawns the robot into Gazebo and runs two bridges for joint states and pose.

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

            # spawn in Gazebo at default pose if not already here
            sl.spawn_gz_model(robot)

            # create a bridge for joint states @ /world/<world>/model/<robot>/joint_state
            # note the relative ROS topic 'joint_states' that is actually namespaced
            gz_js_topic = GazeboBridge.model_prefix(robot)/'joint_state'
            js_bridge = GazeboBridge(gz_topic = gz_js_topic,
                                     ros_topic = 'joint_states',
                                     msg = 'sensor_msgs/JointState',
                                     direction = GazeboBridge.gz2ros)

            # pose publisher bridge @ /model/<robot>
            pose_bridge = GazeboBridge('/model'/robot/'/pose',
                                       'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros)

            # create bridge node with these two topics with default name gz_bridge
            sl.create_gz_bridge([js_bridge, pose_bridge])

    return sl.launch_description()

```

### Full Gazebo simulation

The file `example/gazebo/gazebo_launch.py` runs a basic simulation of a turret robot with a camera, also displayed in RViz.

```
ros2 launch simple_launch gazebo_launch.py
```

You can move the robot around with a joint velocity setpoint (`std_msgs/Float64`) on `/turret/joint#` and see the simulated camera on `/turret/image`.


## I want more examples

Other self-contained examples (and slides) have been used to teach the ROS 2 launch file system and the nav stack.

Various aspects of the launch systen are shown in the [anf_launch](https://github.com/oKermorgant/anf_launch) package.

A simple tutorial on `Nav2`  using `simple_launch` is available in the [anf_nav](https://github.com/oKermorgant/anf_nav) package.
