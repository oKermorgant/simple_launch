^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2025-02-16)
-------------------
* handle gz world export to SDF after some delay
* make gz_launch compatible with substitutions
* Contributors: Olivier Kermorgant

1.10.1 (2024-07-05)
-------------------
* check for gz/ign executable being available
* gz_world_tf + better guess on Gz vs Ign
* Contributors: Olivier Kermorgant

1.10.0 (2024-06-16)
-------------------
* scope_included_files to include other launch files in a Group and avoid changing my scope
* remove dead code about prefixing gz plugins
* forward sim_time even for nodes that load a parameter file
* lazy GazeboBridge
* Contributors: Olivier Kermorgant

1.9.2 (2024-03-25)
------------------
* Gazebo basic example: launch SDF world + spawn from xacro
* remove dead code related to Gazebo.
* sl.arg returns a SimpleSubstitution to allow concatenation
* auto-detect Gazebo world name, allows running the simulation + spawn models in the same launch file
* Type debug on String being Iterable
* better handling of non-string choices in argument declaration
* more robust to various gz/ros combinations
* Contributors: Olivier Kermorgant

1.9.1 (2024-02-20)
------------------
* simplify + debug logic of SimpleSubstitution divisions
* image instead of /image to be detected as an image topic
* remove auto_sim_time function
* more robust to various combinations of ROS and Gazebo
* default Gazebo is still ignition Fortress, better error message about GZ_VERSION
* add new bridges for Gazebo
* Contributors: Olivier Kermorgant

1.9.0 (2023-11-23)
------------------
* update documentation
* warnings/errors on misuse of GazeboBridge
* move example executable to share directory
* dict cannot be updated with | in Foxy, fallback to dict.update
* absolute container name when loading in existing container
* py_eval can now handle conditions
* Contributors: Olivier Kermorgant

1.8.0 (2023-11-20)
------------------
* sync documentation and examples
* when instead of after for events
* scoped events
* add event wrapper
* Contributors: Olivier Kermorgant

1.7.2 (2023-10-02)
------------------
* simplify logic of try_perform
* Merge pull request `#5 <https://github.com/oKermorgant/simple_launch/issues/5>`_ from okvik/devel
  Fix string-valued launch argument substitution
* slight refactor to prepare wrapping Delays
* Contributors: Olivier Kermorgant, Viktor Pocedulic

* slight refactor to prepare wrapping Delays
* Contributors: Olivier Kermorgant

1.7.1 (2023-05-16)
------------------
* list[] -> List[] for type hints in 20.04
* detail on __ns for services
* add service call at launch
* Contributors: Olivier Kermorgant

1.7.0 (2023-02-22)
------------------
* GazeboBridge creates a camera_info topic for all bridged image topics
* Contributors: Olivier Kermorgant

1.6.2 (2023-01-23)
------------------
* add .rviz shortcut, clean Python scripts
* Contributors: Olivier Kermorgant

1.6.1 (2022-10-10)
------------------
* fix bug in robot_state_publisher with raw URDF
* Contributors: Olivier Kermorgant

1.6.0 (2022-10-06)
------------------
* check use_sim_time versus a parameter file
* resolve any builtin type as Substitution, not only text
* improve documentation
* Gazebo + humble compat
* OpaqueFunction wrapper and SimpleSubstitution
* switch to CMake to avoid deprecation messages
* more spawn_gz
* Contributors: Olivier Kermorgant

1.5.0 (2022-08-26)
------------------
* allow spawning a Gz model from file
* parameters are list of dict
* allow substitution for topics in ros_ign_image bridge
* Gazebo bridge handles invalid ROS topics through yaml config file. Image bridges are automatically run through ros_ign_image
* better handling of gazebo spawn pose
* Contributors: Olivier Kermorgant

1.4.1 (2022-06-01)
------------------
* more handling of param types
* Contributors: Olivier Kermorgant

1.4.0 (2022-06-01)
------------------
* use Gazebo instead of Ignition
* better type comprehension
* Contributors: Olivier Kermorgant

1.3.1 (2022-05-02)
------------------
* use underscores in setup.cfg
* typo in included_launch example
* Merge pull request `#4 <https://github.com/oKermorgant/simple_launch/issues/4>`_ from yushijinhun/patch-1
  Add package & executable parameter to container
* [readme] add doc for container.package
* Add package & executable param to container
  This allows the user to use a component container implementation
  other than `component_container`, such as `component_container_isolated`
  and `component_container_mt`.
* Contributors: Haowei Wen, Olivier Kermorgant

1.3.0 (2022-02-03)
------------------
* more use_sim_time: constructor + warning on 'auto'
* auto_sim_time can be forced instead of checking /clock
* Contributors: Olivier Kermorgant

1.2.1 (2022-01-24)
------------------
* add Ignition support
* Contributors: Olivier Kermorgant

1.2.0 (2021-11-02)
------------------
* make it clearer for the prefix_gz_plugins param
* Contributors: Olivier Kermorgant

1.1.0 (2021-09-25)
------------------
* add capacity to prefix and namespace Gazebo-published messages, as an alternative to using Gazebo namespaces that also remap /tf
* Contributors: Olivier Kermorgant

1.0.6 (2021-09-22)
------------------
* node arguments is a raw list and accepts substitutions
* Contributors: Olivier Kermorgant

1.0.5 (2021-09-22)
------------------
* automagically convert to expected type for node parameters, remappings, launch arguments
* Contributors: Olivier Kermorgant

1.0.4 (2021-09-03)
------------------
* some Python2 legacy bugs
* Contributors: Olivier Kermorgant

1.0.3 (2021-04-17)
------------------
* solve typo in node_args
* better handling of node arguments in nested lists
* bug fixes around Substitutions
* Merge pull request `#1 <https://github.com/oKermorgant/simple_launch/issues/1>`_ from oKermorgant/master
  deduce executable from package name if needed
* Contributors: Olivier Kermorgant

1.0.2 (2020-12-08)
------------------
* composition with existing container
* Contributors: Olivier Kermorgant

1.0.1 (2020-10-12)
------------------
* PythonExpression shortcut
* xacro args now take substitution args
* add LoadComposableNode feature for existing containers
* add name_join and path_join to combine raw str and Substitution stuff
* first commit
* Contributors: Olivier Kermorgant
