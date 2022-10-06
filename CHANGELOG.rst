^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
