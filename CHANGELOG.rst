^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
