from simple_launch import SimpleLauncher

sl = SimpleLauncher(scope_included_files=True)
sl.declare_arg('prefix', 'turtle')


def launch_setup():

    sl.include('simple_launch', 'scoped_included_launch.py',
               launch_arguments={'which': 'first', 'prefix':sl.arg('prefix') + '1'})
    # at this point sl.arg('prefix') is still 'turtle' or similar

    # this launch files is included with prefix := turtle2 or similar
    sl.include('simple_launch', 'scoped_included_launch.py',
               launch_arguments={'which': 'second', 'prefix': sl.arg('prefix') + '2'})

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
