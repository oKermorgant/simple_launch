from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()   # no scope for included launch file argument override
    sl.declare_arg('prefix', 'turtle')

    sl.include('simple_launch', 'scoped_included_launch.py',
               launch_arguments={'which': 'first', 'prefix': sl.arg('prefix') + 1})
    # at this point sl.arg('prefix') is now 'turtle1' or similar

    # this launch files is included with prefix := turtle12 or similar
    sl.include('simple_launch', 'scoped_included_launch.py',
               launch_arguments={'which': 'second', 'prefix': sl.arg('prefix') + '2'})

    return sl.launch_description()
