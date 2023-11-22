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
# all sl.arg() returned values are raw Python types
def launch_setup():

    # we can use raw if's
    if sl.arg('robot1'):
        sl.include('simple_launch', 'included_launch.py', launch_arguments = {'prefix': 'robot1'})

    # and even combine conditions
    if sl.arg('robot2') and not sl.arg('no_robot2'):

        args = {'prefix': 'robot2', 'x':sl.arg('robot2_x'), 'y': sl.arg('robot2_y')}
        # summing up args and strings
        sl.include('simple_launch', sl.arg('included') + '.py', launch_arguments=args)

    if sl.arg('rviz'):
        sl.rviz(sl.find('simple_launch', 'turret.rviz'))

    return sl.launch_description()


# tell SimpleLauncher to rely on the opaque_function in the launch description
# /!\ no `def generate_launch_description():`

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
