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

    # combining conditions amounts to nesting groups
    with sl.group(if_arg='robot2'):
        with sl.group(unless_arg='no_robot2'):
            args = {'prefix': 'robot2', 'x':sl.arg('robot2_x'), 'y': sl.arg('robot2_y')}
            sl.include('simple_launch', 'included_launch.py', launch_arguments=args)

    with sl.group(if_arg='rviz'):
        sl.rviz(sl.find('simple_launch', 'turret.rviz'))

    return sl.launch_description()
