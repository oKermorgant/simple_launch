from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg('listener', True)

    # load Talker into new container
    with sl.container(name='my_container', output='screen'):
        sl.node(package='composition', plugin='Talker', name='talker')

    # load Listener into existing container if corresponding arg is True
    with sl.group(if_arg='listener'):
        with sl.container(name='my_container', existing=True):
            sl.node(package='composition', plugin='Listener', name='listener')

    return sl.launch_description()
