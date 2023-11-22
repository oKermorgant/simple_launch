from simple_launch import SimpleLauncher

sl = SimpleLauncher()
sl.declare_arg('listener1', True)
sl.declare_arg('listener2', False)


def launch_setup():

    # load Talker into new container
    with sl.container(name='my_container'):
        sl.node(package='composition', plugin='Talker', name='talker')

        # load Listener into this container if corresponding arg is True
        # loaded in a namespace but topic is remapped
        if sl.arg('listener1'):
            sl.node(package='composition', plugin='Listener', name='listener1',
                    remappings = {'chatter': '/chatter'},
                    namespace = 'other_ns')

    # defining groups in containers with Python logic
    if sl.arg('listener2'):
        with sl.container(name='/my_container', existing=True):
            sl.node(package='composition', plugin='Listener', name='listener2',
                        remappings = {'chatter': '/chatter'},
                        namespace = 'ns2')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
