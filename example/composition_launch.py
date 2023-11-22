from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('listener1', True)
    sl.declare_arg('listener2', False)

    # load Talker into new container
    with sl.container(name='my_container'):
        sl.node(package='composition', plugin='Talker', name='talker')

    # load Listener into existing container if corresponding arg is True
    # loaded in a namespace but topic is remapped
    with sl.group(ns = 'other_ns', if_arg='listener1'):
        with sl.container(name='/my_container', existing=True):
            sl.node(package='composition',
                    plugin='Listener',  # could be composition::Listener, package namespace will be added
                    name='listener1',
                    remappings = {'chatter': '/chatter'})

    # defining groups in containers does not work as below
    # ComposableNode are added in pure Python logic and cannot inherit from any namespace or condition
    '''
    with sl.container(name='my_container', existing=True):
        with sl.group(ns = 'ns2', if_arg='listener2'):
            sl.node(package='composition', plugin='Listener', name='listener2',
                        remappings = {'chatter': '/chatter'})
    '''
    return sl.launch_description()
