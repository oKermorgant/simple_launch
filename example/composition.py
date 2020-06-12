from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    with sl.container(name='my_container', output='screen'):
        sl.node(package='composition', plugin='Talker', name='talker')
        sl.node(package='composition', plugin='Listener', name='listener')
        
    return sl.launch_description()
