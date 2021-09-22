from simple_launch import SimpleLauncher

def generate_launch_description():
    '''
    
    Launch description for a single robot - runs the two nodes in their own namespace
    '''
    sl = SimpleLauncher()
        
    sl.declare_arg('prefix', default_value = '', description='namespace of the robot (also tf prefix)')
    sl.declare_arg('x', default_value = 0, description='x-offset of the robot')
    sl.declare_arg('y', default_value = 0, description='y-offset of the robot')
    sl.declare_arg('use_gui', default_value = True, description='Use JSP gui')
    
    xacro_args = sl.arg_map(('prefix', 'x', 'y'))
    xacro_args['prefix'] = [xacro_args['prefix'], '/']  # cannot just sum a launch argument and a string
    
    with sl.group(ns=sl.arg('prefix')):
        sl.robot_state_publisher('simple_launch', 'turret.xacro', xacro_args = xacro_args)
        sl.joint_state_publisher(sources_list = ['source_joints'], use_gui = sl.arg('use_gui'))
        
    return sl.launch_description()
