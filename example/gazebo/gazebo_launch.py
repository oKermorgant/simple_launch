from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    # run the simulation
    sl.gz_launch(sl.find('simple_launch', 'demo_world.sdf'), '-r')

    ns = 'turret'
    with sl.group(ns = ns):

        sl.robot_state_publisher('simple_launch', 'turret.xacro', xacro_args={'prefix': ns+'/'})

        sl.spawn_gz_model(ns)

        # build bridges
        bridges = [GazeboBridge.clock()]

        # cmd_vel for joints
        for joint in ('joint1', 'joint2', 'joint3'):
            bridges.append(GazeboBridge(f'{ns}/{joint}_cmd_vel', f'{joint}_cmd_vel',
                                        'std_msgs/Float64', GazeboBridge.ros2gz))

        # joint state feedback on /world/<world>/model/<model>/joint_state
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

        # image
        bridges.append(GazeboBridge(f'{ns}/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

        sl.create_gz_bridge(bridges)

    # also display in RViz
    sl.rviz(sl.find('simple_launch', 'turret_gazebo.rviz'))

    return sl.launch_description()
