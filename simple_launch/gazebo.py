from .simple_substitution import SimpleSubstitution


def only_show_args():
    '''
    Returns True if the launch file was launched only to show its arguments
    '''
    import sys
    return any(show_arg in sys.argv for show_arg in ('-s', '--show-args', '--show-arguments'))


def silent_exec(cmd):
    '''
    Executes the given command and returns the output
    Returns an empty list if any error
    '''
    from subprocess import check_output, STDOUT
    if isinstance(cmd, str):
        from shlex import split
        cmd = split(cmd)
    try:
        return check_output(cmd, stderr=STDOUT).decode().splitlines()
    except:
        return []


class GazeboBridge:
    gz2ros = '['
    ros2gz = ']'
    bidirectional = '@'
    models = None
    world_name = None

    # ros <-> gz mapping
    # from https://github.com/ignitionrobotics/ros_ign/blob/foxy/ros_ign_bridge/README.md
    msg_map = {'std_msgs/msg/Bool': 'ignition.msgs.Boolean',
 'std_msgs/msg/Empty': 'ignition.msgs.Empty',
 'std_msgs/msg/Float32': 'ignition.msgs.Float',
 'std_msgs/msg/Float64': 'ignition.msgs.Double',
 'std_msgs/msg/Header': 'ignition.msgs.Header',
 'std_msgs/msg/Int32': 'ignition.msgs.Int32',
 'std_msgs/msg/String': 'ignition.msgs.StringMsg',
 'geometry_msgs/msg/Quaternion': 'ignition.msgs.Quaternion',
 'geometry_msgs/msg/Vector3': 'ignition.msgs.Vector3d',
 'geometry_msgs/msg/Point': 'ignition.msgs.Vector3d',
 'geometry_msgs/msg/Pose': 'ignition.msgs.Pose',
 'geometry_msgs/msg/PoseStamped': 'ignition.msgs.Pose',
 'geometry_msgs/msg/Transform': 'ignition.msgs.Pose',
 'geometry_msgs/msg/TransformStamped': 'ignition.msgs.Pose',
 'geometry_msgs/msg/Twist': 'ignition.msgs.Twist',
 'nav_msgs/msg/Odometry': 'ignition.msgs.Odometry',
 'rosgraph_msgs/msg/Clock': 'ignition.msgs.Clock',
 'sensor_msgs/msg/BatteryState': 'ignition.msgs.BatteryState',
 'sensor_msgs/msg/CameraInfo': 'ignition.msgs.CameraInfo',
 'sensor_msgs/msg/FluidPressure': 'ignition.msgs.FluidPressure',
 'sensor_msgs/msg/Imu': 'ignition.msgs.IMU',
 'sensor_msgs/msg/Image': 'ignition.msgs.Image',
 'sensor_msgs/msg/JointState': 'ignition.msgs.Model',
 'sensor_msgs/msg/LaserScan': 'ignition.msgs.LaserScan',
 'sensor_msgs/msg/MagneticField': 'ignition.msgs.Magnetometer',
 'sensor_msgs/msg/PointCloud2': 'ignition.msgs.PointCloudPacked',
 'tf2_msgs/msg/TFMessage': 'ignition.msgs.Pose_V',
 'trajectory_msgs/msg/JointTrajectory': 'ignition.msgs.JointTrajectory'}

    @staticmethod
    def read_models():
        if GazeboBridge.models is not None:
            return
        if only_show_args():
            # set a dummy world model, we are not running anyway
            GazeboBridge.models = []
            GazeboBridge.world_name = 'default'
            print('\033[93mThis launch file will request information on a running Gazebo instance at the time of the launch\033[0m')
            return

        # TODO adapt to gz vs ig
        models = silent_exec('ign model --list')
        for line in models:
            if line.startswith('Requesting'):
                GazeboBridge.world_name = line.replace(']','[').split('[')[1]
                break
        else:
            print('\033[91mGazeboBridge: could not find any Gazebo instance, launch will probably fail\033[0m')
            return

        GazeboBridge.models = [line.strip('- ') for line in models if line.startswith('- ')]
        print('\033[92mGazeboBridge: connected to a running Gazebo instance\033[0m')

    def __init__(self, gz_topic, ros_topic, msg, direction):
        '''
        Create a bridge instance to be passed to SimpleLauncher.create_gz_bridge
        '''

        if '/msg/' not in msg:
            msg = msg.replace('/', '/msg/')

        if msg not in self.msg_map:
            print(f'Cannot build a ros <-> gz bridge for message "{msg}": unknown type')
            return

        if not GazeboBridge.valid(direction):
            print(f'Cannot build ros <-> gz bridge with direction "{direction}": should be in {{[,],@}}')
            return

        self.gz_topic = gz_topic
        self.ros_topic = ros_topic

        # Images with gz2ros use ros_ign_image bridge
        if msg == 'sensor_msgs/msg/Image':
            self.is_image = True
            return

        self.is_image = False
        self.direction = direction
        self.ros_msg = msg

    def yaml(self, gz_ign):
        '''
        use YAML-based config for other bridges
        - topic_name: "chatter"
          ign_topic_name: "ign_chatter"
          ros_type_name: "std_msgs/msg/String"
          ign_type_name: "ignition.msgs.StringMsg"
          direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
                                    # "IGN_TO_ROS" - Bridge Ignition topic to ROS
                                    # "ROS_TO_IGN" - Bridge ROS topic

        takes gz prefix that is either gz or ign depending on current ROS 2 version
        '''

        direction = 'BIDIRECTIONAL'
        if self.direction == self.gz2ros:
            direction = f'{gz_ign.upper()}_TO_ROS'
        elif self.direction == self.ros2gz:
            direction = f'ROS_TO_{gz_ign.upper()}'

        return SimpleSubstitution(f'- {gz_ign}_topic_name: ', self.gz_topic, '\n',
                                 f'  {gz_ign}_type_name: ', self.msg_map[self.ros_msg], '\n',
                                 '  ros_topic_name: ',self.ros_topic, '\n',
                                 '  ros_type_name: ', self.ros_msg, '\n',
                                 '  direction: ', direction, '\n')

    @staticmethod
    def valid(direction):
        return direction in (GazeboBridge.gz2ros, GazeboBridge.ros2gz, GazeboBridge.bidirectional)

    @staticmethod
    def world():
        GazeboBridge.read_models()
        return GazeboBridge.world_name

    @staticmethod
    def model_prefix(model):
        if isinstance(model, str):
            return f"/world/{GazeboBridge.world()}/model/{model}"
        return SimpleSubstitution(f"/world/{GazeboBridge.world()}/model/", model)

    @staticmethod
    def model_topic(model, topic):
        return SimpleSubstitution("/model/", model, '/', topic)

    @staticmethod
    def clock():
        return GazeboBridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', GazeboBridge.gz2ros)

    @staticmethod
    def has_model(model):
        '''
        Returns True (or equivalent substitution) if the model already exists in Gazebo
        '''
        GazeboBridge.read_models()
        if isinstance(model, str):
            return model in GazeboBridge.models
        # come on, we are checking a substitution against a command line output
        from launch.substitutions import PythonExpression
        return PythonExpression(["'", model, "' in ", str(GazeboBridge.models)])
