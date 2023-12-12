from .simple_substitution import SimpleSubstitution
from . import console


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
        return check_output(cmd, stderr=STDOUT).decode()
    except:
        return ''


def ros_gz_prefix():
    from os import environ
    return 'ign' if environ['ROS_DISTRO'] < 'humble' else 'gz'


def _get_gz_world_info(gz_exec):
    '''
    Tries to get the world name and models, assuming some gz_exec (ign / gz)
    '''
    out = silent_exec([gz_exec, 'model', '--list'])

    if 'timed out' in out:
        return None, None

    out = out.splitlines()
    world_name = None
    for line in out:
        if line.startswith('Requesting'):
            world_name = line.replace(']','[').split('[')[1]
    return world_name, [line.strip('- ') for line in out if line.startswith('- ')]


class GazeboBridge:
    gz2ros = '['
    ros2gz = ']'
    bidirectional = '@'
    models = None
    world_name = None

    gz_exec = None

    # ros <-> gz mapping
    # from https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge

    def generate_msg_map(ros_gz_bridge_readme):
        '''
        function to regenerate known messages from readme
        '''
        out = ["    msg_map = {"]

        for line in ros_gz_bridge_readme.splitlines():
            if '|' in line and '/msg/' in line and 'gz.msgs' in line:
                _,ros,gz,_ = line.split('|')
                out.append(f"    '{ros.strip()}': '{gz.strip()}',")
        print('\n'.join(out)[:-1] + '}')

    msg_map = {'actuator_msgs/msg/Actuators': 'gz.msgs.Actuators',
    'builtin_interfaces/msg/Time': 'gz.msgs.Time',
    'geometry_msgs/msg/Point': 'gz.msgs.Vector3d',
    'geometry_msgs/msg/Pose': 'gz.msgs.Pose',
    'geometry_msgs/msg/PoseArray': 'gz.msgs.Pose_V',
    'geometry_msgs/msg/PoseStamped': 'gz.msgs.Pose',
    'geometry_msgs/msg/PoseWithCovariance': 'gz.msgs.PoseWithCovariance',
    'geometry_msgs/msg/PoseWithCovarianceStamped': 'gz.msgs.PoseWithCovariance',
    'geometry_msgs/msg/Quaternion': 'gz.msgs.Quaternion',
    'geometry_msgs/msg/Transform': 'gz.msgs.Pose',
    'geometry_msgs/msg/TransformStamped': 'gz.msgs.Pose',
    'geometry_msgs/msg/Twist': 'gz.msgs.Twist',
    'geometry_msgs/msg/TwistStamped': 'gz.msgs.Twist',
    'geometry_msgs/msg/TwistWithCovariance': 'gz.msgs.TwistWithCovariance',
    'geometry_msgs/msg/TwistWithCovarianceStamped': 'gz.msgs.TwistWithCovariance',
    'geometry_msgs/msg/Vector3': 'gz.msgs.Vector3d',
    'geometry_msgs/msg/Wrench': 'gz.msgs.Wrench',
    'geometry_msgs/msg/WrenchStamped': 'gz.msgs.Wrench',
    'gps_msgs/msg/GPSFix': 'gz.msgs.NavSat',
    'nav_msgs/msg/Odometry': 'gz.msgs.OdometryWithCovariance',
    'rcl_interfaces/msg/ParameterValue': 'gz.msgs.Any',
    'ros_gz_interfaces/msg/Altimeter': 'gz.msgs.Altimeter',
    'ros_gz_interfaces/msg/Contact': 'gz.msgs.Contact',
    'ros_gz_interfaces/msg/Contacts': 'gz.msgs.Contacts',
    'ros_gz_interfaces/msg/Dataframe': 'gz.msgs.Dataframe',
    'ros_gz_interfaces/msg/Entity': 'gz.msgs.Entity',
    'ros_gz_interfaces/msg/Float32Array': 'gz.msgs.Float_V',
    'ros_gz_interfaces/msg/GuiCamera': 'gz.msgs.GUICamera',
    'ros_gz_interfaces/msg/JointWrench': 'gz.msgs.JointWrench',
    'ros_gz_interfaces/msg/Light': 'gz.msgs.Light',
    'ros_gz_interfaces/msg/ParamVec': 'gz.msgs.Param',
    'ros_gz_interfaces/msg/SensorNoise': 'gz.msgs.SensorNoise',
    'ros_gz_interfaces/msg/StringVec': 'gz.msgs.StringMsg_V',
    'ros_gz_interfaces/msg/TrackVisual': 'gz.msgs.TrackVisual',
    'ros_gz_interfaces/msg/VideoRecord': 'gz.msgs.VideoRecord',
    'rosgraph_msgs/msg/Clock': 'gz.msgs.Clock',
    'sensor_msgs/msg/BatteryState': 'gz.msgs.BatteryState',
    'sensor_msgs/msg/CameraInfo': 'gz.msgs.CameraInfo',
    'sensor_msgs/msg/FluidPressure': 'gz.msgs.FluidPressure',
    'sensor_msgs/msg/Image': 'gz.msgs.Image',
    'sensor_msgs/msg/Imu': 'gz.msgs.IMU',
    'sensor_msgs/msg/JointState': 'gz.msgs.Model',
    'sensor_msgs/msg/Joy': 'gz.msgs.Joy',
    'sensor_msgs/msg/LaserScan': 'gz.msgs.LaserScan',
    'sensor_msgs/msg/MagneticField': 'gz.msgs.Magnetometer',
    'sensor_msgs/msg/NavSatFix': 'gz.msgs.NavSat',
    'sensor_msgs/msg/PointCloud2': 'gz.msgs.PointCloudPacked',
    'std_msgs/msg/Bool': 'gz.msgs.Boolean',
    'std_msgs/msg/ColorRGBA': 'gz.msgs.Color',
    'std_msgs/msg/Empty': 'gz.msgs.Empty',
    'std_msgs/msg/Float32': 'gz.msgs.Float',
    'std_msgs/msg/Float64': 'gz.msgs.Double',
    'std_msgs/msg/Header': 'gz.msgs.Header',
    'std_msgs/msg/Int32': 'gz.msgs.Int32',
    'std_msgs/msg/String': 'gz.msgs.StringMsg',
    'std_msgs/msg/UInt32': 'gz.msgs.UInt32',
    'tf2_msgs/msg/TFMessage': 'gz.msgs.Pose_V',
    'trajectory_msgs/msg/JointTrajectory': 'gz.msgs.JointTrajectory',
    'vision_msgs/msg/Detection2D': 'gz.msgs.AnnotatedAxisAligned2DBox',
    'vision_msgs/msg/Detection2DArray': 'gz.msgs.AnnotatedAxisAligned2DBox_V'}

    @staticmethod
    def read_models():

        if GazeboBridge.models is not None:
            return

        if only_show_args():
            # set a dummy world model, we are not running anyway
            GazeboBridge.models = []
            GazeboBridge.world_name = 'default'
            console.warn('This launch file will request information on a running Gazebo instance at the time of the launch')
            return

        # GZ / IGN: look for a clue in compilation flag, otherwise try both
        # up to Iron, Fortress is still the packaged one but probably not the used one
        from os import environ
        candidates = ['gz', 'ign']
        if environ['ROS_DISTRO'] < 'humble':
            candidates = ['ign','gz']
        for key in ('GZ_VERSION', 'IGNITION_VERSION'):
            if key in environ:
                if environ[key] == 'fortress':
                    candidates = ['ign']
                else:
                    candidates = ['gz']
                break

        # try to call Gazebo according to the exec candidates, that should be running at this point
        for GazeboBridge.gz_exec in candidates:
            GazeboBridge.world_name, GazeboBridge.models = _get_gz_world_info(GazeboBridge.gz_exec)
            if GazeboBridge.world_name is not None:
                break

        if GazeboBridge.world_name is None:
            console.error(f'GazeboBridge: could not find any Gazebo instance (tried {" and ".join(candidates)}), run Gazebo before this launch file.')
            return

        console.info('GazeboBridge: connected to a running Gazebo instance')

    @staticmethod
    def use_prefix(prefix):
        '''
        Tell GazeboBridge to use either ign or gz prefix (default)
        '''
        GazeboBridge.gz_exec = prefix

    def __init__(self, gz_topic, ros_topic, ros_msg, direction, gz_msg = None):
        '''
        Create a bridge instance to be passed to SimpleLauncher.create_gz_bridge
        '''

        if '/msg/' not in ros_msg:
            ros_msg = ros_msg.replace('/', '/msg/')

        if gz_msg is not None:
            self.gz_msg = gz_msg
        elif ros_msg not in self.msg_map:
            console.error(f'Cannot build a ros <-> gz bridge for message "{ros_msg}": unknown type or give explicit gz_msg')
            return
        else:
            self.gz_msg = self.msg_map[ros_msg]

        if not GazeboBridge.valid(direction):
            console.error(f'Cannot build ros <-> gz bridge with direction "{direction}": use GazeboBrige.{{gz2ros,ros2gz,bidirectional}}')
            return

        self.gz_topic = SimpleSubstitution(gz_topic)
        self.ros_topic = SimpleSubstitution(ros_topic)

        # Images with gz2ros use ros_gz_image bridge
        self.is_image = ros_msg == 'sensor_msgs/msg/Image'
        self.direction = direction
        self.ros_msg = ros_msg

    def yaml(self):
        '''
        use YAML-based config for other bridges
        - topic_name: "chatter"
          ign_topic_name: "ign_chatter"
          ros_type_name: "std_msgs/msg/String"
          ign_type_name: "ignition.msgs.StringMsg"
          direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
                                    # "IGN_TO_ROS" - Bridge Ignition topic to ROS
                                    # "ROS_TO_IGN" - Bridge ROS topic
        '''

        if self.gz_exec is None:
            self.gz_exec = ros_gz_prefix()

        direction = 'BIDIRECTIONAL'
        if self.direction == self.gz2ros:
            direction = f'{self.gz_exec.upper()}_TO_ROS'
        elif self.direction == self.ros2gz:
            direction = f'ROS_TO_{self.gz_exec.upper()}'

        if self.gz_exec == 'ign':
            self.gz_msg = self.gz_msg.replace('gz.','ignition.')

        return SimpleSubstitution(f'- {self.gz_exec}_topic_name: ', self.gz_topic, '\n',
                                 f'  {self.gz_exec}_type_name: ', self.gz_msg, '\n',
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
