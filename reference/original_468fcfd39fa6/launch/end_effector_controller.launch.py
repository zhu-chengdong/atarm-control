from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    dev_name_arg             = DeclareLaunchArgument('device_name',                 default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEQOED-if00-port0', description='Serial device connected to the DYNAMIXEL bus.')
    baudrate_arg             = DeclareLaunchArgument('baudrate',                    default_value='57600',            description='DYNAMIXEL serial baudrate.')
    is_dry_run_arg           = DeclareLaunchArgument('is_dry_run',                  default_value='false',            description='Run without opening the serial port.')
    publish_rate_hz_arg      = DeclareLaunchArgument('publish_rate_hz',             default_value='20.0',             description='Present angle publish rate.')
    pan_gear_ratio_arg       = DeclareLaunchArgument('pan_motion.gear_ratio',       default_value=str(154.0/19.0),    description='Pan output-to-servo gear ratio.')
    arc_gear_ratio_arg       = DeclareLaunchArgument('arc_motion.gear_ratio',       default_value=str(100.0/6.0*2.0), description='Arc output-to-servo gear ratio.')
    pan_invert_direction_arg = DeclareLaunchArgument('pan_motion.invert_direction', default_value='false',            description='Invert pan output angle direction.')
    arc_invert_direction_arg = DeclareLaunchArgument('arc_motion.invert_direction', default_value='true',             description='Invert arc output angle direction.')
    arc_min_output_deg_arg   = DeclareLaunchArgument('arc_motion.min_output_deg',   default_value='0.0',              description='Minimum arc target angle in degrees.')
    arc_max_output_deg_arg   = DeclareLaunchArgument('arc_motion.max_output_deg',   default_value='45.0',             description='Maximum arc target angle in degrees.')

    ld.add_action(dev_name_arg)
    ld.add_action(baudrate_arg)
    ld.add_action(is_dry_run_arg)
    ld.add_action(publish_rate_hz_arg)
    ld.add_action(pan_gear_ratio_arg)
    ld.add_action(arc_gear_ratio_arg)
    ld.add_action(pan_invert_direction_arg)
    ld.add_action(arc_invert_direction_arg)
    ld.add_action(arc_min_output_deg_arg)
    ld.add_action(arc_max_output_deg_arg)

    ''' ------------------------------------------------------------------------ '''
    device_name     = LaunchConfiguration('device_name')
    baudrate        = LaunchConfiguration('baudrate')
    is_dry_run      = LaunchConfiguration('is_dry_run')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    pan_gear_ratio       = LaunchConfiguration('pan_motion.gear_ratio')
    arc_gear_ratio       = LaunchConfiguration('arc_motion.gear_ratio')
    pan_invert_direction = LaunchConfiguration('pan_motion.invert_direction')
    arc_invert_direction = LaunchConfiguration('arc_motion.invert_direction')
    arc_min_output_deg   = LaunchConfiguration('arc_motion.min_output_deg')
    arc_max_output_deg   = LaunchConfiguration('arc_motion.max_output_deg')

    node = Node(
        package='atarm_controller',
        executable='end_effector_controller',
        name='end_effector_controller_node',
        output='screen',
        parameters=[{
            'device_name':                                device_name,
            'baudrate':                    ParameterValue(baudrate, value_type=int),
            'is_dry_run':                  ParameterValue(is_dry_run, value_type=bool),
            'publish_rate_hz':             ParameterValue(publish_rate_hz, value_type=float),
            'pan_motion.gear_ratio':       ParameterValue(pan_gear_ratio, value_type=float),
            'arc_motion.gear_ratio':       ParameterValue(arc_gear_ratio, value_type=float),
            'pan_motion.invert_direction': ParameterValue(pan_invert_direction, value_type=bool),
            'arc_motion.invert_direction': ParameterValue(arc_invert_direction, value_type=bool),
            'arc_motion.min_output_deg':   ParameterValue(arc_min_output_deg, value_type=float),
            'arc_motion.max_output_deg':   ParameterValue(arc_max_output_deg, value_type=float),
        }],
    )
    ld.add_action(node)


    return ld
