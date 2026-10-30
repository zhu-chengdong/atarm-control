from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    port_active_arg                = DeclareLaunchArgument('port_active',                 default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6CAQYY-if00-port0', description='Serial device for the three active bending A1 motors.')
    port_deploy_arg                = DeclareLaunchArgument('port_deploy',                 default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5N1XWM-if00-port0', description='Serial device for the deploy/retract A1 motor.')
    is_dry_run_arg                 = DeclareLaunchArgument('is_dry_run',                  default_value='false', description='Run without opening Unitree serial ports.')
    step_rate_hz_arg               = DeclareLaunchArgument('step_rate_hz',                default_value='50.0', description='ROS timer rate for stepping bend targets.')
    publish_rate_hz_arg            = DeclareLaunchArgument('publish_rate_hz',             default_value='20.0', description='Motor status publish rate.')
    auto_deploy_on_start_arg       = DeclareLaunchArgument('auto_deploy_on_start',        default_value='false', description='Deploy the arm immediately after node startup.')
    deploy_delta_length_arg        = DeclareLaunchArgument('deploy_delta_length',         default_value='0.05', description='Deploy cable delta length in metres.')
    micro_adjust_max_step_m_arg     = DeclareLaunchArgument('micro_adjust_max_step_m',     default_value='0.01', description='Max signed length increment for a micro-adjust command, in metres.')
    deploy_motor_torque_gain_scale_arg = DeclareLaunchArgument('deploy_motor_torque_gain_scale', default_value='0.0', description='Required explicit verified deploy gain >= 1; zero is an invalid unset sentinel.')
    active_motor_0_id_arg          = DeclareLaunchArgument('active_motor_0.id',           default_value='1', description='Motor ID for active cable 0.')
    active_motor_1_id_arg          = DeclareLaunchArgument('active_motor_1.id',           default_value='0', description='Motor ID for active cable 1.')
    active_motor_2_id_arg          = DeclareLaunchArgument('active_motor_2.id',           default_value='2', description='Motor ID for active cable 2.')
    active_motor_0_direction_arg   = DeclareLaunchArgument('active_motor_0.direction',    default_value='1', description='Direction for active cable 0, either 1 or -1.')
    active_motor_1_direction_arg   = DeclareLaunchArgument('active_motor_1.direction',    default_value='1', description='Direction for active cable 1, either 1 or -1.')
    active_motor_2_direction_arg   = DeclareLaunchArgument('active_motor_2.direction',    default_value='-1', description='Direction for active cable 2, either 1 or -1.')
    deploy_motor_id_arg            = DeclareLaunchArgument('deploy_motor.id',             default_value='2', description='Motor ID for deploy cable.')
    deploy_motor_direction_arg     = DeclareLaunchArgument('deploy_motor.direction',      default_value='0', description='Required explicit installation direction, +1 or -1; zero is an invalid unset sentinel.')

    ld.add_action(port_active_arg)
    ld.add_action(port_deploy_arg)
    ld.add_action(is_dry_run_arg)
    ld.add_action(step_rate_hz_arg)
    ld.add_action(publish_rate_hz_arg)
    ld.add_action(auto_deploy_on_start_arg)
    ld.add_action(deploy_delta_length_arg)
    ld.add_action(micro_adjust_max_step_m_arg)
    ld.add_action(deploy_motor_torque_gain_scale_arg)
    ld.add_action(active_motor_0_id_arg)
    ld.add_action(active_motor_1_id_arg)
    ld.add_action(active_motor_2_id_arg)
    ld.add_action(active_motor_0_direction_arg)
    ld.add_action(active_motor_1_direction_arg)
    ld.add_action(active_motor_2_direction_arg)
    ld.add_action(deploy_motor_id_arg)
    ld.add_action(deploy_motor_direction_arg)

    ''' ------------------------------------------------------------------------ '''
    port_active             = LaunchConfiguration('port_active')
    port_deploy             = LaunchConfiguration('port_deploy')
    is_dry_run              = LaunchConfiguration('is_dry_run')
    step_rate_hz            = LaunchConfiguration('step_rate_hz')
    publish_rate_hz         = LaunchConfiguration('publish_rate_hz')
    auto_deploy_on_start    = LaunchConfiguration('auto_deploy_on_start')
    deploy_delta_length     = LaunchConfiguration('deploy_delta_length')
    micro_adjust_max_step_m = LaunchConfiguration('micro_adjust_max_step_m')
    deploy_motor_torque_gain_scale = LaunchConfiguration('deploy_motor_torque_gain_scale')
    active_motor_0_id       = LaunchConfiguration('active_motor_0.id')
    active_motor_1_id       = LaunchConfiguration('active_motor_1.id')
    active_motor_2_id       = LaunchConfiguration('active_motor_2.id')
    active_motor_0_direction = LaunchConfiguration('active_motor_0.direction')
    active_motor_1_direction = LaunchConfiguration('active_motor_1.direction')
    active_motor_2_direction = LaunchConfiguration('active_motor_2.direction')
    deploy_motor_id         = LaunchConfiguration('deploy_motor.id')
    deploy_motor_direction  = LaunchConfiguration('deploy_motor.direction')

    node = Node(
        package='atarm_controller',
        executable='arm_controller',
        name='arm_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port_active':                port_active,
            'port_deploy':                port_deploy,
            'is_dry_run':                 ParameterValue(is_dry_run, value_type=bool),
            'step_rate_hz':               ParameterValue(step_rate_hz, value_type=float),
            'publish_rate_hz':            ParameterValue(publish_rate_hz, value_type=float),
            'auto_deploy_on_start':       ParameterValue(auto_deploy_on_start, value_type=bool),
            'deploy_delta_length':        ParameterValue(deploy_delta_length, value_type=float),
            'micro_adjust_max_step_m':    ParameterValue(micro_adjust_max_step_m, value_type=float),
            'deploy_motor_torque_gain_scale': ParameterValue(deploy_motor_torque_gain_scale, value_type=float),
            'active_motor_0.id':          ParameterValue(active_motor_0_id, value_type=int),
            'active_motor_1.id':          ParameterValue(active_motor_1_id, value_type=int),
            'active_motor_2.id':          ParameterValue(active_motor_2_id, value_type=int),
            'active_motor_0.direction':   ParameterValue(active_motor_0_direction, value_type=int),
            'active_motor_1.direction':   ParameterValue(active_motor_1_direction, value_type=int),
            'active_motor_2.direction':   ParameterValue(active_motor_2_direction, value_type=int),
            'deploy_motor.id':            ParameterValue(deploy_motor_id, value_type=int),
            'deploy_motor.direction':     ParameterValue(deploy_motor_direction, value_type=int),
        }],
    )
    ld.add_action(node)

    return ld
