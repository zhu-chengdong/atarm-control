import argparse,json,sys,platform
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir()
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import rclpy
from geometry_msgs.msg import PoseStamped
from test_simulated_serial import Rig
from atarm_controller.ros_control_config import load_control_config
options,reference=load_control_config(ROOT/'config/ros_simulation_only.json')
rclpy.init();rig=Rig()
try:
    rig.construct();data={'rclpy':rclpy.__file__,'python':platform.python_version(),'geometry_msgs':True,'native_motor_feedbacks_valid':all(x['feedback_valid'] for x in rig.arm.get_feedbacks()),'reference':reference}
    (a.out/'results.json').write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data))
finally:rig.close();rclpy.shutdown()
