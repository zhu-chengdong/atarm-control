"""Dry preflight by default; physical UART run requires explicit user intent."""
import argparse,json,os,sys
from pathlib import Path
from .commissioning_config import validate_site


def main(args=None):
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=('check','run'))
    p.add_argument('--site',type=Path,required=True)
    p.add_argument('--connect-hardware',action='store_true')
    p.add_argument('--run-dir',type=Path)
    a=p.parse_args(args)
    try:report,site,session,site_path,session_path=validate_site(a.site)
    except (OSError,ValueError,TypeError,KeyError) as e:
        print(json.dumps({'ready_for_run':False,'error':str(e),'serial_devices_opened':False},ensure_ascii=False));return 2
    if a.action=='check':
        print(json.dumps(report,ensure_ascii=False,indent=2));return 0 if report['ready_for_run'] else 2
    if not a.connect_hardware:
        print(json.dumps({'ready_for_run':False,'error':'Explicit --connect-hardware required','serial_devices_opened':False}));return 2
    if not report['ready_for_run']:
        print(json.dumps(report,ensure_ascii=False,indent=2));return 2
    if a.run_dir is None:
        print('A new --run-dir is required for configuration snapshots and audit.');return 2
    # Freeze the exact validated values, avoiding a second read of an editable
    # session file between validation and construction of the ROS worker.
    a.run_dir.mkdir(parents=True,exist_ok=False)
    (a.run_dir/'site.json').write_text(json.dumps(site,indent=2)+'\n',encoding='utf-8')
    frozen_session=a.run_dir/'session.json';frozen_session.write_text(json.dumps(session,indent=2)+'\n',encoding='utf-8')
    (a.run_dir/'preflight.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    os.environ['ROS_DOMAIN_ID']=str(site['ros']['domain_id'])
    os.environ['RMW_IMPLEMENTATION']=site['ros']['rmw_implementation']
    # All ROS, SDK and UART imports are below the explicit connection gate.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from .cartesian_fd_node import FdCartesianNode
    from .physical_uart import PhysicalUartFactory
    library=(site_path.parent/site['native_library']['path']).resolve()
    factory=PhysicalUartFactory(site['ports'],library,site['native_library']['sha256'])
    ros_args=['--ros-args','-r','__ns:='+site['ros']['namespace'],
              '-r','atarm/camera/pose:='+site['ros']['pose_topic'],
              '-r','atarm/camera/source_status:='+site['ros']['source_status_topic']]
    rclpy.init(args=ros_args);node=None;executor=SingleThreadedExecutor()
    try:
        node=FdCartesianNode(frozen_session,factory);executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown()
        if node:node.destroy_node()
        audit={'physical_ports_opened':factory.opened_ports,'configured_ports':factory.configured_ports,
               'worker_alive':node.runner.thread.is_alive() if node else False,'runtime':node.runner.audit if node else None}
        (a.run_dir/'shutdown_audit.json').write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
