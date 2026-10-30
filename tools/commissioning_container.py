"""Host entry for Mark's workstation. check maps no devices or host network."""
import argparse,json,os,subprocess,sys
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT))
IMAGE='ros@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617'


def docker_args(project,site_dir,native_dir,*,devices=(),run_dir=None):
    args=['docker','run','--rm','--pull','never','--read-only','--cap-drop','ALL','--security-opt','no-new-privileges',
        '--user',str(os.getuid())+':'+str(os.getgid()),'--tmpfs','/tmp:rw,size=128m',
        '-e','HOME=/tmp','-e','ROS_LOG_DIR=/tmp/ros-logs','-e','PYTHONDONTWRITEBYTECODE=1','-e','PYTHONPATH=/project',
        '--mount','type=bind,src='+str(project)+',dst=/project,readonly',
        '--mount','type=bind,src='+str(site_dir)+',dst=/site,readonly',
        '--mount','type=bind,src='+str(native_dir)+',dst=/native,readonly']
    if devices:
        args+=['--network','host']
        for group in os.getgroups():args+=['--group-add',str(group)]
        for device in devices:args+=['--device',device+':'+device+':rw']
        args+=['--mount','type=bind,src=/dev/serial/by-id,dst=/dev/serial/by-id,readonly',
               '--mount','type=bind,src='+str(run_dir)+',dst=/run-out']
    else:args+=['--network','none']
    args +=[IMAGE,'python3','-B','-m','atarm_controller.commissioning_launcher']
    return args


def main(args=None):
    p=argparse.ArgumentParser();p.add_argument('action',choices=('check','run'));p.add_argument('--site-dir',type=Path,required=True)
    p.add_argument('--native-dir',type=Path,required=True);p.add_argument('--project',type=Path,default=PROJECT)
    p.add_argument('--connect-hardware',action='store_true');p.add_argument('--run-dir',type=Path)
    a=p.parse_args(args)
    if a.action=='run' and (not a.connect_hardware or a.run_dir is None):
        p.error('run requires explicit --connect-hardware and a new --run-dir')
    for name in ('project','site_dir','native_dir'):
        path=getattr(a,name).resolve(strict=True)
        if not path.is_dir():p.error(name+' must be an existing directory')
        setattr(a,name,path)
    if a.run_dir and a.run_dir.exists():p.error('run-dir must not already exist')
    check=docker_args(a.project,a.site_dir,a.native_dir)+['check','--site','/site/site.json']
    result=subprocess.run(check,capture_output=True,text=True)
    if a.action=='check' or result.returncode!=0:
        print(result.stdout,end='');print(result.stderr,end='',file=sys.stderr);return result.returncode
    # No device metadata is inspected until the explicit run gate AND the
    # no-device container preflight have passed.
    from atarm_controller.physical_uart import LinuxUartBackend
    from atarm_controller.commissioning_config import file_sha
    report=json.loads(result.stdout)
    if not report.get('ready_for_run'):raise ValueError('Container preflight did not confirm readiness')
    site_file=a.site_dir/'site.json'
    if file_sha(site_file)!=report['site_sha256']:raise ValueError('Site changed since preflight')
    site=json.loads(site_file.read_text(encoding='utf-8-sig'))
    session_file=(a.site_dir/site['session_file']).resolve()
    if file_sha(session_file)!=report['session_sha256']:raise ValueError('Session changed since preflight')
    backend=LinuxUartBackend();ports=[backend.inspect(site['ports'][bus]) for bus in ('active','deploy')]
    if len({p.rdev for p in ports})!=2:raise ValueError('Both aliases resolve to one device')
    # Use frozen site/session copies for the actual launch, after validation.
    a.run_dir=a.run_dir.resolve();a.run_dir.mkdir(parents=True,exist_ok=False)
    frozen=a.run_dir/'input';frozen.mkdir()
    site['session_file']='session.json'
    (frozen/'site.json').write_text(json.dumps(site,indent=2)+'\n',encoding='utf-8')
    (frozen/'session.json').write_bytes(session_file.read_bytes())
    native_path=site['native_library']['path']
    if not native_path.startswith('/native/') or '..' in Path(native_path).parts:
        raise ValueError('Container native library must live below /native/')
    command=docker_args(a.project,frozen,a.native_dir,devices=[p.path for p in ports],run_dir=a.run_dir)
    command+=['run','--site','/site/site.json','--connect-hardware','--run-dir','/run-out/node']
    (a.run_dir/'container_command.json').write_text(json.dumps(command,indent=2)+'\n')
    return subprocess.call(command)


if __name__=='__main__':raise SystemExit(main())
