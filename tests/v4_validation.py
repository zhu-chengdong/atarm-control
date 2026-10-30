"""Bounded validation: isolated unit processes, PTY factory, existing ROS matrix."""
import argparse,json,re,subprocess,sys
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--native-dir',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
a.out.mkdir(parents=True,exist_ok=False);rows={}
for name in ('test_commissioning','test_fd_interfaces','test_cartesian_offline','test_cartesian_jobs'):
    r=subprocess.run([sys.executable,'-B','-m','unittest','discover','-s',str(PROJECT/'tests'),'-p',name+'.py'],capture_output=True,text=True,cwd=PROJECT)
    text=r.stdout+r.stderr;(a.out/(name+'.log')).write_text(text)
    count=re.search(r'Ran (\d+) tests',text);rows[name]={'tests':int(count.group(1)) if count else None,'passed':r.returncode==0}
r=subprocess.run([sys.executable,'-B',str(PROJECT/'tests/commissioning_linux_checks.py'),'--native-dir',str(a.native_dir),'--out',str(a.out/'linux_checks.json')],capture_output=True,text=True)
(a.out/'linux_checks.log').write_text(r.stdout+r.stderr);rows['linux_kernel_and_native']={'passed':r.returncode==0}
r=subprocess.run([sys.executable,'-B',str(PROJECT/'tests/ros_fd_matrix.py'),'--out',str(a.out/'ros_matrix'),'--case','all'],capture_output=True,text=True)
(a.out/'ros_matrix.log').write_text(r.stdout+r.stderr);rows['ros_matrix']={'passed':r.returncode==0}
result={'passed':all(x['passed'] for x in rows.values()),'groups':rows}
(a.out/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
raise SystemExit(0 if result['passed'] else 1)
