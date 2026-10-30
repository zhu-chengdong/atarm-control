"""Local pure-Python checks. No ROS, SDK loading, UART or workstation access."""
from pathlib import Path
import argparse,json,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);rows=[]
    for name in ('test_commissioning','test_fd_interfaces','test_cartesian_offline','test_cartesian_jobs'):
        command=[sys.executable,'-B','-m','unittest','discover','-s','tests','-p',name+'.py']
        r=subprocess.run(command,capture_output=True,text=True,cwd=ROOT)
        (out/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
        rows.append({'name':name,'exit_code':r.returncode})
    for strategy in ('alternating','coordinated'):
        r=subprocess.run([sys.executable,'-B','tools/run_cartesian_sim.py','--strategy',strategy,'--output',str(out/strategy)],capture_output=True,text=True,cwd=ROOT)
        (out/(strategy+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
        rows.append({'name':strategy,'exit_code':r.returncode})
    result={'passed':all(r['exit_code']==0 for r in rows),'python':sys.version,'scope':'Pure Python, synthetic plant, no hardware or ROS access','groups':rows}
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
