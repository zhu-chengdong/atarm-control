"""Build separate artifacts; never opens a UART, PTY or hardware descriptor."""
import argparse,ctypes,hashlib,json,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def build(output,mode):
    folder=output/mode;folder.mkdir(parents=True,exist_ok=False)
    sdk=output/'vendor/lib';sdk.mkdir(parents=True,exist_ok=True)
    target=sdk/'libUnitreeMotorSDK_Linux64.so'
    if not target.exists():shutil.copyfile(ROOT/'native/vendor/lib/libUnitreeMotorSDK_Linux64.so',target)
    commands=[['cmake','-S',str(ROOT/'native'),'-B',str(folder),'-DATARM_ENABLE_PHYSICAL_UART='+('ON' if mode=='physical' else 'OFF')],
              ['cmake','--build',str(folder),'--target','a1_serial_adapter','-j2']]
    with (folder/'build.log').open('w') as log:
        for command in commands:subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
    library=folder/'liba1_serial_adapter.so';dll=ctypes.CDLL(str(library))
    dll.a1_physical_uart_enabled.restype=ctypes.c_int;cap=dll.a1_physical_uart_enabled()
    assert cap==(1 if mode=='physical' else 0)
    receipt={'mode':mode,'physical_uart_capability':bool(cap),'library':str(library),
        'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
        'sdk_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'commands':commands,'devices_opened':False}
    (folder/'build_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=('offline','physical','both'),default='offline')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    result=[build(a.output,m) for m in (('offline','physical') if a.mode=='both' else (a.mode,))]
    print(json.dumps(result,indent=2))
