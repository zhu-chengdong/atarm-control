"""Linux kernel + actual compiled libraries, using test-created PTYs only."""
import argparse,hashlib,json,os,struct,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
PROJECT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(PROJECT),str(PROJECT/'tests')]
from atarm_controller.serial_adapter import NativeLibrary,NativeSerialAdapter,MotorCmd
from atarm_controller.physical_uart import PhysicalUartFactory,LinuxUartBackend,QualifiedPort
from pty_cartesian_world import WireBoard


class PtyBackend(LinuxUartBackend):
    def __init__(self,boards,fail_second=False):self.boards=boards;self.fail_second=fail_second;self.raw_fds=[]
    def inspect(self,port):
        board=self.boards[port['test_bus']];info=os.fstat(board.slave)
        assert os.ttyname(board.slave).startswith('/dev/pts/')
        return QualifiedPort(os.ttyname(board.slave),info.st_dev,info.st_ino,info.st_rdev)
    def open(self,q):
        fd=super().open(q);self.raw_fds.append(fd);return fd
    def configure(self,fd,port):
        if self.fail_second and port['test_bus']=='deploy':raise RuntimeError('injected second configuration failure')
        return super().configure(fd,port)


def ports():
    return {bus:{'test_bus':bus,'baud_rate':4800000,'data_bits':8,'parity':'N','stop_bits':1,'flow_control':'none',
        'adapter_direction_control':'automatic','reply_timeout_s':.02} for bus in ('active','deploy')}


class KernelChecks(unittest.TestCase):
    def setUp(self):
        self.positions=[0.,0.,0.,.04];self.boards={bus:WireBoard(bus,self.positions) for bus in ('active','deploy')}
        self.library=ARGS.native_dir/'physical/liba1_serial_adapter.so';self.sha=hashlib.sha256(self.library.read_bytes()).hexdigest()
    def tearDown(self):
        for board in self.boards.values():board.close()

    def test_build_capabilities_and_default_library_remains_offline(self):
        self.assertFalse(NativeLibrary().physical_uart_enabled())
        self.assertFalse(NativeLibrary(ARGS.native_dir/'offline/liba1_serial_adapter.so').physical_uart_enabled())
        self.assertTrue(NativeLibrary(self.library).physical_uart_enabled())

    def test_factory_custom_baud_and_real_a1_exchange_on_ptys(self):
        import fcntl
        backend=PtyBackend(self.boards);factory=PhysicalUartFactory(ports(),self.library,self.sha,backend=backend)
        adapters=factory()
        try:
            self.assertEqual(factory.opened_ports,['active','deploy'])
            self.assertTrue(all(p['baud_rate_readback']==4800000 for p in factory.configured_ports))
            for bus,mid in [('active',1),('deploy',2)]:
                sample=adapters[bus].exchange(MotorCmd(id=mid,q=.02,kp=.06,kd=.08))
                self.assertTrue(sample.valid,sample);self.assertLess(abs(sample.q-.02),.001)
                raw=bytearray(44);fcntl.ioctl(self.boards[bus].slave,0x802c542a,raw,True)
                self.assertEqual(struct.unpack('=IIIIB19sII',raw)[-2:],(4800000,4800000))
        finally:
            for adapter in adapters.values():adapter.close()

    def test_partial_startup_rolls_back_all_owned_handles(self):
        saved=[]
        class RecordingAdapter(NativeSerialAdapter):
            def __init__(self,*args,**kwargs):super().__init__(*args,**kwargs);saved.append(self)
        backend=PtyBackend(self.boards,fail_second=True)
        with patch('atarm_controller.physical_uart.NativeSerialAdapter',RecordingAdapter):
            with self.assertRaisesRegex(RuntimeError,'second configuration'):
                PhysicalUartFactory(ports(),self.library,self.sha,backend=backend)()
        self.assertEqual(len(saved),1);self.assertIsNone(saved[0]._handle)
        for fd in backend.raw_fds:
            with self.assertRaises(OSError):os.fstat(fd)

    def test_physical_library_rejects_regular_file_descriptor(self):
        with tempfile.TemporaryFile() as f:
            with self.assertRaises(RuntimeError):NativeSerialAdapter(f.fileno(),native_library=NativeLibrary(self.library))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--native-dir',type=Path,required=True);p.add_argument('--out',type=Path,required=True);ARGS=p.parse_args()
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(KernelChecks))
    ARGS.out.write_text(json.dumps({'tests':result.testsRun,'passed':result.wasSuccessful(),'physical_devices_opened':False,
        'only_test_created_ptys':True,'errors':len(result.errors),'failures':len(result.failures)},indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
