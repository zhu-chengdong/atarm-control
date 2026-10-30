"""Launch gate checks; every path here is temporary or explicitly synthetic."""
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict
import hashlib,io,json,sys,tempfile,unittest,importlib.util
from pathlib import Path
from unittest.mock import patch
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT))
from atarm_controller.commissioning_config import validate_site,CONFIRMATIONS
from atarm_controller.commissioning_launcher import main
from atarm_controller.cartesian_simulation import simulation_config
from atarm_controller.physical_uart import PhysicalUartFactory,QualifiedPort,LinuxUartBackend


def fixture(folder):
    # These values are only for validator tests, never delivered as site data.
    site=json.loads((PROJECT/'config/commissioning/site.template.json').read_text())
    session=json.loads((PROJECT/'config/offline_fd_validation.json').read_text())
    session['calibration_kind']='commissioned';session['calibration_id']='unit-test-fixture'
    site['site_id']='unit-test-only';site['hardware_enabled']=True;site['session_file']='session.json'
    native=folder/'fake_library.so';native.write_bytes(b'not an ELF; must never load during check')
    site['native_library']={'path':str(native),'sha256':hashlib.sha256(native.read_bytes()).hexdigest()}
    for bus in ('active','deploy'):
        site['ports'][bus]={'by_id_path':'/dev/serial/by-id/unit-test-'+bus,'usb_serial':'test-'+bus,'usb_vid':'0403','usb_pid':'6001',
            'baud_rate':4800000,'data_bits':8,'parity':'N','stop_bits':1,'flow_control':'none','adapter_direction_control':'automatic','reply_timeout_s':.02}
    site['ros']={'domain_id':167,'rmw_implementation':'rmw_fastrtps_cpp','namespace':'/unit_test','pose_topic':'/unit_test/pose','source_status_topic':'/unit_test/source'}
    site['electrical']={'nominal_voltage_v':24.,'minimum_voltage_v':20.,'maximum_voltage_v':28.,'continuous_current_a':2.,'peak_current_a':3.,'peak_duration_s':1.,
        'monitoring':'external_manual','measurement_and_stop_procedure':'unit test fixture only'}
    site['stop']={'policy':'zero_effort_requires_external_support','external_stop_procedure':'unit test fixture only','auto_home':False,'auto_deploy':False}
    site['confirmations']={key:True for key in CONFIRMATIONS}
    site['confirmations'].update(user_name='test user',mark_name='test Mark field',date='2026-10-30',evidence_paths=['unit-test-fixture-not-field-data'])
    return site,session


def save(folder,site,session):
    (folder/'session.json').write_text(json.dumps(session));path=folder/'site.json';path.write_text(json.dumps(site));return path


class GateTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.folder=Path(self.temp.name);self.site,self.session=fixture(self.folder)
    def tearDown(self):self.temp.cleanup()

    def test_blank_templates_refuse_and_do_not_load_native(self):
        with patch('ctypes.CDLL',side_effect=AssertionError('native load forbidden')):
            report,*_=validate_site(PROJECT/'config/commissioning/site.template.json')
        self.assertFalse(report['ready_for_run']);self.assertFalse(report['serial_devices_opened']);self.assertGreater(len(report['issues']),10)

    def test_complete_fixture_preflight_is_read_only(self):
        path=save(self.folder,self.site,self.session)
        with patch('ctypes.CDLL',side_effect=AssertionError('native load forbidden')):
            report,*_=validate_site(path)
        self.assertTrue(report['ready_for_run'],report);self.assertFalse(report['native_library_loaded'])

    def test_explicit_run_gate_precedes_ros_and_hardware_imports(self):
        path=save(self.folder,self.site,self.session)
        with redirect_stdout(io.StringIO()),patch('ctypes.CDLL',side_effect=AssertionError('load forbidden')):
            self.assertEqual(main(['run','--site',str(path)]),2)
        self.assertNotIn('rclpy',sys.modules)

    def test_disabled_or_unconfirmed_refuses_even_with_run_flag(self):
        for key in ('hardware_enabled',*CONFIRMATIONS):
            site=deepcopy(self.site)
            if key=='hardware_enabled':site[key]=False
            else:site['confirmations'][key]=False
            path=save(self.folder,site,self.session)
            with redirect_stdout(io.StringIO()),patch('ctypes.CDLL',side_effect=AssertionError('load forbidden')):
                self.assertEqual(main(['run','--site',str(path),'--connect-hardware']),2)

    def test_removed_limit_never_falls_back_to_simulation_default(self):
        del self.session['controller']['position_tolerance_m']
        r,*_=validate_site(save(self.folder,self.site,self.session));self.assertFalse(r['ready_for_run'])
        self.assertTrue(any(x['path'].endswith('position_tolerance_m') for x in r['issues']))

    def test_wrong_hash_and_synthetic_calibration_refuse(self):
        self.site['native_library']['sha256']='0'*64;self.session['calibration_kind']='synthetic-pty-validation'
        r,*_=validate_site(save(self.folder,self.site,self.session));self.assertFalse(r['ready_for_run'])

    def test_wrong_device_and_line_settings_refuse(self):
        for field,value in [('by_id_path','/dev/ttyUSB0'),('baud_rate',None),('data_bits',7),('flow_control','rtscts'),('adapter_direction_control','manual')]:
            site=deepcopy(self.site);site['ports']['active'][field]=value
            r,*_=validate_site(save(self.folder,site,self.session));self.assertFalse(r['ready_for_run'])

    def test_mapping_and_timeout_budget_refuse(self):
        self.session['channels'][1]['motor_id']=self.session['channels'][0]['motor_id']
        self.site['ports']['active']['reply_timeout_s']=.2
        r,*_=validate_site(save(self.folder,self.site,self.session));self.assertFalse(r['ready_for_run'])

    def test_native_capability_checked_before_device_inspection(self):
        class OfflineLib:
            def __init__(self,path):pass
            def physical_uart_enabled(self):return False
        class NoBackend:
            def inspect(self,p):raise AssertionError('no device inspection before capability')
        f=PhysicalUartFactory(self.site['ports'],self.site['native_library']['path'],self.site['native_library']['sha256'],backend=NoBackend(),library_loader=OfflineLib)
        with self.assertRaisesRegex(ValueError,'CAPABILITY_DISABLED'):f()
        self.assertEqual(f.opened_ports,[])

    def test_both_identities_checked_before_either_open(self):
        class Lib:
            def __init__(self,path):pass
            def physical_uart_enabled(self):return True
        class Backend:
            count=0
            def inspect(self,p):
                self.count+=1
                if self.count==2:raise ValueError('injected USB mismatch')
                return QualifiedPort('/test',1,1,1)
            def open(self,p):raise AssertionError('must not open first device yet')
        f=PhysicalUartFactory(self.site['ports'],self.site['native_library']['path'],self.site['native_library']['sha256'],backend=Backend(),library_loader=Lib)
        with self.assertRaisesRegex(ValueError,'USB mismatch'):f()
        self.assertEqual(f.opened_ports,[])

    def test_production_backend_rejects_pty_alias(self):
        with self.assertRaises(ValueError):LinuxUartBackend().inspect({'by_id_path':'/dev/pts/1'})

    def wrapper(self):
        spec=importlib.util.spec_from_file_location('commissioning_container_test',PROJECT/'tools/commissioning_container.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

    def test_host_check_command_has_no_devices_or_host_network(self):
        wrapper=self.wrapper()
        with patch('os.getuid',return_value=1000,create=True),patch('os.getgid',return_value=1000,create=True):
            args=wrapper.docker_args(Path('/project'),Path('/site'),Path('/native'))
        self.assertNotIn('--device',args);self.assertEqual(args[args.index('--network')+1],'none')
        self.assertNotIn('/dev/serial/by-id',str(args))

    def test_host_run_refuses_before_docker_without_explicit_flag(self):
        wrapper=self.wrapper()
        with patch.object(wrapper.subprocess,'run',side_effect=AssertionError('Docker must not run')),redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):wrapper.main(['run','--site-dir',str(self.folder),'--native-dir',str(self.folder)])

    def test_bad_ros_name_is_rejected_before_constructing_worker(self):
        self.site['ros']['pose_topic']='/camera//pose'
        r,*_=validate_site(save(self.folder,self.site,self.session));self.assertFalse(r['ready_for_run'])


if __name__=='__main__':unittest.main()
