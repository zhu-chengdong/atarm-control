"""Offline configuration validation. Never opens serial paths or loads an SDK."""
import hashlib,json,math,re
from dataclasses import fields
from pathlib import Path
from .cartesian_control import CartesianConfig
from .external_vision import ExternalVision

CONFIRMATIONS=('power_and_basic_motion','support_and_stop','mapping_and_reference',
               'vision_and_clock','jacobian_and_workspace','sole_controller_owner')


def file_sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_site(path):
    site_path=Path(path).resolve()
    with site_path.open(encoding='utf-8-sig') as f:site=json.load(f)
    if not isinstance(site,dict):raise ValueError('Site configuration must be an object')
    session_name=site.get('session_file')
    if not isinstance(session_name,str) or not session_name:raise ValueError('session_file required')
    session_path=(site_path.parent/session_name).resolve()
    with session_path.open(encoding='utf-8-sig') as f:session=json.load(f)
    return site,session,site_path,session_path


def validate_site(path):
    site,session,site_path,session_path=load_site(path)
    issues=[];inhibits=[]
    def issue(key,reason):issues.append({'path':key,'reason':reason})
    def missing(value,key):
        if value is None or (isinstance(value,str) and (not value.strip() or 'TODO' in value or '待' in value)):
            issue(key,'required field is not confirmed');return
        if isinstance(value,dict):
            for k,v in value.items():missing(v,key+'.'+k)
        if isinstance(value,list):
            if not value:issue(key,'empty list')
            for i,v in enumerate(value):missing(v,key+f'[{i}]')
        if isinstance(value,float) and not math.isfinite(value):issue(key,'non-finite number')
    def positive(value):return type(value) in (float,int) and math.isfinite(value) and value>0
    for key in ('schema_version','session_file','site_id','native_library','ports','ros','electrical','stop','confirmations'):
        if key not in site:issue('site.'+key,'required section missing')
    missing(site,'site');missing(session,'session')
    if site.get('schema_version')!=1:issue('site.schema_version','unsupported schema')
    if site.get('hardware_enabled') is not True:inhibits.append('hardware_enabled is not true')
    confirmations=site.get('confirmations',{})
    for key in CONFIRMATIONS:
        if confirmations.get(key) is not True:inhibits.append('confirmation missing: '+key)
    for key in ('user_name','mark_name','date','evidence_paths'):
        if key not in confirmations:issue('site.confirmations.'+key,'required confirmation record missing')
    if confirmations.get('date') and not re.fullmatch(r'\d{4}-\d{2}-\d{2}',str(confirmations['date'])):
        issue('site.confirmations.date','expected YYYY-MM-DD')
    if session.get('calibration_kind')!='commissioned':issue('session.calibration_kind','requires actual commissioned calibration')
    if session.get('schema_version')!=1:issue('session.schema_version','unsupported session schema')
    for field in fields(CartesianConfig):
        if field.name not in session.get('controller',{}):issue('session.controller.'+field.name,'explicit value required; no simulation fallback')
    calibration_id=session.get('calibration_id')
    if not isinstance(calibration_id,str) or calibration_id.lower().startswith(('sim','synthetic','offline','validation')):
        issue('session.calibration_id','synthetic/absent calibration is not a site calibration')
    try:
        cfg=CartesianConfig(**session['controller'])
        ExternalVision(session['vision'],cfg.max_pose_age_s)
    except (ValueError,TypeError,KeyError) as e:issue('session.controller/vision',str(e));cfg=None
    channels=session.get('channels',[])
    if not isinstance(channels,list) or len(channels)!=4:issue('session.channels','exactly four ordered channels required');channels=[]
    identities=[]
    for i,c in enumerate(channels):
        key=f'session.channels[{i}]'
        if not isinstance(c,dict):issue(key,'channel object required');continue
        for field in ('bus','motor_id','direction','radius_m','gear_ratio','encoder_reference_rad','length_reference_m','kp','kd','max_feedback_torque_nm','max_temperature_c'):
            if field not in c:issue(key+'.'+field,'required channel field missing')
        bus=c.get('bus');mid=c.get('motor_id');identities.append((bus,mid))
        if bus!=('deploy' if i==3 else 'active'):issue(key+'.bus','first three active, fourth deploy')
        if type(mid) is not int or not 0<=mid<255 or mid==187:issue(key+'.motor_id','valid non-broadcast A1 ID required')
        if type(c.get('direction')) is not int or c['direction'] not in (-1,1):issue(key+'.direction','requires verified +1 or -1')
        for field in ('radius_m','gear_ratio','kp','kd','max_feedback_torque_nm','max_temperature_c'):
            if not positive(c.get(field)):issue(key+'.'+field,'positive measured/confirmed value required')
        for field in ('encoder_reference_rad','length_reference_m'):
            v=c.get(field)
            if type(v) not in (float,int) or not math.isfinite(v):issue(key+'.'+field,'finite reference required')
        if positive(c.get('kp')) and c['kp']>15.9:issue(key+'.kp','A1 codec bound exceeded')
        if positive(c.get('kd')) and c['kd']>31.9:issue(key+'.kd','A1 codec bound exceeded')
    if len(set(identities))!=len(identities):issue('session.channels','duplicate ID on same bus')
    if cfg and channels and tuple(c.get('length_reference_m') for c in channels)!=cfg.reference_lengths_m:
        issue('session.channels','length references disagree with controller')
    if not positive(session.get('speed_m_s')):issue('session.speed_m_s','positive confirmed cable speed required')
    ports=site.get('ports',{})
    if set(ports)!= {'active','deploy'}:issue('site.ports','active and deploy ports required')
    paths=[];serials=[]
    for name,port in ports.items():
        key='site.ports.'+name
        if not isinstance(port,dict):issue(key,'port object required');continue
        for field in ('by_id_path','usb_serial','usb_vid','usb_pid','baud_rate','data_bits','parity','stop_bits','flow_control','adapter_direction_control','reply_timeout_s'):
            if field not in port:issue(key+'.'+field,'required serial field missing')
        p=port.get('by_id_path');paths.append(p);serials.append(port.get('usb_serial'))
        if not isinstance(p,str) or not p.startswith('/dev/serial/by-id/') or '/..' in p or len(p.split('/'))!=5:
            issue(key+'.by_id_path','one stable /dev/serial/by-id entry required')
        for field in ('usb_vid','usb_pid'):
            if not isinstance(port.get(field),str) or not re.fullmatch(r'[0-9a-fA-F]{4}',port[field]):issue(key+'.'+field,'four hexadecimal digits required')
        if type(port.get('baud_rate')) is not int or not 0<port['baud_rate']<=10000000:issue(key+'.baud_rate','explicit supported positive baud required')
        if port.get('data_bits')!=8 or port.get('parity')!='N' or port.get('stop_bits')!=1 or port.get('flow_control')!='none':
            issue(key,'this launcher implements 8N1 without hardware/software flow control only')
        if port.get('adapter_direction_control')!='automatic':issue(key+'.adapter_direction_control','adapter must handle half duplex automatically; other control is not implemented')
        if not positive(port.get('reply_timeout_s')) or port.get('reply_timeout_s',99)>1:issue(key+'.reply_timeout_s','finite timeout in (0,1] required')
    if len(paths)==2 and paths[0]==paths[1]:issue('site.ports','ports must be distinct')
    if cfg and len(ports)==2 and all(positive(p.get('reply_timeout_s')) for p in ports.values()):
        budget=3*ports['active']['reply_timeout_s']+ports['deploy']['reply_timeout_s']+cfg.poll_s
        if budget>cfg.max_motor_age_s:issue('site.ports','serial timeout budget exceeds configured motor feedback age')
    ros=site.get('ros',{})
    for key in ('domain_id','rmw_implementation','namespace','pose_topic','source_status_topic'):
        if key not in ros:issue('site.ros.'+key,'required ROS setting missing')
    if type(ros.get('domain_id')) is not int or not 0<=ros['domain_id']<=232:issue('site.ros.domain_id','integer ROS domain in [0,232] required')
    if not isinstance(ros.get('rmw_implementation'),str) or not re.fullmatch(r'rmw_[a-zA-Z0-9_]+',ros['rmw_implementation']):issue('site.ros.rmw_implementation','explicit RMW implementation required')
    for key in ('namespace','pose_topic','source_status_topic'):
        if not isinstance(ros.get(key),str) or not re.fullmatch(r'/[a-zA-Z_][a-zA-Z0-9_]*(?:/[a-zA-Z_][a-zA-Z0-9_]*)*',ros[key]):issue('site.ros.'+key,'absolute ROS name required')
    stop=site.get('stop',{})
    if not isinstance(stop.get('external_stop_procedure'),str) or not stop.get('external_stop_procedure','').strip():
        issue('site.stop.external_stop_procedure','explicit external stop/support procedure required')
    if stop.get('policy')!='zero_effort_requires_external_support':issue('site.stop.policy','supported stop requires independent mechanical support')
    if stop.get('auto_home') is not False or stop.get('auto_deploy') is not False:issue('site.stop','auto-home and auto-deploy must both be false')
    electrical=site.get('electrical',{})
    for field in ('nominal_voltage_v','minimum_voltage_v','maximum_voltage_v','continuous_current_a','peak_current_a'):
        if not positive(electrical.get(field)):issue('site.electrical.'+field,'confirmed hardware rating/test bound required')
    if not positive(electrical.get('peak_duration_s')):issue('site.electrical.peak_duration_s','confirmed peak duration required')
    if not isinstance(electrical.get('measurement_and_stop_procedure'),str) or not electrical.get('measurement_and_stop_procedure','').strip():
        issue('site.electrical.measurement_and_stop_procedure','external monitoring procedure required')
    if positive(electrical.get('peak_current_a')) and positive(electrical.get('continuous_current_a')) and electrical['peak_current_a']<electrical['continuous_current_a']:
        issue('site.electrical','peak current must not be below continuous rating')
    if electrical.get('monitoring')!='external_manual':issue('site.electrical.monitoring','only external/manual electrical monitoring is provided')
    if all(positive(electrical.get(x)) for x in ('minimum_voltage_v','nominal_voltage_v','maximum_voltage_v')):
        if not electrical['minimum_voltage_v']<electrical['nominal_voltage_v']<electrical['maximum_voltage_v']:issue('site.electrical','voltage bounds must bracket nominal voltage')
    lib=site.get('native_library',{});libpath=lib.get('path');libhash=lib.get('sha256');binary_hash_valid=False
    if not isinstance(libpath,str) or not libpath:issue('site.native_library.path','explicit native library path required')
    if not isinstance(libhash,str) or not re.fullmatch(r'[0-9a-f]{64}',libhash):issue('site.native_library.sha256','verified SHA256 required')
    if isinstance(libpath,str) and libpath:
        resolved=(site_path.parent/libpath).resolve()
        try:binary_hash_valid=file_sha(resolved)==libhash
        except OSError:issue('site.native_library.path','library file unavailable')
        else:
            if not binary_hash_valid:issue('site.native_library.sha256','library hash mismatch')
    return {'parameters_valid':not issues,'ready_for_run':not issues and not inhibits,'issues':issues,'inhibits':inhibits,
            'site_sha256':file_sha(site_path),'session_sha256':file_sha(session_path),'binary_hash_valid':binary_hash_valid,
            'native_library_loaded':False,'serial_devices_opened':False,'device_identity_checked':False,
            'physical_execution_verified':False},site,session,site_path,session_path
