"""Independent wire emulator and external pose source, outside controller PID."""
import errno,math,os,select,struct,threading,time


def crc(data,words):
    value=0xffffffff
    for start in range(0,words*4,4):
        word=int.from_bytes(data[start:start+4],'little')
        for bit in range(31,-1,-1):
            top=((value>>31)^((word>>bit)&1))&1;value=(value<<1)&0xffffffff
            if top:value^=0x04c11db7
    return value


def feedback(mid,raw,error=0):
    b=bytearray(78);b[:4]=bytes((254,238,mid,0));b[4]=10;b[6]=25;b[7]=error
    struct.pack_into('<i',b,30,raw);struct.pack_into('<I',b,74,crc(b,18));return b


class WireBoard:
    def __init__(self,bus,positions):
        self.bus=bus;self.positions=positions;self.master,self.slave=os.openpty();self.mode='normal'
        self.raw={0:0,1:0,2:0};self.logs=[];self.errors=[];self.done=threading.Event()
        self.thread=threading.Thread(target=self.run,name='test-wire-'+bus,daemon=True);self.thread.start()

    def run(self):
        buf=b''
        while not self.done.is_set():
            try:
                if not select.select([self.master],[],[],.02)[0]:continue
                buf+=os.read(self.master,4096)
                while len(buf)>=34:
                    request,buf=buf[:34],buf[34:];assert request[:2]==b'\xfe\xee'
                    assert struct.unpack_from('<I',request,30)[0]==crc(request,7)
                    mid=request[2];target=struct.unpack_from('<i',request,16)[0]
                    kp=struct.unpack_from('<h',request,20)[0];kd=struct.unpack_from('<h',request,22)[0]
                    if kp and self.mode not in ('stall','false_encoder'):self.raw[mid]=target
                    index=3 if self.bus=='deploy' else {1:0,0:1,2:2}[mid]
                    sign=-1 if index in (2,3) else 1
                    q=sign*self.raw[mid]*(6.2832/16384)/9.1*.026+(.04 if index==3 else 0.)
                    self.positions[index]=q
                    measured=target if self.mode=='false_encoder' and kp else self.raw[mid]
                    reply=feedback((mid+1)%3 if self.mode=='wrong_id' else mid,measured,7 if self.mode=='motor_error' else 0)
                    if self.mode=='bad_crc':reply[74]^=1
                    self.logs.append({'s':time.monotonic(),'bus':self.bus,'motor_id':mid,'kp_raw':kp,'kd_raw':kd,
                        'target_raw':target,'physical_raw':self.raw[mid],'mode':self.mode,'request_hex':request.hex(),
                        'response_hex':None if self.mode=='timeout' else reply.hex()})
                    if self.mode!='timeout':os.write(self.master,reply)
            except OSError as e:
                if not self.done.is_set() and e.errno not in (errno.EIO,errno.EBADF):self.errors.append(str(e))
                time.sleep(.002)
            except Exception as e:self.errors.append(repr(e));return

    def close(self):
        self.done.set();self.thread.join(2.);os.close(self.master);os.close(self.slave)
        assert not self.thread.is_alive() and not self.errors,self.errors


def true_tcp(q):
    q0,q1,q2,q3=q;a=(2*q0-q1-q2)/3;b=(q1-q2)/math.sqrt(3);s=q3-.04
    return (.3+.85*a+.18*b+.22*s+3*a*s+1.2*b*b,
            .05-.12*a+1.05*b-.16*s+2*b*s-1.5*a*a,
            .18+.12*a-.08*b+.9*s+2.8*a*a+2*b*b-1.5*s*a)


def pose_process(positions,mode,stop,namespace):
    import json,rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    rclpy.init();node=Node('external_vision_test_source',namespace=namespace)
    pp=node.create_publisher(PoseStamped,'atarm/camera/pose',8);sp=node.create_publisher(String,'atarm/camera/source_status',8)
    index=0;last=None
    def tick():
        nonlocal index,last
        code=mode.value
        if code==1 and last:
            pp.publish(last[0]);sp.publish(last[1]);return
        index+=1;stamp=node.get_clock().now().nanoseconds
        if code==5:stamp-=500_000_000
        q=list(positions);p=true_tcp(q);a=(2*q[0]-q[1]-q[2])/3;b=(q[1]-q[2])/math.sqrt(3)
        angle=.8*a-.4*b
        # Independently invert base translation and camera->TCP lever arm.
        psource=(p[0]-.1-.02*math.cos(angle),p[1]+.02,p[2]-.03+.02*math.sin(angle))
        pose=PoseStamped();pose.header.stamp.sec=stamp//10**9;pose.header.stamp.nanosec=stamp%10**9
        pose.header.frame_id='wrong' if code==4 else 'source_map'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=psource
        pose.pose.orientation.y=math.sin(angle/2);pose.pose.orientation.w=math.cos(angle/2)
        meta=String();meta.data=json.dumps({'stamp_ns':stamp,'frame_id':index,'tracking_ok':code!=2,
            'map_epoch':'changed' if code==3 else 'validation_map_0','metric':code!=6,'child_frame':'sensor_camera'})
        pp.publish(pose);sp.publish(meta);last=(pose,meta)
    node.create_timer(.02,tick)
    try:
        while not stop.is_set():rclpy.spin_once(node,timeout_sec=.03)
    finally:
        node.destroy_node();rclpy.shutdown()
