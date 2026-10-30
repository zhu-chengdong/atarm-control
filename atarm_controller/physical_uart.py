"""Explicit, commissioned UART provisioning; no device is touched on import."""
from dataclasses import dataclass
from pathlib import Path
import os,platform,stat,struct
from .commissioning_config import file_sha
from .serial_adapter import NativeLibrary,NativeSerialAdapter


@dataclass(frozen=True)
class QualifiedPort:
    path:str
    device:int
    inode:int
    rdev:int


class LinuxUartBackend:
    def inspect(self,port):
        alias=Path(port['by_id_path'])
        if not str(alias).startswith('/dev/serial/by-id/') or len(alias.parts)!=5 or not alias.is_symlink():
            raise ValueError('UART_STABLE_BY_ID_SYMLINK_REQUIRED')
        real=alias.resolve(strict=True)
        if real.parent!=Path('/dev') or not real.name.startswith(('ttyUSB','ttyACM')):
            raise ValueError('UART_PHYSICAL_USB_TTY_REQUIRED')
        info=real.stat()
        if not stat.S_ISCHR(info.st_mode):raise ValueError('UART_CHARACTER_DEVICE_REQUIRED')
        sysdev=(Path('/sys/dev/char')/f'{os.major(info.st_rdev)}:{os.minor(info.st_rdev)}'/'device').resolve(strict=True)
        identity=None
        for ancestor in (sysdev,*sysdev.parents):
            if all((ancestor/n).is_file() for n in ('serial','idVendor','idProduct')):
                identity={n:(ancestor/n).read_text().strip() for n in ('serial','idVendor','idProduct')};break
        if identity is None or identity['serial']!=port['usb_serial'] or identity['idVendor'].lower()!=port['usb_vid'].lower() or identity['idProduct'].lower()!=port['usb_pid'].lower():
            raise ValueError('UART_USB_IDENTITY_MISMATCH')
        return QualifiedPort(str(real),info.st_dev,info.st_ino,info.st_rdev)

    def open(self,qualified):
        fd=os.open(qualified.path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK|os.O_CLOEXEC|os.O_NOFOLLOW)
        info=os.fstat(fd)
        if (info.st_dev,info.st_ino,info.st_rdev)!=(qualified.device,qualified.inode,qualified.rdev):
            os.close(fd);raise ValueError('UART_IDENTITY_CHANGED_DURING_OPEN')
        return fd

    def configure(self,fd,port):
        import fcntl,termios
        if platform.system()!='Linux' or platform.machine()!='x86_64':
            raise ValueError('UART_TERMIOS2_ABI_REQUIRES_LINUX_X86_64')
        if (port['data_bits'],port['parity'],port['stop_bits'],port['flow_control'],port['adapter_direction_control'])!=(8,'N',1,'none','automatic'):
            raise ValueError('UART_UNSUPPORTED_LINE_CONFIGURATION')
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.ioctl(fd,termios.TIOCEXCL)
        attrs=termios.tcgetattr(fd)
        attrs[0]=0;attrs[1]=0;attrs[3]=0
        attrs[2]=(attrs[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|getattr(termios,'CRTSCTS',0)))|termios.CS8|termios.CREAD|termios.CLOCAL
        attrs[4]=attrs[5]=termios.B38400;attrs[6][termios.VMIN]=0;attrs[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,attrs)
        # Linux x86_64 asm-generic termios2: 4 uint flags, line byte, 19 cc
        # bytes, two uint speeds. BOTHER supports non-standard confirmed baud.
        layout='=IIIIB19sII'
        if struct.calcsize(layout)!=44:raise ValueError('UART_TERMIOS2_LAYOUT_MISMATCH')
        tcgets2=0x802c542a;tcsets2=0x402c542b;cbaud=0x100f;bother=0x1000
        raw=bytearray(44);fcntl.ioctl(fd,tcgets2,raw,True);values=list(struct.unpack(layout,raw))
        values[2]=(values[2]&~cbaud)|bother;values[-2]=values[-1]=port['baud_rate']
        fcntl.ioctl(fd,tcsets2,struct.pack(layout,*values))
        fcntl.ioctl(fd,tcgets2,raw,True);readback=struct.unpack(layout,raw)
        if readback[-2:]!=(port['baud_rate'],port['baud_rate']):raise ValueError('UART_BAUD_READBACK_MISMATCH')
        termios.tcflush(fd,termios.TCIOFLUSH)
        return {'baud_rate_readback':readback[-1],'data_bits':8,'parity':'N','stop_bits':1,'flow_control':'none','exclusive_requested':True}

    @staticmethod
    def close(fd):os.close(fd)


class PhysicalUartFactory:
    def __init__(self,ports,library_path,library_sha256,*,backend=None,library_loader=NativeLibrary):
        self.ports=ports;self.library_path=Path(library_path);self.library_sha256=library_sha256
        self.backend=backend or LinuxUartBackend();self.library_loader=library_loader
        self.opened_ports=[];self.configured_ports=[]

    def __call__(self):
        if file_sha(self.library_path)!=self.library_sha256:raise ValueError('UART_NATIVE_HASH_MISMATCH')
        library=self.library_loader(self.library_path)
        if not library.physical_uart_enabled():raise ValueError('UART_NATIVE_PHYSICAL_CAPABILITY_DISABLED')
        if set(self.ports)!= {'active','deploy'}:raise ValueError('UART_TWO_PORTS_REQUIRED')
        # Both identities must pass before either port can be opened.
        qualified={bus:self.backend.inspect(port) for bus,port in self.ports.items()}
        if len({p.rdev for p in qualified.values()})!=2:raise ValueError('UART_PORTS_RESOLVE_TO_SAME_DEVICE')
        transports={}
        try:
            for bus,port in self.ports.items():
                fd=self.backend.open(qualified[bus]);self.opened_ports.append(bus)
                try:
                    if self.backend.inspect(port)!=qualified[bus]:raise ValueError('UART_USB_IDENTITY_CHANGED_AFTER_OPEN')
                    configured=self.backend.configure(fd,port);self.configured_ports.append(dict(bus=bus,**configured))
                    transports[bus]=NativeSerialAdapter(fd,port['reply_timeout_s'],native_library=library)
                finally:self.backend.close(fd)
            return transports
        except Exception:
            for adapter in transports.values():adapter.close()
            raise
