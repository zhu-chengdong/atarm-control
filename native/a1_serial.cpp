#include "a1_serial.h"
#include "unitreeMotor/unitreeMotor.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
uint32_t read32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24);
}
// Same MSB-first word algorithm as the pinned official SDK crc/crc32.h.
uint32_t crc_words(const uint8_t* bytes, size_t words) {
    uint32_t crc = 0xffffffffU;
    for (size_t i=0; i<words; ++i) {
        uint32_t word = read32(bytes+4*i);
        for (uint32_t mask=0x80000000U; mask; mask>>=1) {
            bool top = (crc & 0x80000000U) != 0;
            crc <<= 1;
            if (top) crc ^= 0x04c11db7U;
            if (word & mask) crc ^= 0x04c11db7U;
        }
    }
    return crc;
}
struct Port {
    int fd;
    uint64_t timeout_ns, sequence=0;
    bool quarantined=false;
    std::mutex mutex;
    Port(int source, uint64_t timeout):fd(-1),timeout_ns(timeout) {
        char name[256]{};
        if (ttyname_r(source,name,sizeof(name)) != 0) throw 1;
#ifndef ATARM_ENABLE_PHYSICAL_UART
        // Current offline build only accepts an existing pseudo-terminal slave.
        if (std::string(name).rfind("/dev/pts/",0) != 0) throw 1;
#endif
        fd = dup(source);
        if (fd < 0) throw 1;
        termios options{};
        if (tcgetattr(fd,&options) != 0) { close(fd); fd=-1; throw 1; }
#ifndef ATARM_ENABLE_PHYSICAL_UART
        cfmakeraw(&options);
        options.c_cc[VMIN]=0; options.c_cc[VTIME]=0;
        if (tcsetattr(fd,TCSANOW,&options) != 0) { close(fd); fd=-1; throw 1; }
#else
        // A commissioned caller owns UART configuration, including BOTHER
        // custom baud. POSIX tcsetattr here could overwrite termios2 speeds.
        if ((options.c_lflag&(ICANON|ECHO|ISIG)) || (options.c_oflag&OPOST) ||
            (options.c_iflag&(IXON|IXOFF|IXANY)) || (options.c_cflag&CSIZE)!=CS8 ||
            (options.c_cflag&(PARENB|CSTOPB|CRTSCTS))) { close(fd); fd=-1; throw 1; }
#endif
        if (fcntl(fd,F_SETFL,fcntl(fd,F_GETFL)|O_NONBLOCK) != 0) {
            close(fd); fd=-1; throw 1;
        }
    }
    ~Port() { if (fd>=0) close(fd); }
    int fail(A1Feedback& out, int status) { quarantined=true; out.status=status; return status; }
    int ready(short events, uint64_t deadline) {
        while (true) {
            auto current=now_ns();
            if (current>=deadline) return 0;
            auto remain=deadline-current;
            int ms=int(std::max<uint64_t>(1,(remain+999999)/1000000));
            pollfd item{fd,events,0};
            int result=poll(&item,1,ms);
            if (result<0 && errno==EINTR) continue;
            if (result<0 || (item.revents&(POLLERR|POLLHUP|POLLNVAL))) return -1;
            if (result>0 && (item.revents&events)) return 1;
        }
    }
    int exchange(const A1Command& command,A1Feedback& out) {
        std::lock_guard<std::mutex> lock(mutex);
        out={};
        out.q=out.dq=out.tau=std::numeric_limits<double>::quiet_NaN();
        out.motor_id=255; out.mode=255;
        if (quarantined) { out.status=A1_QUARANTINED; return out.status; }
        if (command.motor_id==0xbb || command.mode!=a1_foc_mode() ||
            !std::isfinite(command.q) || !std::isfinite(command.dq) || !std::isfinite(command.tau) ||
            !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
            std::abs(command.q)>800000 || std::abs(command.dq)>255 ||
            std::abs(command.tau)>127 || command.kp<0 || command.kp>15.9 ||
            command.kd<0 || command.kd>31.9) {
            out.status=A1_INVALID_COMMAND; return out.status;
        }
        // This transaction owns no prior receive buffer. Drain bytes queued before TX.
        uint8_t bytes[512];
        uint64_t drain_deadline=now_ns()+timeout_ns;
        while (true) {
            if (now_ns()>=drain_deadline) return fail(out,A1_TIMEOUT);
            ssize_t count=read(fd,bytes,sizeof(bytes));
            if (count>0) {
                out.discarded_bytes+=uint32_t(count);
                if (out.discarded_bytes>8192) return fail(out,A1_IO_ERROR);
            } else if (count==0 || errno==EAGAIN || errno==EWOULDBLOCK) break;
            else if (errno!=EINTR) return fail(out,A1_IO_ERROR);
        }
        MotorCmd sdk_command;
        sdk_command.motorType=MotorType::A1;
        sdk_command.hex_len=34; sdk_command.Res.u32=0;
        std::memset(sdk_command.get_motor_send_data(),0,34);
        sdk_command.id=command.motor_id; sdk_command.mode=command.mode;
        sdk_command.q=float(command.q); sdk_command.dq=float(command.dq);
        sdk_command.tau=float(command.tau); sdk_command.kp=float(command.kp); sdk_command.kd=float(command.kd);
        sdk_command.modify_data(&sdk_command);
        if (sdk_command.hex_len!=34) return fail(out,A1_INVALID_DATA);
        auto* request=sdk_command.get_motor_send_data();
        out.tx_ns=now_ns();
        uint64_t deadline=out.tx_ns+timeout_ns;
        size_t sent=0;
        while (sent<34) {
            int available=ready(POLLOUT,deadline);
            if (available<=0) return fail(out,available==0?A1_TIMEOUT:A1_IO_ERROR);
            ssize_t count=write(fd,request+sent,34-sent);
            if (count>0) sent+=size_t(count);
            else if (count<0 && errno!=EINTR && errno!=EAGAIN && errno!=EWOULDBLOCK) return fail(out,A1_IO_ERROR);
        }
        std::vector<uint8_t> buffer;
        while (now_ns()<deadline) {
            int available=ready(POLLIN,deadline);
            if (available<=0) return fail(out,available==0?A1_TIMEOUT:A1_IO_ERROR);
            ssize_t count=read(fd,bytes,sizeof(bytes));
            if (count<0) {
                if (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
                return fail(out,A1_IO_ERROR);
            }
            if (count==0) continue;
            buffer.insert(buffer.end(),bytes,bytes+count);
            while (buffer.size()>=2 && (buffer[0]!=0xfe || buffer[1]!=0xee)) {
                buffer.erase(buffer.begin()); ++out.discarded_bytes;
            }
            if (buffer.size()<78) continue;
            out.actual_rx_length=78;
            // Official A1 decoder checks 18 words; trailing CRC is at byte 74.
            // This exact coverage is cross-checked with the offline SDK probe.
            if (crc_words(buffer.data(),18)!=read32(buffer.data()+74)) return fail(out,A1_BAD_CRC);
            if (buffer[2]!=command.motor_id) return fail(out,A1_WRONG_ID);
            if (buffer[4]!=command.mode) return fail(out,A1_WRONG_MODE);
            MotorData data;
            data.motorType=MotorType::A1; data.hex_len=78; data.correct=false;
            std::memcpy(data.get_motor_recv_data(),buffer.data(),78);
            if (!data.extract_data(&data) || !data.correct) return fail(out,A1_INVALID_DATA);
            if (data.motor_id!=command.motor_id || data.mode!=command.mode) return fail(out,A1_INVALID_DATA);
            if (data.merror!=0) return fail(out,A1_MOTOR_ERROR);
            if (!std::isfinite(data.q) || !std::isfinite(data.dq) || !std::isfinite(data.tau)) return fail(out,A1_INVALID_DATA);
            auto received=now_ns();
            if (received>=deadline) return fail(out,A1_TIMEOUT);
            out.frame_id=++sequence; out.rx_ns=received; out.status=A1_OK;
            out.motor_id=data.motor_id; out.mode=data.mode;
            out.temp=data.temp; out.merror=data.merror;
            out.q=data.q; out.dq=data.dq; out.tau=data.tau;
            out.discarded_bytes+=uint32_t(buffer.size()-78);
            return A1_OK;
        }
        return fail(out,A1_TIMEOUT);
    }
};
}

extern "C" {
void* a1_create_from_fd(int fd,uint64_t timeout_ns) {
    if (timeout_ns==0 || timeout_ns>5000000000ULL) return nullptr;
    try { return new Port(fd,timeout_ns); } catch (...) { return nullptr; }
}
void a1_destroy(void* handle) { delete static_cast<Port*>(handle); }
int a1_exchange(void* handle,const A1Command* command,A1Feedback* result) {
    if (!result || !command) return A1_INVALID_COMMAND;
    if (!handle) { *result={}; result->status=A1_CLOSED; return A1_CLOSED; }
    try { return static_cast<Port*>(handle)->exchange(*command,*result); }
    catch (...) { *result={}; result->status=A1_IO_ERROR; return A1_IO_ERROR; }
}
const char* a1_status_name(int status) {
    static const char* names[]={"OK","TIMEOUT","BAD_CRC","WRONG_ID","WRONG_MODE","MOTOR_ERROR",
        "INVALID_DATA","IO_ERROR","CLOSED","QUARANTINED","INVALID_COMMAND"};
    return status>=0 && status<=10 ? names[status] : "UNKNOWN_ERROR";
}
double a1_gear_ratio() { return queryGearRatio(MotorType::A1); }
int a1_foc_mode() { return queryMotorMode(MotorType::A1,MotorMode::FOC); }
uint32_t a1_command_size() { return sizeof(A1Command); }
uint32_t a1_feedback_size() { return sizeof(A1Feedback); }
int a1_physical_uart_enabled() {
#ifdef ATARM_ENABLE_PHYSICAL_UART
    return 1;
#else
    return 0;
#endif
}
}
