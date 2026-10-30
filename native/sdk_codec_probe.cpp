// Offline codec-only check. No SerialPort object or device access exists here.
#include "unitreeMotor/unitreeMotor.h"
#include "crc/crc32.h"
#include <array>
#include <cstring>
#include <iomanip>
#include <iostream>

int main() {
    MotorCmd cmd;
    cmd.motorType=MotorType::A1; cmd.hex_len=34; cmd.Res.u32=0;
    std::memset(cmd.get_motor_send_data(),0,34);
    cmd.motorType=MotorType::A1; cmd.id=2;
    cmd.mode=queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.q=1.25f; cmd.dq=0; cmd.tau=0; cmd.kp=0.02f; cmd.kd=3.0f;
    cmd.modify_data(&cmd);
    std::cout<<"gear="<<queryGearRatio(MotorType::A1)<<" mode="<<cmd.mode<<" command_len="<<cmd.hex_len<<"\n";
    std::cout<<"command_hex=";
    for(int i=0;i<cmd.hex_len;++i) std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(cmd.get_motor_send_data()[i]);
    std::cout<<std::dec<<"\n";
    ServoComdDataV3 frame{};
    frame.head.start[0]=0xfe; frame.head.start[1]=0xee; frame.head.motorID=2;
    frame.Mdata.mode=cmd.mode; frame.Mdata.Temp=25; frame.Mdata.Pos=123456;
    uint32_t words[18]{}; std::memcpy(words,&frame,72);
    frame.CRCdata.u32=crc32_core(words,18);
    for(int changed : {-1,30,71,72,73}) {
        MotorData data;
        data.motorType=MotorType::A1; data.hex_len=78;
        std::memcpy(data.get_motor_recv_data(),&frame,78);
        if(changed>=0) data.get_motor_recv_data()[changed]^=1;
        bool result=data.extract_data(&data);
        std::cout<<"changed="<<changed<<" accepted="<<result<<" correct="<<data.correct;
        if(result) std::cout<<" q="<<std::setprecision(12)<<data.q;
        std::cout<<"\n";
    }
}
