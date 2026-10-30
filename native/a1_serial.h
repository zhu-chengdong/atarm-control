#pragma once
#include <cstdint>

extern "C" {
struct A1Command {
    uint8_t motor_id, mode;
    double q, dq, tau, kp, kd;
};
struct A1Feedback {
    uint64_t frame_id, tx_ns, rx_ns;
    uint32_t actual_rx_length, discarded_bytes;
    int32_t status;
    uint8_t motor_id, mode;
    int32_t temp, merror;
    double q, dq, tau;
};
enum A1Status { A1_OK=0, A1_TIMEOUT=1, A1_BAD_CRC=2, A1_WRONG_ID=3,
    A1_WRONG_MODE=4, A1_MOTOR_ERROR=5, A1_INVALID_DATA=6,
    A1_IO_ERROR=7, A1_CLOSED=8, A1_QUARANTINED=9, A1_INVALID_COMMAND=10 };
void* a1_create_from_fd(int fd, uint64_t timeout_ns);
void a1_destroy(void* handle);
int a1_exchange(void* handle, const A1Command* command, A1Feedback* result);
const char* a1_status_name(int status);
double a1_gear_ratio();
int a1_foc_mode();
uint32_t a1_command_size();
uint32_t a1_feedback_size();
int a1_physical_uart_enabled();
}
