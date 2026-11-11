#pragma once
#include <ICANController.hpp>
#include <atomic>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <poll.h>
#include <queue>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>

using namespace ICANCONTROLLER;

namespace SOCKETCANCONTROLLER {

typedef struct {
    uint8_t data[CAN_MAX_DLEN];
    std::mutex mutex;
    int event_fd{-1};
    CANFilterCallback callback;
} CANFilterData_t;

class SocketCANController : public ICANController {
  public:
    SocketCANController(const std::string& interface_name, const std::vector<uint16_t>& filter_ids);
    ~SocketCANController();

    err_t write(uint32_t can_id, uint8_t* tx_buffer, uint8_t tx_buffer_size, bool is_extended = false) override;
    err_t read(uint32_t id, uint8_t* rx_buffer, uint16_t timeout_ms) override;
    err_t addCallback(uint32_t can_id, ICANCONTROLLER::CANFilterCallback callback) override;
    void startReadThread(uint16_t pref_core);
    void stopReadThread();

  private:
    err_t readThreadFunction();
    std::thread read_thread;
    std::atomic<bool> is_running;
    struct can_frame tx_frame{};
    int socket_fd;
    std::unordered_map<uint32_t, std::unique_ptr<CANFilterData_t>> filter_data;
};

} // namespace SOCKETCANCONTROLLER
