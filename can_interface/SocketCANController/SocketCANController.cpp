// SocketCANController.cpp

#include <SocketCANController.hpp>
#include <array>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <thread>

using namespace ICANCONTROLLER;
using namespace SOCKETCANCONTROLLER;

SocketCANController::SocketCANController(const std::string& interface_name, const std::vector<uint16_t>& filter_ids)
    : is_running(false)
{

    struct ifreq ifr{};
    struct sockaddr_can addr{};

    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        perror("Socket");
        throw std::runtime_error("CAN socket creation failed");
    }

    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        throw std::runtime_error("CAN interface not found");
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        throw std::runtime_error("CAN bind failed");
    }

    std::vector<struct can_filter> filters;
    for (auto id : filter_ids) {
        struct can_filter f;
        f.can_id   = id;
        f.can_mask = CAN_EFF_MASK;
        filters.push_back(f);
    }
    setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(struct can_filter));

    // Non-blocking mod etkinleştirme
    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl get error");
        return;
    }
    if (fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) == -1) { perror("fcntl set error"); }

    for (auto id : filter_ids) {
        auto filter = std::make_unique<CANFilterData_t>();

        filter->event_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (filter->event_fd < 0) {
            perror("eventfd");
            for (auto& kv : filter_data) {
                if (kv.second->event_fd >= 0) close(kv.second->event_fd);
            }
            throw std::runtime_error("Init error!");
        }
        filter_data.emplace(id, std::move(filter));
    }
}

void SocketCANController::startReadThread(uint16_t pref_core)
{
    if (read_thread.joinable()) return;
    is_running.store(true, std::memory_order_release);
    read_thread = std::thread(&SocketCANController::readThreadFunction, this);

    pthread_t native = read_thread.native_handle();
    // --- CPU affinity ---
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(pref_core, &cpuset);

    if (pthread_setaffinity_np(native, sizeof(cpu_set_t), &cpuset) != 0) { perror("pthread_setaffinity_np failed"); }
}

void SocketCANController::stopReadThread()
{
    is_running.store(false, std::memory_order_release);
    if (socket_fd >= 0) { ::shutdown(socket_fd, SHUT_RD); }

    if (read_thread.joinable()) {
        read_thread.join();
        fprintf(stderr, "SocketCAN read thread exited.\r\n");
    }
}

err_t SocketCANController::readThreadFunction()
{
    struct can_frame frame;
    ssize_t nbytes;
    uint64_t one = 1;
    while (is_running) {
        nbytes = ::read(socket_fd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) continue;
        if (nbytes < sizeof(struct can_frame)) perror("Incomplete CAN frame received");

        auto it = filter_data.find(frame.can_id);
        if (it == filter_data.end()) continue; // Unknown ID, skip processing

        {
            std::lock_guard<std::mutex> lock(it->second->mutex);
            __builtin_memcpy(it->second->data, frame.data, frame.can_dlc);
            if (it->second->callback) it->second->callback(it->second->data, frame.can_dlc);
        }

        ssize_t ret = ::write(it->second->event_fd, &one, sizeof(one));
        if (ret < 0) {
            if (errno != EAGAIN) perror("eventfd write");
        }
    }
    return OK; // Thread stopped gracefully
}

err_t SocketCANController::addCallback(uint32_t can_id, CANFilterCallback callback)
{
    auto it = filter_data.find(can_id);

    if (it == filter_data.end()) return ERROR;

    {
        std::lock_guard<std::mutex> lock(it->second->mutex);
        it->second->callback = std::move(callback);
    }

    return OK;
}

err_t SocketCANController::write(uint32_t can_id, uint8_t* tx_buffer, uint8_t tx_buffer_size, bool is_extended)
{

    tx_frame.can_id = can_id;
    if (is_extended) {
        tx_frame.can_id |= CAN_EFF_FLAG; // Set extended frame flag if needed
    }
    tx_frame.len = tx_buffer_size;
    // std::memcpy(tx_frame.data, tx_buffer, tx_buffer_size);
    __builtin_memcpy(tx_frame.data, tx_buffer, tx_frame.len);
    if (::write(socket_fd, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
        perror("Socket write error");
        return WRITE_ERROR;
    }
    return OK;
}

err_t SocketCANController::read(uint32_t id, uint8_t* rx_buffer, uint16_t timeout_ms)
{
    auto it = filter_data.find(id);
    if (it == filter_data.end()) {
        is_running ? std::cerr << "Error: No filter data for ID :" << id << std::endl
                   : std::cerr << "Error: No filter data for ID :" << id << ", controller not running" << std::endl;
        return READ_ERROR;
    }

    int efd = it->second->event_fd;

    struct pollfd pfd{};
    pfd.fd     = efd;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret == 0) return TIMEOUT;
    if (ret < 0) return READ_ERROR;
    if (!(pfd.revents & POLLIN)) return READ_ERROR;

    uint64_t counter;
    if (::read(efd, &counter, sizeof(counter)) < 0) return READ_ERROR;

    {
        std::lock_guard<std::mutex> lock(it->second->mutex);
        __builtin_memcpy(rx_buffer, it->second->data, CAN_MAX_DLEN);
    }
    return OK;
}

SocketCANController::~SocketCANController()
{
    stopReadThread();

    for (auto& kv : filter_data) {
        if (kv.second->event_fd >= 0) close(kv.second->event_fd);
    }

    if (socket_fd >= 0) { close(socket_fd); }
    fprintf(stderr, "SocketCAN deleted.\r\n");
}
