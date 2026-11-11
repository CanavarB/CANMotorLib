#pragma once
#include <cstdint>
#include <functional>

namespace ICANCONTROLLER {

typedef enum {
    OK          = 0,
    ERROR       = -1,
    TIMEOUT     = -2,
    WRITE_ERROR = -10,
    READ_ERROR  = -11,
} err_t;
using CANFilterCallback = std::function<err_t(const uint8_t*, uint8_t)>;

#if DEBUG

#define LOGI(fmt, ...) fprintf(stdout, "[INFO] " fmt "\r\n", ##__VA_ARGS__)

#define LOGE(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\r\n", ##__VA_ARGS__)

#else

#define LOGI(fmt, ...) ((void)0)
#define LOGE(fmt, ...) ((void)0)

#endif

#define RETURN_ON_NOK(expr)                                                                                            \
    do {                                                                                                               \
        err = (expr);                                                                                                  \
        if (err != OK) return err;                                                                                     \
    } while (0)

#define RETURN_ON_NOK_LOG(expr, msg)                                                                                   \
    do {                                                                                                               \
        err = (expr);                                                                                                  \
        if (err != OK) {                                                                                               \
            fprintf(stderr, "%s: ERR:%d\r\n", msg, err);                                                               \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

class ICANController {
  public:
    /**
     * @brief Write to can bus
     * @param can_id The can id of the message
     *  @param tx_buffer The buffer to send
     * @param tx_buffer_size The size of the buffer
     * @param is_extended If true, the can id is extended (29 bits), otherwise standard (11 bits) default is
     * false
     * @return int 0 if success, -1 if fail
     */
    virtual err_t write(uint32_t can_id, uint8_t* tx_buffer, uint8_t tx_buffer_size, bool is_extended = false) = 0;

    /**
     * @param id The can id of the message
     * @param rx_buffer The buffer to store the received data
     * @param timeout_ms The timeout in milliseconds to wait for a message
     * @return err_t 0 if success, -1 if fail, -2 if timeout
     */
    virtual err_t read(uint32_t id, uint8_t* rx_buffer, uint16_t timeout_ms) = 0;

    virtual err_t addCallback(uint32_t can_id, ICANCONTROLLER::CANFilterCallback callback) = 0;
};

} // namespace ICANCONTROLLER
