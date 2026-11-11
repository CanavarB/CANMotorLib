#pragma once
#include <can_interface/ICANController.hpp>

#include <cstdint>
#include <endian.h>
#include <memory>

#define CAN_EFF_FLAG 0x80000000U

#define ENCODER_RESOLUTION 16384

using namespace ICANCONTROLLER;

namespace AK70_Driver {

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
    CAN_PACKET_UPLOAD_MESSAGE = 0x29,
} CAN_PACKET_ID;

typedef union AK70_Upload_Message {
    struct {
        int16_t position;
        int16_t velocity;
        int16_t current;
        int8_t temperature;
        std::uint8_t error_code;
    } __attribute__((packed));
    uint8_t raw[sizeof(int16_t) + sizeof(int16_t) + sizeof(int16_t) + sizeof(int8_t) + sizeof(std::uint8_t)];
} AK70_Upload_Message;

typedef struct {
    uint32_t duty_cycle;
    uint32_t current_loop;
    uint32_t current_break;
    uint32_t speed_loop;
    uint32_t position_loop;
    uint32_t set_origin;
    uint32_t position_speed_loop;
    uint32_t upload_message;

} AK70_Mode_IDs;

typedef union AK70_Position_Speed_Loop_Message {
    struct {
        int32_t position;
        int16_t speed;
        int16_t acceleration;
    } __attribute__((packed));
    uint8_t raw[sizeof(int32_t) + sizeof(int16_t) + sizeof(int16_t)];
} AK70_Position_Speed_Loop_Message_raw;

typedef union AK70_Speed_Loop_Message {
    int32_t speed;
    uint8_t raw[sizeof(int32_t)];
} AK70_Speed_Loop_Message_raw;

class AK70_Driver {
  public:
    AK70_Driver(ICANController* CanController, std::uint8_t motor_id);

    err_t read_upload();
    /**
     * @brief Get the position of the motor in degrees
     * @return double The position of the motor in degrees
     * @note The position is in -3200 to 3200
     */
    double get_position();

    /**
     * @brief Get the velocity of the motor in ERPM
     * @return double The velocity of the motor in ERPM
     * @note The velocity is in -320000 to 320000
     */
    double get_velocity();

    /**
     * @brief Get the current of the motor in Ampere
     * @return double The current of the motor in Ampere
     * @note The current is in -60A to 60A
     */
    double get_current();
    /**
     * @brief Get the temperature of the motor in Celsius
     * @return int8_t The temperature of the motor in Celsius
     * @note The temperature is in -20C to 127C
     */
    int8_t get_temperature() { return upload_msg.temperature; }

    /**
     * @brief Get the error code of the motor
     * @return uint8_t The error code of the motor
     * @note 0 indicates no fault, 1 indicates motor over-temperature fault, 2 indicates over-current fault, 3
     * indicates over-voltage fault, 4 indicates under-voltage fault, 5 indicates encoder fault, 6 indicates
     * MOSFET over-temperature fault, 7 indicates motor stall.      */
    std::uint8_t get_error_code() { return upload_msg.error_code; }

    err_t position_speed_loop(int32_t position, int16_t speed, int16_t acceleration);

    err_t speed_loop(int32_t speed);

  private:
    AK70_Upload_Message upload_msg;
    AK70_Mode_IDs mode_ids;
    ICANController* CanController;
    std::uint8_t motor_id;
};
} // namespace AK70_Driver
