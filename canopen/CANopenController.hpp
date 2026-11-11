/*
CANopen Motor Controller Abstract Class
This class provides an interface for controlling CANopen motor drivers.
It includes methods for managing the motor state, sending and receiving CAN messages,
*/

#pragma once
#include "CANopenDefines.h"
#include <ICANController.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define FULL_CIRCLE_RADIAN (2 * M_PI)

#define CANopenCONTROLLER_SDO_WRITE_RETRY_COUNT 10
#define CANopenCONTROLLER_SDO_READ_RETRY_COUNT 10

using namespace ICANCONTROLLER;

namespace CANOPENCONTROLLER {
union PDO_MAPPING {
    struct {
        OBJECT_INDEX object_index;
        SUB_INDEX sub_index;
        uint8_t length;
    };
    uint32_t value;
};

typedef uint8_t PDO[MAX_PDO_SIZE];

typedef struct __attribute__((packed)) {
    SDO_COMMAND ccs_value;
    OBJECT_INDEX object_index;
    SUB_INDEX sub_index;
    uint32_t message_data;
} SDOMessage;

typedef struct __attribute__((packed)) {
    NMT_COMMAND ccs_value;
    uint8_t node_id;
} NMTMessage;

typedef struct {
    uint8_t node_id;
    uint32_t encoder_resolution;
    float gear_reduction_ratio;
    float screw_pitch_mm;
    int32_t position_offset;
    float max_position_limit_degree;
    float min_position_limit_degree;
    float max_following_error_degree;
    uint8_t max_output_current_ma;
    uint8_t max_continuous_current_ma;
    uint8_t max_velocity_rpm;
    uint8_t max_profile_velocity_rpm;
    uint8_t max_acceleration_rpm_per_sec;
    uint8_t profile_acceleration_rpm_per_sec;
    uint8_t profile_deceleration_rpm_per_sec;
    uint8_t quick_stop_deceleration_rpm_per_sec;
} MotorConfig_t;

class CANopenController {
  public:
    MotorConfig_t config;
    ~CANopenController() = default;

    // Network Management
    err_t SYNC_Send() { return CanController.write(SYNC_COB_ID, 0, 0); };
    virtual err_t resetCommunication() { return NMT_Send(NMT_COMMAND::RESET_COMM); }
    virtual err_t startRemoteNode() { return NMT_Send(NMT_COMMAND::START); }
    virtual err_t resetRemoteNode() { return NMT_Send(NMT_COMMAND::RESET); }
    virtual err_t stopRemoteNode() { return NMT_Send(NMT_COMMAND::STOPPED); }

    // State Machine
    virtual err_t clearFault()
    { return SDO_Write(OBJECT_INDEX::CONTROL_WORD, SUB_INDEX::ZERO, (uint16_t)CONTROLWORD_MASK::FAULT_RESET); };
    virtual err_t shutDown()
    {
        err_t err = SDO_Write(OBJECT_INDEX::CONTROL_WORD, SUB_INDEX::ZERO, (uint16_t)CONTROLWORD_COMMAND::SHUTDOWN);

        if (err != OK) return err;

        uint32_t statusword = 0;

        const auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

        do {
            err = SDO_Read(OBJECT_INDEX::STATUS_WORD, SUB_INDEX::ZERO, &statusword);
            if (err != OK) return err;

            if ((statusword & 0x006F) == 0x0021) return OK;

        } while (std::chrono::steady_clock::now() < timeout);

        return TIMEOUT;
    }
    virtual err_t switchOn()
    {
        err_t err = SDO_Write(OBJECT_INDEX::CONTROL_WORD, SUB_INDEX::ZERO, (uint16_t)CONTROLWORD_COMMAND::SWITCH_ON);

        if (err != OK) return err;

        uint32_t statusword;

        const auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

        do {
            err = SDO_Read(OBJECT_INDEX::STATUS_WORD, SUB_INDEX::ZERO, &statusword);

            if (err != OK) return err;

            if ((statusword & 0x006F) == (uint32_t)STATUSWORD_STATE::SWITCHED_ON) return OK;

        } while (std::chrono::steady_clock::now() < timeout);

        return TIMEOUT;
    }
    virtual err_t enableOperation()
    {
        err_t err =
            SDO_Write(OBJECT_INDEX::CONTROL_WORD, SUB_INDEX::ZERO, (uint16_t)CONTROLWORD_COMMAND::ENABLE_OPERATION);

        if (err != OK) return err;

        uint32_t statusword;

        const auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

        do {
            err = SDO_Read(OBJECT_INDEX::STATUS_WORD, SUB_INDEX::ZERO, &statusword);

            if (err != OK) return err;
            if ((statusword & 0x006F) == (uint32_t)STATUSWORD_STATE::OPERATION_ENABLED) return OK;

        } while (std::chrono::steady_clock::now() < timeout);

        return TIMEOUT;
    }
    virtual err_t activateMode(MODES_OF_OPERATION mode)
    { return SDO_Write(OBJECT_INDEX::MODES_OF_OPERATION, SUB_INDEX::ZERO, (uint8_t)mode); };
    virtual err_t setProfileVelocity(uint32_t velocity_rpm)
    { return SDO_Write(OBJECT_INDEX::PROFILE_VELOCITY, SUB_INDEX::ZERO, velocity_rpm); };
    virtual err_t setHomeOffset(int32_t home_pos)
    { return SDO_Write(OBJECT_INDEX::HOME_OFFSET, SUB_INDEX::ZERO, home_pos); }
    virtual err_t setProfileAcceleration(uint32_t acceleration_rpm)
    { return SDO_Write(OBJECT_INDEX::PROFILE_ACCELERATION, SUB_INDEX::ZERO, acceleration_rpm); };
    virtual err_t setProfileDeceleration(uint32_t deceleration_rpm)
    { return SDO_Write(OBJECT_INDEX::PROFILE_DECELERATION, SUB_INDEX::ZERO, deceleration_rpm); };
    virtual err_t setQuickStopDeceleration(uint32_t value)
    { return SDO_Write(OBJECT_INDEX::QUICK_STOP_DECELERATION, SUB_INDEX::ZERO, value); };
    virtual err_t setSyncMessageStatus(uint32_t value)
    { return SDO_Write(OBJECT_INDEX::SYNC_COB_ID, SUB_INDEX::ZERO, value); };
    virtual err_t setCommunicationCyclePeriod(uint32_t value)
    { return SDO_Write(OBJECT_INDEX::COMMUNICATION_CYCLE_PERIOD, SUB_INDEX::ZERO, value); };
    virtual err_t setInterpolationTimeUnits(uint32_t value)
    { return SDO_Write(OBJECT_INDEX::INTERPOLATION_TIME_PERIOD, SUB_INDEX::ONE, value); };
    virtual err_t setInterpolationTimeIndex(uint32_t value)
    { return SDO_Write(OBJECT_INDEX::INTERPOLATION_TIME_PERIOD, SUB_INDEX::TWO, value); };
    virtual err_t getActualPosition(int32_t* data)
    { return SDO_Read(OBJECT_INDEX::POSITION_ACTUAL_VALUE, SUB_INDEX::ZERO, (uint32_t*)data); };
    virtual err_t getErrorCode(uint16_t* data)
    { return SDO_Read(OBJECT_INDEX::ERROR_CODE, SUB_INDEX::ZERO, (uint32_t*)data); };

    // UTILITY FUNCTIONS
    static int32_t convertFromRadToMotorPulse(float rad, uint32_t encoder_resolution, float gear_reduction_ratio)
    { return (int32_t)(((rad * encoder_resolution) / (FULL_CIRCLE_RADIAN)) / gear_reduction_ratio); }
    static float convertFromMotorPulseToRad(int32_t motor_pulse, uint32_t encoder_resolution,
                                            float gear_reduction_ratio)
    {
        float resultValue = (((float)motor_pulse * FULL_CIRCLE_RADIAN) / encoder_resolution) * gear_reduction_ratio;
        return resultValue;
    };
    static float convertFromMotorPulseToMm(int32_t motor_pulse, uint32_t encoder_resolution, float gear_reduction_ratio,
                                           float screw_pitch_mm)
    {
        float resultValue = (((float)motor_pulse * FULL_CIRCLE_RADIAN) / encoder_resolution) * gear_reduction_ratio *
                            (screw_pitch_mm / FULL_CIRCLE_RADIAN);

        return resultValue;
    };
    static int32_t convertFromMmToMotorPulse(float position_mm, uint32_t encoder_resolution, float gear_reduction_ratio,
                                             float screw_pitch_mm)
    {
        float motor_pulse = (position_mm / screw_pitch_mm) * encoder_resolution / gear_reduction_ratio;

        return static_cast<int32_t>(motor_pulse);
    };
    static int32_t convertFromDegreeToMotorPulse(float degree, uint32_t encoder_resolution, float gear_reduction_ratio)
    {
        float resultValue = ((((float)encoder_resolution / 360) * gear_reduction_ratio) * degree);
        return (int32_t)resultValue;
    };
    static uint32_t convertFromRPMToMotorPulse(uint32_t rpm, uint32_t encoder_resolution, float gear_reduction_ratio)
    { return roundf(((((float)encoder_resolution / 60) * gear_reduction_ratio) * rpm)); };

    static float convertFromMotorPulseToDegree(int32_t motor_pulse, uint32_t encoder_resolution,
                                               float gear_reduction_ratio)
    { return (((float)motor_pulse * 360) / encoder_resolution) * gear_reduction_ratio; };

    err_t setTPDO1(const uint8_t* data, uint8_t dlc)
    {
        __builtin_memcpy((uint8_t*)data, TPDO1, MAX_PDO_SIZE);
        return OK;
    }
    err_t setTPDO2(const uint8_t* data, uint8_t dlc)
    {
        __builtin_memcpy((uint8_t*)data, TPDO2, MAX_PDO_SIZE);
        return OK;
    }

  protected:
    PDO RPDO1;
    PDO RPDO2;
    PDO TPDO1;
    PDO TPDO2;

    CANopenController(ICANController& CanController, MotorConfig_t config)
        : config(config), CanController(CanController),
          sdo_transmit_message_offset(SDO_TX_COB_ID_OFFSET + config.node_id),
          sdo_receive_message_offset(SDO_RX_COB_ID_OFFSET + config.node_id) {};

    ICANController& CanController;

    uint16_t sdo_receive_message_offset;
    uint16_t sdo_transmit_message_offset;

    err_t NMT_Send(NMT_COMMAND cmd)
    {
        NMTMessage msg = {cmd, config.node_id};
        return CanController.write(0, (uint8_t*)&msg, NMT_MESSAGE_SIZE);
    };

    template <typename T>
    err_t SDO_Write(OBJECT_INDEX index, SUB_INDEX sub_index, T data,
                    uint8_t retry_count = CANopenCONTROLLER_SDO_WRITE_RETRY_COUNT, bool confirmation_required = true)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || std::is_same_v<T, uint32_t> ||
                          std::is_same_v<T, int8_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, int32_t>,
                      "Unsupported SDO data type");

        constexpr SDO_COMMAND command = sizeof(T) == 1   ? SDO_COMMAND::WRITE_1_BYTE
                                        : sizeof(T) == 2 ? SDO_COMMAND::WRITE_2_BYTE
                                                         : SDO_COMMAND::WRITE_4_BYTE;

        return __SDO_Write(index, sub_index, data, command, retry_count, confirmation_required);
    };

    err_t __SDO_Write(OBJECT_INDEX index, SUB_INDEX sub_index, uint32_t data, SDO_COMMAND write_request_size_type,
                      uint8_t retry_count = CANopenCONTROLLER_SDO_WRITE_RETRY_COUNT, bool confirmation_required = true)
    {
        SDOMessage SDO_TX = {write_request_size_type, index, sub_index, data};
        SDOMessage SDO_RX;
        err_t err;

        for (size_t i = 0; i < retry_count; i++) {
            err = CanController.write(sdo_transmit_message_offset, (uint8_t*)&SDO_TX, STANDARD_SDO_MESSAGE_SIZE);

            if (err < 0) continue;
            if (!confirmation_required) return err;

            err = CanController.read(sdo_receive_message_offset, (uint8_t*)&SDO_RX, 100);

            if (err == TIMEOUT) continue;
            if (SDO_RX.ccs_value == SDO_COMMAND::WRITE_ACK && SDO_RX.object_index == index &&
                SDO_RX.sub_index == sub_index)
                return OK;
            else if (SDO_RX.ccs_value == SDO_COMMAND::ABORT)
                return WRITE_ERROR;
        }
        return err;
    };

    err_t SDO_Read(OBJECT_INDEX index, SUB_INDEX sub_index, uint32_t* data,
                   uint8_t retry_count = CANopenCONTROLLER_SDO_READ_RETRY_COUNT)
    {
        SDOMessage SDO_TX = {SDO_COMMAND::READ, index, sub_index, 0};
        SDOMessage SDO_RX{};
        err_t err;

        for (size_t i = 0; i < retry_count; i++) {
            err = CanController.write(sdo_transmit_message_offset, (uint8_t*)&SDO_TX, STANDARD_SDO_MESSAGE_SIZE);
            if (err < 0) continue;

            err = CanController.read(sdo_receive_message_offset, (uint8_t*)&SDO_RX, 10);

            if (err < 0) continue;

            if (SDO_RX.ccs_value == SDO_COMMAND::ABORT) {
                err = READ_ERROR;
                continue;
            }
            else if (SDO_RX.object_index == index && SDO_RX.sub_index == sub_index) {
                *data = SDO_RX.message_data;
                return OK;
            }
            else {
                err = READ_ERROR;
                continue;
            }
        }
        return err;
    };
};
} // namespace CANOPENCONTROLLER
