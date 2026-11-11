#pragma once
#include <cstddef>
#include <cstdint>

namespace CANOPENCONTROLLER {

constexpr size_t BITS_PER_BYTE             = 8;
constexpr size_t NMT_MESSAGE_SIZE          = 2;
constexpr size_t STANDARD_SDO_MESSAGE_SIZE = 8;
constexpr size_t MAX_PDO_SIZE              = 8;
constexpr uint32_t SYNC_COB_ID             = 0x080;
constexpr uint16_t EMCY_COB_ID_OFFSET      = 0x0080;
constexpr uint16_t TPDO1_COB_ID_OFFSET     = 0x0180;
constexpr uint16_t TPDO2_COB_ID_OFFSET     = 0x0280;
constexpr uint16_t TPDO3_COB_ID_OFFSET     = 0x0380;
constexpr uint16_t TPDO4_COB_ID_OFFSET     = 0x0480;
constexpr uint16_t RPDO1_COB_ID_OFFSET     = 0x0200;
constexpr uint16_t RPDO2_COB_ID_OFFSET     = 0x0300;
constexpr uint16_t RPDO3_COB_ID_OFFSET     = 0x0400;
constexpr uint16_t RPDO4_COB_ID_OFFSET     = 0x0500;
constexpr uint16_t SDO_TX_COB_ID_OFFSET    = 0x0580;
constexpr uint16_t SDO_RX_COB_ID_OFFSET    = 0x0600;
constexpr uint16_t HEARTBEAT_COB_ID_OFFSET = 0x0700;

enum class SDO_COMMAND : std::uint8_t {
    WRITE_1_BYTE = 0x2F,
    WRITE_2_BYTE = 0x2B,
    WRITE_3_BYTE = 0x27,
    WRITE_4_BYTE = 0x23,
    READ         = 0x40,
    WRITE_ACK    = 0x60,
    READ_1_BYTE  = 0x4F,
    READ_2_BYTE  = 0x4B,
    READ_3_BYTE  = 0x47,
    READ_4_BYTE  = 0x43,
    ABORT        = 0x80,
};

enum class OBJECT_INDEX : std::uint16_t {

    RPDO1_COMMUNICATION_PARAMETER = 0x1400,
    RPDO2_COMMUNICATION_PARAMETER = 0x1401,
    RPDO3_COMMUNICATION_PARAMETER = 0x1402,
    RPDO4_COMMUNICATION_PARAMETER = 0x1403,
    RPDO1_MAPPING                 = 0x1600,
    RPDO2_MAPPING                 = 0x1601,
    RPDO3_MAPPING                 = 0x1602,
    RPDO4_MAPPING                 = 0x1603,
    TPDO1_COMMUNICATION_PARAMETER = 0x1800,
    TPDO2_COMMUNICATION_PARAMETER = 0x1801,
    TPDO3_COMMUNICATION_PARAMETER = 0x1802,
    TPDO4_COMMUNICATION_PARAMETER = 0x1803,
    TPDO1_MAPPING                 = 0x1A00,
    TPDO2_MAPPING                 = 0x1A01,
    TPDO3_MAPPING                 = 0x1A02,
    TPDO4_MAPPING                 = 0x1A03,
    SYNC_COB_ID                   = 0x1005,
    COMMUNICATION_CYCLE_PERIOD    = 0x1006,
    TEMPERATURE                   = 0x22A2,
    ERROR_CODE                    = 0x603F,
    CONTROL_WORD                  = 0x6040,
    STATUS_WORD                   = 0x6041,
    MODES_OF_OPERATION            = 0x6060,
    MODES_OF_OPERATION_DISPLAY    = 0x6061,
    POSITION_ACTUAL_VALUE         = 0x6064,
    VELOCITY_ACTUAL_VALUE         = 0x606C,
    TARGET_POSITION               = 0x607A,
    INTERPOLATION_TIME_PERIOD     = 0x60C2,
    DIGITAL_INPUTS                = 0x60FD,
    TARGET_VELOCITY               = 0x60FF,
    MAX_PROFILE_VELOCITY          = 0x607F,
    HOME_OFFSET                   = 0x607C,
    PROFILE_VELOCITY              = 0x6081,
    PROFILE_ACCELERATION          = 0x6083,
    PROFILE_DECELERATION          = 0x6084,
    QUICK_STOP_DECELERATION       = 0x6085,
    MAX_ACCELERATION              = 0x60C5,
    MAX_MOTOR_SPEED               = 0x6080,
    RELEASE_BRAKE                 = 0x4602,
};

enum class SUB_INDEX : std::uint8_t {
    ZERO  = 0x00,
    ONE   = 0x01,
    TWO   = 0x02,
    THREE = 0x03,
    FOUR  = 0x04,
    FIVE  = 0x05,
    SIX   = 0x06,
    SEVEN = 0x07,
    EIGHT = 0x08,
};

enum class NMT_COMMAND : uint8_t {
    START      = 0x01,
    STOPPED    = 0x02,
    SLEEP      = 0x50,
    STANDBY    = 0x60,
    PRE_OP     = 0x80,
    RESET      = 0x81,
    RESET_COMM = 0x82
};

enum class MODES_OF_OPERATION : std::uint8_t {
    PROFILE_POSITION            = 0x01,
    VELOCITY                    = 0x02,
    PROFILE_VELOCITY            = 0x03,
    PROFILE_TORQUE              = 0x04,
    HOMING                      = 0x06,
    INTERPOLATED_POSITION       = 0x07,
    CYCLIC_SYNCHRONOUS_POSITION = 0x08,
    CYCLIC_SYNCHRONOUS_VELOCITY = 0x09,
    CYCLIC_SYNCHRONOUS_TORQUE   = 0x0A,
};

enum class STATUSWORD_MASK : std::uint16_t {
    READY_TO_SWITCH_ON         = 1u << 0,  // Bit 0
    SWITCHED_ON                = 1u << 1,  // Bit 1
    OPERATION_ENABLED          = 1u << 2,  // Bit 2
    FAULT                      = 1u << 3,  // Bit 3
    VOLTAGE_ENABLED            = 1u << 4,  // Bit 4
    QUICK_STOP                 = 1u << 5,  // Bit 5
    SWITCH_ON_DISABLED         = 1u << 6,  // Bit 6
    WARNING                    = 1u << 7,  // Bit 7
    MANUFACTURER_SPECIFIC_8    = 1u << 8,  // Bit 8
    REMOTE                     = 1u << 9,  // Bit 9
    TARGET_REACHED             = 1u << 10, // Bit 10
    INTERNAL_LIMIT_ACTIVE      = 1u << 11, // Bit 11
    OPERATION_MODE_SPECIFIC_12 = 1u << 12, // Bit 12
    OPERATION_MODE_SPECIFIC_13 = 1u << 13, // Bit 13
    OPERATION_MODE_SPECIFIC_14 = 1u << 14, // Bit 14
    MANUFACTURER_SPECIFIC_15   = 1u << 15, // Bit 15
};
enum class STATUSWORD_STATE : std::uint16_t {
    NOT_READY_TO_SWITCH_ON = 0x0000,
    SWITCH_ON_DISABLED     = 0x0040,
    READY_TO_SWITCH_ON     = 0x0021,
    SWITCHED_ON            = 0x0023,
    OPERATION_ENABLED      = 0x0027,
    QUICK_STOP_ACTIVE      = 0x0007,
    FAULT_REACTION_ACTIVE  = 0x000F,
    FAULT                  = 0x0008,
};
enum class CONTROLWORD_MASK : std::uint16_t {
    SWITCH_ON                = 1u << 0,  // Bit 0
    ENABLE_VOLTAGE           = 1u << 1,  // Bit 1
    QUICK_STOP               = 1u << 2,  // Bit 2
    ENABLE_OPERATION         = 1u << 3,  // Bit 3
    OPERATION_MODE_4         = 1u << 4,  // Bit 4
    OPERATION_MODE_5         = 1u << 5,  // Bit 5
    OPERATION_MODE_6         = 1u << 6,  // Bit 6
    FAULT_RESET              = 1u << 7,  // Bit 7
    HALT                     = 1u << 8,  // Bit 8
    MANUFACTURER_SPECIFIC_9  = 1u << 9,  // Bit 9
    MANUFACTURER_SPECIFIC_10 = 1u << 10, // Bit 10
    MANUFACTURER_SPECIFIC_11 = 1u << 11, // Bit 11
    RESERVED_12              = 1u << 12, // Bit 12
    RESERVED_13              = 1u << 13, // Bit 13
    RESERVED_14              = 1u << 14, // Bit 14
    RESERVED_15              = 1u << 15, // Bit 15
};

enum class CONTROLWORD_COMMAND : std::uint16_t {
    SHUTDOWN          = 0x0006,
    SWITCH_ON         = 0x0007,
    ENABLE_OPERATION  = 0x000F,
    DISABLE_OPERATION = 0x0007,
    QUICK_STOP        = 0x0002,
    DISABLE_VOLTAGE   = 0x0000,
    FAULT_RESET       = 0x0080,
};

enum class DIGITAL_INPUT_MASK : std::uint32_t {
    NEGATIVE_LIMIT_ERROR = 1UL << 0,
    POSITIVE_LIMIT_ERROR = 1UL << 1,
    HOME_SWITCH          = 1UL << 2,
    GENERAL_PURPOSE_A    = 1UL << 16,
    GENERAL_PURPOSE_B    = 1UL << 17,
    GENERAL_PURPOSE_C    = 1UL << 18,
    GENERAL_PURPOSE_D    = 1UL << 19,
    GENERAL_PURPOSE_E    = 1UL << 20,
    GENERAL_PURPOSE_F    = 1UL << 21,
    GENERAL_PURPOSE_G    = 1UL << 22,
    GENERAL_PURPOSE_H    = 1UL << 23,
    NEGATIVE_LIMIT       = 1UL << 24,
    POSITIVE_LIMIT       = 1UL << 25,
    TOUCH_PROBE          = 1UL << 26,
    DRIVE_ENABLE         = 1UL << 27,
    QUICK_STOP           = 1UL << 28,
};

enum class PDO_TRANSMISSION_TYPE : std::uint8_t {
    SYNCHRONOUS_ACYCLIC = 0x00,
    SYNCHRONOUS_1       = 0x01,
    ASYNCHRONOUS        = 0xFF,
};

} // namespace CANOPENCONTROLLER
