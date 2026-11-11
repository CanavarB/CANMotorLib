#pragma once
#include <CANopenController.hpp>

namespace LAIFUALCONTROLLER {
using namespace CANOPENCONTROLLER;

// RPDO1: Controlword, Target Position and Current Mode
typedef union {
    struct __attribute__((packed)) {
        uint16_t control_word;
        uint32_t target_position;
        MODES_OF_OPERATION mode;
    };
    PDO pdo;
} RPDO1_t;

// RPDO2: Profile Velocity and Target velocity
typedef union {
    struct __attribute__((packed)) {
        uint32_t profile_velocity;
        int32_t target_velocity;
    };
    PDO pdo;
} RPDO2_t;

// TPDO1: Statusword, Actual Position and Modes of Display
typedef union {
    struct __attribute__((packed)) {
        uint16_t status_word;
        int32_t actual_position;
        uint8_t mode_display;
    };
    PDO pdo;
} TPDO1_t;

// TPDO2: Temperature and Velocity actual value
typedef union {
    struct __attribute__((packed)) {
        int16_t temperature;
        int32_t velocity_actual_value;
    };
    PDO pdo;
} TPDO2_t;

class LaifualController : public CANopenController {
  public:
    int32_t first_pos = 0;
    LaifualController(ICANController& can_controller, MotorConfig_t config)
        : CANopenController(can_controller, config), rpdo1_message_offset(RPDO1_COB_ID_OFFSET + config.node_id),
          rpdo2_message_offset(RPDO2_COB_ID_OFFSET + config.node_id),
          tpdo1_message_offset(TPDO1_COB_ID_OFFSET + config.node_id),
          tpdo2_message_offset(TPDO2_COB_ID_OFFSET + config.node_id) {};
    ~LaifualController();

    err_t init(uint32_t wake_up_timeout_ms);
    err_t configure_PDO();

    // RxPDO1
    void setControlWord_PDO(uint16_t value) { getRPDO1()->control_word = value; }
    uint16_t getControlWord_PDO() { return getRPDO1()->control_word; }

    void setTargetPosition_PDO(int32_t target_position) { getRPDO1()->target_position = target_position; }
    int32_t getTargetPosition_PDO() { return getRPDO1()->target_position; }

    void setMode_PDO(MODES_OF_OPERATION mode) { getRPDO1()->mode = mode; }
    MODES_OF_OPERATION getMode_PDO() { return getRPDO1()->mode; }

    // RxPDO2
    void setProfileVelocity_PDO(uint32_t profile_velocity) { getRPDO2()->profile_velocity = profile_velocity; }
    uint32_t getProfileVelocity_PDO() { return getRPDO2()->profile_velocity; }

    void setTargetVelocity_PDO(int32_t target_velocity) { getRPDO2()->target_velocity = target_velocity; }
    int32_t getTargetVelocity_PDO() { return getRPDO2()->target_velocity; }

    // TxPDO1
    uint16_t getStatusWord_PDO() { return getTPDO1()->status_word; }
    int32_t getActualPosition_PDO() { return getTPDO1()->actual_position; }
    uint8_t getModeDisplay_PDO() { return getTPDO1()->mode_display; }

    // TxPDO2
    int32_t getVelocityActualValue_PDO() { return getTPDO2()->velocity_actual_value; }
    int16_t getTemperature_PDO() { return getTPDO2()->temperature; }

    err_t readTPDO1(uint16_t timeout_ms = 100)
    { return CanController.read(tpdo1_message_offset, getTPDO1()->pdo, timeout_ms); }

    err_t readTPDO2(uint16_t timeout_ms = 100)
    { return CanController.read(tpdo2_message_offset, getTPDO2()->pdo, timeout_ms); }

    err_t transmitRPDO1() { return CanController.write(rpdo1_message_offset, getRPDO1()->pdo, sizeof(RPDO1_t)); }

    err_t transmitRPDO2() { return CanController.write(rpdo2_message_offset, getRPDO2()->pdo, sizeof(RPDO2_t)); }

    err_t setReleaseBrake(uint32_t value) { return SDO_Write(OBJECT_INDEX::RELEASE_BRAKE, SUB_INDEX::ZERO, value); }

  private:
    inline RPDO1_t* getRPDO1() { return reinterpret_cast<RPDO1_t*>(RPDO1); }
    inline RPDO2_t* getRPDO2() { return reinterpret_cast<RPDO2_t*>(RPDO2); }
    inline TPDO1_t* getTPDO1() { return reinterpret_cast<TPDO1_t*>(TPDO1); }
    inline TPDO2_t* getTPDO2() { return reinterpret_cast<TPDO2_t*>(TPDO2); }
    uint16_t rpdo1_message_offset;
    uint16_t rpdo2_message_offset;

    uint16_t tpdo1_message_offset;
    uint16_t tpdo2_message_offset;
};
} // namespace LAIFUALCONTROLLER

static_assert(sizeof(LAIFUALCONTROLLER::RPDO1_t) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "RPDO1_t too big!");
static_assert(sizeof(LAIFUALCONTROLLER::RPDO2_t) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "RPDO2_t too big!");
static_assert(sizeof(LAIFUALCONTROLLER::TPDO1_t) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "TPDO1_t too big!");
static_assert(sizeof(LAIFUALCONTROLLER::TPDO2_t) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "TPDO2_t too big!");