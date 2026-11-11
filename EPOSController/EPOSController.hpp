#pragma once
#include "CANopenController.hpp"

namespace EPOSCONTROLLER {
using namespace CANOPENCONTROLLER;

typedef union {
    struct __attribute__((packed)) {
        uint16_t control_word;
        int32_t target_position;
    };
    uint8_t raw[6];
} RPDO1_Message;

typedef union {
    struct __attribute__((packed)) {
        uint16_t target_torque;
        uint32_t target_velocity;
    };
    uint8_t raw[6];
} RPDO2_Message;

typedef union {
    struct __attribute__((packed)) {
        uint16_t status_word;
        int32_t actual_position;
    };
    uint8_t raw[6];
} TPDO1_Message;

typedef union {
    struct __attribute__((packed)) {
        uint32_t digital_inputs;
    };
    uint8_t raw[4];
} TPDO2_Message;

class EPOSController : public CANopenController {
  public:
    EPOSController(ICANController& can_controller, MotorConfig_t config)
        : CANopenController(can_controller, config), rpdo1_message_offset(RPDO1_COB_ID_OFFSET + config.node_id),
          rpdo2_message_offset(RPDO2_COB_ID_OFFSET + config.node_id),
          tpdo1_message_offset(TPDO1_COB_ID_OFFSET + config.node_id),
          tpdo2_message_offset(TPDO2_COB_ID_OFFSET + config.node_id)
    {
    }
    ~EPOSController() = default;
    err_t init(uint32_t wake_up_timeout_ms);
    err_t configurePDO();

    void setControlWord_PDO(uint16_t value) { RPDO1_Status.control_word = value; }
    uint16_t getControlWord_PDO() { return RPDO1_Status.control_word; }

    void setTargetPosition_PDO(int32_t target_position) { RPDO1_Status.target_position = target_position; }
    int32_t getTargetPosition_PDO() { return RPDO1_Status.target_position; }

    TPDO1_Message getTPDO1() { return TPDO1_Status; };
    TPDO2_Message getTPDO2() { return TPDO2_Status; };
    int32_t getActualPosition_PDO();
    uint32_t getDigitalInputs();

    err_t setTargetPosition(int32_t target_position);
    err_t setTargetTorque(int16_t target_torque);
    err_t transmitRPDO1() { return CanController.write(rpdo1_message_offset, RPDO1_Status.raw, sizeof(RPDO1_Status)); }
    err_t transmitRPDO2() { return CanController.write(rpdo2_message_offset, RPDO2_Status.raw, sizeof(RPDO2_Status)); }

  private:
    uint16_t rpdo1_message_offset;
    uint16_t rpdo2_message_offset;

    uint16_t tpdo1_message_offset;
    uint16_t tpdo2_message_offset;
    RPDO1_Message RPDO1_Status;
    RPDO2_Message RPDO2_Status;
    TPDO1_Message TPDO1_Status;
    TPDO2_Message TPDO2_Status;
};
} // namespace EPOSCONTROLLER

static_assert(sizeof(EPOSCONTROLLER::RPDO1_Message) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "RPDO1_Message too big!");
static_assert(sizeof(EPOSCONTROLLER::RPDO2_Message) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "RPDO2_Message too big!");
static_assert(sizeof(EPOSCONTROLLER::TPDO1_Message) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "TPDO1_Message too big!");
static_assert(sizeof(EPOSCONTROLLER::TPDO2_Message) <= CANOPENCONTROLLER::MAX_PDO_SIZE, "TPDO2_Message too big!");