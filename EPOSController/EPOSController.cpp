#include <EPOSController.hpp>
#include <stdio.h>
#include <string>
#include <unistd.h>

using namespace EPOSCONTROLLER;
using namespace CANOPENCONTROLLER;

err_t EPOSController::init(uint32_t wake_up_timeout_ms)
{
    err_t err;
    RETURN_ON_NOK_LOG(resetCommunication(), "init | Failed to stop remote node");
    RETURN_ON_NOK_LOG(resetRemoteNode(), "init | Failed to reset communication");
    RETURN_ON_NOK_LOG(setProfileVelocity(convertFromRPMToMotorPulse(
                          config.max_profile_velocity_rpm, config.encoder_resolution, config.gear_reduction_ratio)),
                      "init | Failed to set profile velocity");

    uint8_t bootup[STANDARD_SDO_MESSAGE_SIZE] = {0};
    RETURN_ON_NOK_LOG(CanController.read(HEARTBEAT_COB_ID_OFFSET + config.node_id, bootup, wake_up_timeout_ms),
                      "init: boot-up message timeout");

    RETURN_ON_NOK_LOG(clearFault(), "init | Failed to clear fault!");
    RETURN_ON_NOK_LOG(setProfileVelocity(convertFromRPMToMotorPulse(
                          config.max_profile_velocity_rpm, config.encoder_resolution, config.gear_reduction_ratio)),
                      "init | Failed to set profile velocity");
    RETURN_ON_NOK_LOG(
        setProfileAcceleration(convertFromRPMToMotorPulse(config.profile_acceleration_rpm_per_sec,
                                                          config.encoder_resolution, config.gear_reduction_ratio)),
        "init | Failed to set profile acceleration");
    RETURN_ON_NOK_LOG(
        setProfileDeceleration(convertFromRPMToMotorPulse(config.profile_deceleration_rpm_per_sec,
                                                          config.encoder_resolution, config.gear_reduction_ratio)),
        "init | Failed to set profile deceleration");
    return OK;
    /* err = setSyncMessageStatus(0x0080);
    if (err != OK) {
        fprintf(stderr, "init | Failed to set sync message status\r\n");
        return err;
    }
    err = setCommunicationCyclePeriod(1000);
    if (err != OK) {
        fprintf(stderr, "init | Failed to set communication cycle period\r\n");
        return err;
    } */
}

err_t EPOSController::configurePDO()
{
    NMT_Send(NMT_COMMAND::PRE_OP);

    PDO_MAPPING RPDO1_Mapping[] = {{.object_index = OBJECT_INDEX::CONTROL_WORD,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(RPDO1_Status.control_word) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::TARGET_POSITION,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(RPDO1_Status.target_position) * BITS_PER_BYTE}};

    SDO_Write(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, (0x80000000) + rpdo1_message_offset);
    SDO_Write(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
              (uint8_t)PDO_TRANSMISSION_TYPE::SYNCHRONOUS_1);

    SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ZERO, 0x00);
    SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ONE, RPDO1_Mapping[0].value);
    SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::TWO, RPDO1_Mapping[1].value);
    SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ZERO, (uint8_t)(sizeof(RPDO1_Mapping) / sizeof(PDO_MAPPING)));
    SDO_Write(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, rpdo1_message_offset);

    // RPDO2: Target torque and Target velocity
    /* SDO_Write(RPDO2_COMMUNICATION_PARAMETER_INDEX, SUB_INDEX::ONE, 0x80000202);
    SDO_Write(RPDO2_COMMUNICATION_PARAMETER_INDEX, SUB_INDEX::TWO, 0x00);

    SDO_Write(RPDO2_MAPPING_INDEX, SUB_INDEX::ZERO, 0x00);
    SDO_Write(RPDO2_MAPPING_INDEX, SUB_INDEX::ONE, 0x60710010);
    SDO_Write(RPDO2_MAPPING_INDEX, SUB_INDEX::TWO, 0x60FF0020);
    SDO_Write(RPDO2_MAPPING_INDEX, SUB_INDEX::ZERO, 0x02);
    SDO_Write(RPDO2_COMMUNICATION_PARAMETER_INDEX, SUB_INDEX::ONE, 0x00000202); */

    PDO_MAPPING TPDO1_Mapping[] = {{.object_index = OBJECT_INDEX::STATUS_WORD,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(TPDO1_Status.status_word) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::POSITION_ACTUAL_VALUE,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(TPDO1_Status.actual_position) * BITS_PER_BYTE}};

    SDO_Write(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, 0x80000000 + tpdo1_message_offset);
    SDO_Write(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
              (uint8_t)PDO_TRANSMISSION_TYPE::SYNCHRONOUS_ACYCLIC);

    SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ZERO, 0x00);
    SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ONE, TPDO1_Mapping[0].value);
    SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::TWO, TPDO1_Mapping[1].value);
    SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ZERO, (uint8_t)(sizeof(TPDO1_Mapping) / sizeof(PDO_MAPPING)));
    SDO_Write(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, tpdo1_message_offset);

    PDO_MAPPING TPDO2_Mapping[] = {{.object_index = OBJECT_INDEX::DIGITAL_INPUTS,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(TPDO2_Status.digital_inputs) * BITS_PER_BYTE}};

    SDO_Write(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, 0x80000000 + tpdo2_message_offset);
    SDO_Write(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
              (uint8_t)PDO_TRANSMISSION_TYPE::ASYNCHRONOUS);

    SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ZERO, 0x00);
    SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ONE, TPDO2_Mapping[0].value);
    SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ZERO, (uint8_t)(sizeof(TPDO2_Mapping) / sizeof(PDO_MAPPING)));
    SDO_Write(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, tpdo2_message_offset);

    CanController.addCallback(tpdo1_message_offset,
                              [this](const uint8_t* data, uint8_t size) -> err_t { return setTPDO1(data, size); });
    CanController.addCallback(tpdo2_message_offset,
                              [this](const uint8_t* data, uint8_t size) -> err_t { return setTPDO2(data, size); });

    return OK;
}

err_t EPOSController::setTargetTorque(int16_t target_torque)
{
    err_t err                  = OK;
    RPDO2_Status.target_torque = target_torque;
    err                        = transmitRPDO2();
    return err;
}

err_t EPOSController::setTargetPosition(int32_t target_position)
{
    err_t err                    = OK;
    RPDO1_Status.control_word    = 0x2F;
    RPDO1_Status.target_position = target_position;
    err                          = transmitRPDO1();

    return err;
}