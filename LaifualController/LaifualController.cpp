#include <LaifualController.hpp>
#include <cstdint>
#include <stdio.h>

using namespace LAIFUALCONTROLLER;
using namespace CANOPENCONTROLLER;

err_t LaifualController::init(uint32_t wake_up_timeout_ms)
{

    err_t err;
    RETURN_ON_NOK_LOG(stopRemoteNode(), "init | Failed to stop remote node");
    RETURN_ON_NOK_LOG(resetRemoteNode(), "init | Failed to reset communication");

    uint8_t bootup[STANDARD_SDO_MESSAGE_SIZE] = {0};
    RETURN_ON_NOK_LOG(CanController.read(HEARTBEAT_COB_ID_OFFSET + config.node_id, bootup, wake_up_timeout_ms),
                      "init: boot-up message timeout");

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
    RETURN_ON_NOK_LOG(setSyncMessageStatus(0x0080), "init | Failed to set sync message status");
    RETURN_ON_NOK_LOG(setCommunicationCyclePeriod(1000), "init | Failed to set communication cycle period");
    RETURN_ON_NOK_LOG(setInterpolationTimeIndex(0), "init | Failed to set Interpolation Time Index");
    RETURN_ON_NOK_LOG(getActualPosition(&first_pos), "init | First Position get Failed!");
    RETURN_ON_NOK_LOG(setHomeOffset(config.position_offset), "init | Failed to set home offset position");
    return err;
}

err_t LaifualController::configure_PDO()
{
    err_t err;
    NMT_Send(NMT_COMMAND::PRE_OP);

    PDO_MAPPING RPDO1_Mapping[] = {{.object_index = OBJECT_INDEX::CONTROL_WORD,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getRPDO1()->control_word) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::TARGET_POSITION,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getRPDO1()->target_position) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::MODES_OF_OPERATION,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getRPDO1()->mode) * BITS_PER_BYTE}};

    RETURN_ON_NOK(
        SDO_Write<uint32_t>(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, (0x80000000) + rpdo1_message_offset));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
                            (uint8_t)PDO_TRANSMISSION_TYPE::SYNCHRONOUS_1));
    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ZERO, 0x00));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ONE, RPDO1_Mapping[0].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::TWO, RPDO1_Mapping[1].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::THREE, RPDO1_Mapping[2].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_MAPPING, SUB_INDEX::ZERO,
                            (uint8_t)(sizeof(RPDO1_Mapping) / sizeof(PDO_MAPPING))));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, rpdo1_message_offset));

    PDO_MAPPING RPDO2_Mapping[] = {{.object_index = OBJECT_INDEX::PROFILE_VELOCITY,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getRPDO2()->profile_velocity) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::TARGET_VELOCITY,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getRPDO2()->target_velocity) * BITS_PER_BYTE}};

    RETURN_ON_NOK(
        SDO_Write<uint32_t>(OBJECT_INDEX::RPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, (0x80000000) + rpdo2_message_offset));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
                            (uint8_t)PDO_TRANSMISSION_TYPE::SYNCHRONOUS_1));
    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::RPDO2_MAPPING, SUB_INDEX::ZERO, 0x00));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO2_MAPPING, SUB_INDEX::ONE, RPDO2_Mapping[0].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO2_MAPPING, SUB_INDEX::TWO, RPDO2_Mapping[1].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO2_MAPPING, SUB_INDEX::ZERO,
                            (uint8_t)(sizeof(RPDO1_Mapping) / sizeof(PDO_MAPPING))));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::RPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, rpdo2_message_offset));

    PDO_MAPPING TPDO1_Mapping[] = {{.object_index = OBJECT_INDEX::STATUS_WORD,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getTPDO1()->status_word) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::MODES_OF_OPERATION_DISPLAY,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getTPDO1()->mode_display) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::POSITION_ACTUAL_VALUE,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getTPDO1()->actual_position) * BITS_PER_BYTE}};

    RETURN_ON_NOK(
        SDO_Write<uint32_t>(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, 0x80000000 + tpdo1_message_offset));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
                            (uint8_t)PDO_TRANSMISSION_TYPE::ASYNCHRONOUS));
    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::THREE, 100));
    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ZERO, 0x00));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ONE, TPDO1_Mapping[0].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::TWO, TPDO1_Mapping[1].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::THREE, TPDO1_Mapping[2].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_MAPPING, SUB_INDEX::ZERO,
                            (uint8_t)(sizeof(TPDO1_Mapping) / sizeof(PDO_MAPPING))));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO1_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, tpdo1_message_offset));

    PDO_MAPPING TPDO2_Mapping[] = {{.object_index = OBJECT_INDEX::TEMPERATURE,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getTPDO2()->temperature) * BITS_PER_BYTE},
                                   {.object_index = OBJECT_INDEX::VELOCITY_ACTUAL_VALUE,
                                    .sub_index    = SUB_INDEX::ZERO,
                                    .length       = sizeof(getTPDO2()->velocity_actual_value) * BITS_PER_BYTE}};

    RETURN_ON_NOK(
        SDO_Write<uint32_t>(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, 0x80000000 + tpdo2_message_offset));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::TWO,
                            (uint8_t)PDO_TRANSMISSION_TYPE::ASYNCHRONOUS));
    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::THREE, 100));

    RETURN_ON_NOK(SDO_Write<uint32_t>(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ZERO, 0x00));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ONE, TPDO2_Mapping[0].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::TWO, TPDO2_Mapping[1].value));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO2_MAPPING, SUB_INDEX::ZERO,
                            (uint8_t)(sizeof(TPDO2_Mapping) / sizeof(PDO_MAPPING))));
    RETURN_ON_NOK(SDO_Write(OBJECT_INDEX::TPDO2_COMMUNICATION_PARAMETER, SUB_INDEX::ONE, tpdo2_message_offset));

    CanController.addCallback(tpdo1_message_offset,
                              [this](const uint8_t* data, uint8_t size) -> err_t { return setTPDO1(data, size); });
    CanController.addCallback(tpdo2_message_offset,
                              [this](const uint8_t* data, uint8_t size) -> err_t { return setTPDO2(data, size); });

    return OK;
}
LaifualController::~LaifualController()
{
    setControlWord_PDO(0x00);
    transmitRPDO1();
    this->SYNC_Send();
    fprintf(stderr, "Node %d: Deleted.\r\n", this->config.node_id);
}