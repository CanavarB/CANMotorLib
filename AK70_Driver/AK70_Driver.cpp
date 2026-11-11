#include <canfd/devices/AK70_Driver/AK70_Driver.hpp>
#include <memory>

using namespace std;
using namespace AK70_Driver;

AK70_Driver::AK70_Driver::AK70_Driver(ICANController* CanController, std::uint8_t motor_id)
{
    this->CanController = CanController;
    this->motor_id = motor_id;

    mode_ids.duty_cycle = motor_id + CAN_PACKET_SET_DUTY | CAN_EFF_FLAG;                    // Extended ID
    mode_ids.current_loop = motor_id + CAN_PACKET_SET_CURRENT | CAN_EFF_FLAG;               // Extended ID
    mode_ids.current_break = motor_id + CAN_PACKET_SET_CURRENT_BRAKE | CAN_EFF_FLAG;        // Extended ID
    mode_ids.speed_loop = motor_id + (CAN_PACKET_SET_RPM << 8) | CAN_EFF_FLAG;              // Extended ID
    mode_ids.position_loop = motor_id + CAN_PACKET_SET_POS | CAN_EFF_FLAG;                  // Extended ID
    mode_ids.set_origin = motor_id + CAN_PACKET_SET_ORIGIN_HERE | CAN_EFF_FLAG;             // Extended ID
    mode_ids.position_speed_loop = motor_id + (CAN_PACKET_SET_POS_SPD << 8) | CAN_EFF_FLAG; // Extended ID
    mode_ids.upload_message = motor_id + (CAN_PACKET_UPLOAD_MESSAGE << 8) | CAN_EFF_FLAG;   // Extended ID
}

err_t AK70_Driver::AK70_Driver::read_upload()
{
    err_t err = CanController->read(mode_ids.upload_message, upload_msg.raw, 3);
    if (err != OK) return err;
    upload_msg.position = be16toh(upload_msg.position);
    upload_msg.velocity = be16toh(upload_msg.velocity);
    upload_msg.current = be16toh(upload_msg.current);
    return OK;
}

double AK70_Driver::AK70_Driver::get_position() { return static_cast<double>(upload_msg.position) * 0.1; }

double AK70_Driver::AK70_Driver::get_velocity() { return static_cast<double>(upload_msg.velocity) * 10.0; }

double AK70_Driver::AK70_Driver::get_current() { return static_cast<double>(upload_msg.current) * 0.01; }

err_t AK70_Driver::AK70_Driver::position_speed_loop(int32_t position, int16_t speed, int16_t acceleration)
{
    AK70_Position_Speed_Loop_Message request;
    request.position = htobe32(position * 10000.0);
    request.speed = htobe16(speed);
    request.acceleration = htobe16(acceleration);

    return CanController->write(mode_ids.position_speed_loop, request.raw, sizeof(request), true);
}

err_t AK70_Driver::AK70_Driver::speed_loop(int32_t speed)
{
    AK70_Speed_Loop_Message request;
    request.speed = htobe32(speed);
    return CanController->write(mode_ids.speed_loop, request.raw, sizeof(request), true);
}
