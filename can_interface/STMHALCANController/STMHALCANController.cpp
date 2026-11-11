// STMHALCANController.cpp

#include <STMHALCANController.hpp>

using namespace ICANCONTROLLER;
using namespace STMHALCANCONTROLLER;

STMHALCANController::STMHALCANController(FDCAN_HandleTypeDef* fdcan, uint32_t* filter_ids, uint32_t filter_ids_size)
    : fdcan(fdcan)
{

    /* HAL_StatusTypeDef ret = HAL_FDCAN_ConfigFilter(this->fdcan, &stFilterConfig);
    if(ret != HAL_OK)
    {
            fprintf(stderr, "CanbusController::initialize[]->HAL_FDCAN_ConfigFilter failed. Return:%d\r\n",eReturn);
            return -1;
    } */
}

err_t STMHALCANController::write(uint32_t can_id, uint8_t* tx_buffer, size_t tx_buffer_size, bool is_extended)
{ return OK; }

err_t STMHALCANController::read(uint32_t id, uint8_t* rx_buffer, uint16_t timeout_ms) { return OK; }

STMHALCANController::~STMHALCANController() {}
