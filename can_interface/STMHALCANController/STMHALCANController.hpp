#pragma once
#include "../ICANController.hpp"
#include "stm32h7xx_hal.h"

using namespace ICANCONTROLLER;

namespace STMHALCANCONTROLLER {

class STMHALCANController : public ICANController {
  public:
    STMHALCANController(FDCAN_HandleTypeDef* fdcan, uint32_t* filter_ids, uint32_t filter_ids_size);
    ~STMHALCANController();
    err_t write(uint32_t can_id, uint8_t* tx_buffer, uint8_t tx_buffer_size, bool is_extended = false) override;
    err_t read(uint32_t id, uint8_t* rx_buffer, uint16_t timeout_ms) override;

  private:
    FDCAN_HandleTypeDef* fdcan;
};

} // namespace STMHALCANCONTROLLER
