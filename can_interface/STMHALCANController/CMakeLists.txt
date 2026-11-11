add_library(canmotorlib_stm_hal
    STMHALCANController.cpp
)

target_include_directories(canmotorlib_stm_hal
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(canmotorlib_stm_hal
    PUBLIC cxx_std_17
)
target_link_libraries(canmotorlib_stm_hal PUBLIC stm32cubemx)