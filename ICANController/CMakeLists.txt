add_library(can_interface INTERFACE)

target_include_directories(can_interface
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)
