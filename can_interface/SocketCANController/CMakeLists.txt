if(NOT TARGET can_interface)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../../ICANController"
        "${CMAKE_CURRENT_BINARY_DIR}/ICANController"
    )
endif()

add_library(socketcan_controller STATIC
    SocketCANController.cpp
)

target_include_directories(socketcan_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(socketcan_controller
    PUBLIC
        can_interface
)
