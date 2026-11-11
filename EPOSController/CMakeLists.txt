if(NOT TARGET canopen)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../canopen"
        "${CMAKE_CURRENT_BINARY_DIR}/canopen"
    )
endif()



add_library(epos_controller STATIC
    EPOSController.cpp
)

target_include_directories(epos_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(epos_controller
    PUBLIC
        canopen
)
