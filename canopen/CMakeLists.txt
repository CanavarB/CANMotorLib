if(NOT TARGET can_interface)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../ICANController"
        "${CMAKE_CURRENT_BINARY_DIR}/ICANController"
    )
endif()


add_library(canopen INTERFACE)

target_include_directories(canopen
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(canopen
    INTERFACE
        can_interface
)

