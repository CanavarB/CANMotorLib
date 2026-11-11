if(NOT TARGET can_interface)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../ICANController"
        "${CMAKE_CURRENT_BINARY_DIR}/ICANController"
    )
endif()

add_library(ak70_driver STATIC
    AK70_Driver.cpp
)

target_include_directories(ak70_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ak70_driver
    PUBLIC
        can_interface
)

