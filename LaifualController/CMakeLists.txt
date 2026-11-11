if(NOT TARGET canopen)
    add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/../canopen"
        "${CMAKE_CURRENT_BINARY_DIR}/canopen"
    )
endif()


add_library(laifual_controller STATIC
    LaifualController.cpp
)

target_include_directories(laifual_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(laifual_controller
    PUBLIC
        canopen
)
