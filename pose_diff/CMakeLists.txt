project(pose_diff)

add_executable(${PROJECT_NAME}
        include/state.h
        include/rotation.h
        include/transform.h
        include/matrix.h
        src/main.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)