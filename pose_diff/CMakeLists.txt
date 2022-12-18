project(pose_diff)

add_executable(${PROJECT_NAME}
        include/core/state.h
        include/core/rotation.h
        include/core/transform.h
        include/core/matrix.h
        include/point_to_point_icp.h
        include/point_to_plane_icp.h
        src/main.cpp
        src/point_to_point_icp.cpp
        src/point_to_plane_icp.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)