find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Ceres  REQUIRED )


add_subdirectory(evaluate)

include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(${CERES_INCLUDE_DIRS})
include_directories(evaluate)

if(USE_CUDA)
    find_package(CUDA)
    include_directories("${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_CUDA)
endif(USE_CUDA)

add_library(feature SHARED "feature.cpp")
add_library(bucket SHARED "bucket.cpp")
add_library(utils SHARED "utils.cpp")
add_library(visualOdometry SHARED "visualOdometry.cpp")
add_library(rgbd_standalone SHARED "rgbd_standalone.cpp")
add_library(Frame SHARED "Frame.cpp")
add_library(optimizer SHARED "optimizer.cpp")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
message(${PROJECT_SOURCE_DIR})

add_executable( run main.cpp)

target_link_libraries( bucket ${OpenCV_LIBS} )
target_link_libraries( feature ${OpenCV_LIBS} bucket)
target_link_libraries( utils evaluate_odometry feature ${OpenCV_LIBS} )
target_link_libraries( visualOdometry utils bucket feature ${OpenCV_LIBS})
target_link_libraries( Frame ${OpenCV_LIBS} )
target_link_libraries( optimizer visualOdometry feature Frame ${OpenCV_LIBS} ceres Eigen3::Eigen )
target_link_libraries( run ${OpenCV_LIBS} bucket feature utils visualOdometry Frame rgbd_standalone optimizer)
