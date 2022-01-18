project(Feature)

add_library(${PROJECT_NAME} ${SF_LIB_TYPE}
             src/Keypoint.cc
             src/Detector.cc
             src/Descriptor.cc)

target_include_directories(${PROJECT_NAME} 
                            PUBLIC ${PROJECT_SOURCE_DIR}/include
                            PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME}
                      Core Sensor)
