project(Core)

add_library(${PROJECT_NAME}  ${SF_LIB_TYPE}
            src/temp.cc)

target_include_directories(${PROJECT_NAME}  
                        PUBLIC ${PROJECT_SOURCE_DIR}/include
                        PUBLIC ${PROJECT_SOURCE_DIR})      