
message("CMAKE_BINARY_DIR: ${CMAKE_ROOT}" )

macro(create_test target)
  target_link_libraries(${target} gtest_main)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro(create_test)

add_subdirectory(Sensor)
add_subdirectory(Feature)
