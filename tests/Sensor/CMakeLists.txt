add_executable(testSensor
  testImage.cc
)

target_link_libraries(testSensor Sensor Core)
create_test(testSensor)