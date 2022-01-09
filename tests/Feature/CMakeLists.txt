add_executable(testFeature
  testDetector.cc
)

target_link_libraries(testFeature Sensor Core Feature)
create_test(testFeature)