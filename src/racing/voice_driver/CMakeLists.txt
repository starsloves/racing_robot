cmake_minimum_required(VERSION 3.8)
project(voice_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlayVoice.srv"
)

add_executable(voice_node src/voice_node.cpp)
ament_target_dependencies(voice_node rclcpp std_msgs)

ament_python_install_package(voice_api)

install(TARGETS voice_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  voice_api/play_voice_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME play_voice
)

install(PROGRAMS
  voice_api/voice_broadcast_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_broadcast
)

install(PROGRAMS
  voice_api/voice_broadcast_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_broadcast_node
)

install(PROGRAMS
  voice_api/voice_probe_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_probe
)

install(PROGRAMS
  voice_api/voice_hardware_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_hardware_test
)

install(PROGRAMS
  voice_api/voice_force_play.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_force_play
)

install(PROGRAMS
  voice_api/voice_speak_now.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_speak
)

install(PROGRAMS
  voice_api/voice_speak_text.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_speak_text
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
