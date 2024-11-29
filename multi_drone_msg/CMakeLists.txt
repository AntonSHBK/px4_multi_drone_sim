cmake_minimum_required(VERSION 3.5)
project(multi_drone_msg)

# Найдем зависимости
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Установка файлов запуска

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES geometry_msgs
)

# Упаковка проекта
ament_package()
