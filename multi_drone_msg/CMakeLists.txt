cmake_minimum_required(VERSION 3.5)
project(multi_drone_msg)

# Включение зависимостей ROS 2
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Указание сообщения и типа сервиса
set(msg_files
  "msg/CoordinateDataMsg.msg"
  "msg/LocalAndGlobalCoordinatesMsg.msg"
  "msg/DroneParamsMsg.msg"
  "msg/DroneInformMsg.msg"
)

# Генерация сообщений
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

# Указание зависимостей для сборки
ament_export_dependencies(rosidl_default_runtime)

# Упаковка
ament_package()
