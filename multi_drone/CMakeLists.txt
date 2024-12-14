cmake_minimum_required(VERSION 3.5)
project(multi_drone)

# Найдем зависимости
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Установка файлов запуска
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Файлы конфигурации
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    controllers/base/base_controller.py
    controllers/x500/x500_base.py
    controllers/x500/x500.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY controllers/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/controllers
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY move_commands/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/move_commands
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY scripts/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY utils/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/utils
  USE_SOURCE_PERMISSIONS
)

# Упаковка проекта
ament_package()
