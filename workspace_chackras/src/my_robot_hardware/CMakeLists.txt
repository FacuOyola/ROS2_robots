cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)

# Encontrar libmodbus e incluir sus rutas de cabeceras
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# 2. Configurar la librería del Plugin (¡Solo una vez!)
add_library(
  ${PROJECT_NAME}
  SHARED
  src/mobile_base_hardware_interface.cpp
)

# Incluir las carpetas de cabeceras locales y de libmodbus
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)

# Vincular las dependencias de ROS 2
ament_target_dependencies(${PROJECT_NAME}  
  rclcpp 
  rclcpp_lifecycle 
  hardware_interface 
  pluginlib
)

# Vincular la librería física de libmodbus
target_link_libraries(${PROJECT_NAME} 
  ${LIBMODBUS_LIBRARIES}
)

# 3. Exportar el plugin para ros2_control (¡Solo una vez!)
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)

# 4. Instalación de archivos en el sistema
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES my_robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# 5. Exportar dependencias para otros paquetes
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 6. Pruebas y Tests (Opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
