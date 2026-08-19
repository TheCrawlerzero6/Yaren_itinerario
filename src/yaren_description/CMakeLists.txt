cmake_minimum_required(VERSION 3.8)
project(yaren_description)

find_package(ament_cmake REQUIRED)

# El paquete no compila codigo: su contenido son recursos que deben
# quedar accesibles en share/ para que $(find yaren_description) y las URIs
# package:// los resuelvan tanto desde los ficheros de arranque como
# desde el simulador.
install(
  DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
