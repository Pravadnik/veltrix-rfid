cmake_minimum_required(VERSION 3.10)
project(rfid_cli CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

add_executable(rfid
  main.cpp
  protocol.cpp
  transport.cpp
)

# ---- install ----
# Installs the binary to <prefix>/bin (default prefix /usr/local).
#   cmake --install build                       -> /usr/local/bin/rfid (needs sudo)
#   cmake --install build --prefix ~/.local     -> ~/.local/bin/rfid   (no sudo)
include(GNUInstallDirs)
install(TARGETS rfid RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# ---- tests (ctest) ----
enable_testing()
add_executable(protocol_test tests/protocol_test.cpp protocol.cpp)
target_include_directories(protocol_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME protocol_test COMMAND protocol_test)
