# Install script for directory: /home/pi/QT/MF/src/eigen3/eigen/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Cholesky"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/CholmodSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Core"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Dense"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Eigen"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Eigenvalues"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Geometry"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Householder"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/IterativeLinearSolvers"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Jacobi"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/LU"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/MetisSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/OrderingMethods"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/PaStiXSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/PardisoSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/QR"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/QtAlignedMalloc"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SPQRSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SVD"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/Sparse"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SparseCholesky"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SparseCore"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SparseLU"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SparseQR"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/StdDeque"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/StdList"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/StdVector"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/SuperLUSupport"
    "/home/pi/QT/MF/src/eigen3/eigen/Eigen/UmfPackSupport"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/pi/QT/MF/src/eigen3/eigen/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

