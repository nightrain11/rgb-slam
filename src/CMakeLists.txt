SET(ENV{Platform} x64) 
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET( CMAKE_CXX_FLAGS "-std=c++11 -g")
#########################################################################
# 添加g2o的依赖
 #因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST(  APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
LINK_LIBRARIES( ${G2O_LIBRARY_DIRS} ${CSPARSE_LIBRARY_DIRS} )
########################################################################
#ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
#TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS} 
 #   ${PCL_LIBRARIES} )
ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase ${OpenCV_LIBS} ${PCL_LIBRARIES} )
#ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
#TARGET_LINK_LIBRARIES( detectFeatures libslambase.a ${OpenCV_LIBS} 
 #   ${PCL_LIBRARIES} )
#ADD_EXECUTABLE( jointPointCloud jointPointCloud.cpp )
#TARGET_LINK_LIBRARIES( jointPointCloud slambase ${OpenCV_LIBS} 
 #   ${PCL_LIBRARIES} )
#ADD_EXECUTABLE( visualOdometry visualOdometry.cpp )
#TARGET_LINK_LIBRARIES( visualOdometry slambase ${OpenCV_LIBS} 
 #   ${PCL_LIBRARIES} )
ADD_EXECUTABLE( slamEnd slamEnd.cpp )
TARGET_LINK_LIBRARIES( slamEnd slambase ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}     g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})
