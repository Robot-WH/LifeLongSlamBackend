# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files
# Niko Suenderhauf <niko@etit.tu-chemnitz.de>
# Adapted by Felix Endres <endres@informatik.uni-freiburg.de>
IF(UNIX)

  #IF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
  # in cache already
  #  SET(G2O_FIND_QUIETLY TRUE)
#ENDIF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
  MESSAGE(STATUS "----------------------------------------------------------")
  MESSAGE(STATUS "-----------------Searching for g2o -----------------")
  MESSAGE(STATUS "----------------- FindG2O.cmake -----------------")
  MESSAGE(STATUS "----------------------------------------------------------")
  
  # 这段代码是使用 CMake 的 FIND_PATH 命令来寻找 G2O 库的头文件路径，其具体解释如下：

  # FIND_PATH：CMake 中用于寻找指定路径的命令。
  # G2O_INCLUDE_DIR：寻找到的 G2O 库的头文件路径将被存储在此变量中，供后续使用。
  # NAMES：指定要寻找的文件或目录名，这里指定了 "core"、"math_groups" 和 "types"，即 G2O 库中的三个主要部分。
  # PATHS：指定了要在哪些目录中查找，这里指定了 /usr/local 和 /usr。
  # PATH_SUFFIXES：指定在每个 PATHS 路径下要添加的子路径，这里指定了 include/, include/g2o，即在 PATHS 路径下寻找 include/, include/g2o 路径。
  # 在下面的FING_PATH中，将对 /usr/local/include/g2o、/usr/local/include、/usr/include/g2o和/usr/include 进行查找。
  # NO_DEFAULT_PATH 表示只在给定的目录下寻找(不会自动去子目录)，若没有NO_DEFAULT_PATH，则先会去指定目录下
  # 寻找，若寻找不到则会去预定义目录下寻找

  # FIND_PATH(G2O_INCLUDE_DIR
    # NAMES core math_groups types
    # PATHS /usr/local /usr
    # PATH_SUFFIXES include/g2o include
    # NO_DEFAULT_PATH)

  # FIND_PATH(G2O_INCLUDE_DIR
  #   NAMES g2o
  #   PATHS /opt/ros/humble/include
  #   NO_DEFAULT_PATH)    # 不给定PATHS以及NO_DEFAULT_PATH 表示去预定于的路径下搜索，包括/usr/local/include  
  

  # IF (G2O_INCLUDE_DIR)
  #   MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  # ENDIF (G2O_INCLUDE_DIR)

  # FIND_LIBRARY(G2O_CORE_LIB             
  #   NAMES g2o_core g2o_core_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_STUFF_LIB            
  #   NAMES g2o_stuff g2o_stuff_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_TYPES_SLAM2D_LIB     
  #   NAMES g2o_types_slam2d g2o_types_slam2d_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB     
  #   NAMES g2o_types_slam3d g2o_types_slam3d_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)

  # FIND_LIBRARY(G2O_SOLVER_CHOLMOD_LIB   
  #   NAMES g2o_solver_cholmod g2o_solver_cholmod_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_SOLVER_PCG_LIB       
  #   NAMES g2o_solver_pcg g2o_solver_pcg_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_SOLVER_CSPARSE_LIB   
  #   NAMES g2o_solver_csparse g2o_solver_csparse_rd
  #   PATHS /usr/local /usr /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_SOLVER_EIGEN_LIB   
  #   NAMES g2o_solver_eigen g2o_solver_eigen_rd   # lib前缀 和后缀名 .so 不需要写 
  #   PATHS /usr/local /usr /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)

  # FIND_LIBRARY(G2O_INCREMENTAL_LIB      
  #   NAMES g2o_incremental g2o_incremental_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)
  # FIND_LIBRARY(G2O_CSPARSE_EXTENSION_LIB
  #   NAMES g2o_csparse_extension g2o_csparse_extension_rd
  #   PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} /opt/ros/humble/lib/x86_64-linux-gnu
  #   PATH_SUFFIXES lib)

  # SET(G2O_LIBRARIES 
  #   ${G2O_CSPARSE_EXTENSION_LIB}
  #   ${G2O_CORE_LIB}           
  #   ${G2O_STUFF_LIB}          
  #   ${G2O_TYPES_SLAM2D_LIB}   
  #   ${G2O_TYPES_SLAM3D_LIB}   
  #   ${G2O_SOLVER_CHOLMOD_LIB} 
  #   ${G2O_SOLVER_PCG_LIB}     
  #   ${G2O_SOLVER_CSPARSE_LIB} 
  #   ${G2O_SOLVER_EIGEN_LIB}
  #   ${G2O_INCREMENTAL_LIB}                        
  # )

  # IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
  #   SET(G2O_FOUND "YES")
  #   IF(NOT G2O_FIND_QUIETLY)
  #     MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
  #   ENDIF(NOT G2O_FIND_QUIETLY)
  # ELSE(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
  #   IF(NOT G2O_LIBRARIES)
  #     IF(G2O_FIND_REQUIRED)     # 调用 find_package(g2o) 命令时生成的。如果你在调用 find_package(g2o REQUIRED) 时指定了 REQUIRED 选项，那么 G2O_FIND_REQUIRED 变量的值为真。否则，它的值为假。
  #       message(FATAL_ERROR "Could not find libg2o!")
  #     ENDIF(G2O_FIND_REQUIRED)
  #   ENDIF(NOT G2O_LIBRARIES)

  #   IF(NOT G2O_INCLUDE_DIR)
  #     IF(G2O_FIND_REQUIRED)
  #       message(FATAL_ERROR "Could not find g2o include directory!")
  #     ENDIF(G2O_FIND_REQUIRED)
  #   ENDIF(NOT G2O_INCLUDE_DIR)
  # ENDIF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)


  FIND_PATH(G2O_INCLUDE_DIR
    NAMES g2o
    PATHS /usr/local/include
    NO_DEFAULT_PATH)    # 不给定PATHS以及NO_DEFAULT_PATH 表示去预定于的路径下搜索，包括/usr/local/include  
  

  IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF (G2O_INCLUDE_DIR)

  FIND_LIBRARY(G2O_CORE_LIB             
    NAMES g2o_core g2o_core_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_STUFF_LIB            
    NAMES g2o_stuff g2o_stuff_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_TYPES_SLAM2D_LIB     
    NAMES g2o_types_slam2d g2o_types_slam2d_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB     
    NAMES g2o_types_slam3d g2o_types_slam3d_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)

  FIND_LIBRARY(G2O_SOLVER_CHOLMOD_LIB   
    NAMES g2o_solver_cholmod g2o_solver_cholmod_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_PCG_LIB       
    NAMES g2o_solver_pcg g2o_solver_pcg_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_CSPARSE_LIB   
    NAMES g2o_solver_csparse g2o_solver_csparse_rd
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_EIGEN_LIB   
    NAMES g2o_solver_eigen g2o_solver_eigen_rd   # lib前缀 和后缀名 .so 不需要写 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)

  FIND_LIBRARY(G2O_INCREMENTAL_LIB      
    NAMES g2o_incremental g2o_incremental_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_CSPARSE_EXTENSION_LIB
    NAMES g2o_csparse_extension g2o_csparse_extension_rd
    PATHS /usr/local /usr ${CMAKE_PREFIX_PATH} 
    PATH_SUFFIXES lib)

  SET(G2O_LIBRARIES 
    ${G2O_CSPARSE_EXTENSION_LIB}
    ${G2O_CORE_LIB}           
    ${G2O_STUFF_LIB}          
    ${G2O_TYPES_SLAM2D_LIB}   
    ${G2O_TYPES_SLAM3D_LIB}   
    ${G2O_SOLVER_CHOLMOD_LIB} 
    ${G2O_SOLVER_PCG_LIB}     
    ${G2O_SOLVER_CSPARSE_LIB} 
    ${G2O_SOLVER_EIGEN_LIB}
    ${G2O_INCREMENTAL_LIB}                        
  )

  IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    SET(G2O_FOUND "YES")
    IF(NOT G2O_FIND_QUIETLY)
      MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
    ENDIF(NOT G2O_FIND_QUIETLY)
  ELSE(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    IF(NOT G2O_LIBRARIES)
      IF(G2O_FIND_REQUIRED)     # 调用 find_package(g2o) 命令时生成的。如果你在调用 find_package(g2o REQUIRED) 时指定了 REQUIRED 选项，那么 G2O_FIND_REQUIRED 变量的值为真。否则，它的值为假。
        message(FATAL_ERROR "Could not find libg2o!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_LIBRARIES)

    IF(NOT G2O_INCLUDE_DIR)
      IF(G2O_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find g2o include directory!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_INCLUDE_DIR)
  ENDIF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)

ENDIF(UNIX)

