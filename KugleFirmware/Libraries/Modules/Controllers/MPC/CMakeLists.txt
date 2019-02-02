# Configure build as release (for maximum optimization) but still add debug information (-g flag)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "Adding library: mpc [${CMAKE_BUILD_TYPE}]")

set(library_dependencies  m stdc++ ${Eigen3_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} )

#FILE(GLOB_RECURSE binary_sources *.cpp) # find all cpp files
#FILE(GLOB_RECURSE binary_headers *.hpp) # find all hpp files

set(mpc_sources "${CMAKE_CURRENT_SOURCE_DIR}/MPC.cpp"	
		"${CMAKE_CURRENT_SOURCE_DIR}/Path.cpp"		
		"${CMAKE_CURRENT_SOURCE_DIR}/Trajectory.cpp"	
		"${CMAKE_CURRENT_SOURCE_DIR}/codegen/acado_solver.c"		
 		"${CMAKE_CURRENT_SOURCE_DIR}/codegen/acado_integrator.c"
		"${CMAKE_CURRENT_SOURCE_DIR}/codegen/acado_auxiliary_functions.c"
		"${CMAKE_CURRENT_SOURCE_DIR}/codegen/acado_qpoases3_interface.c"
)

# Consider to move qpOASES as a seperate library which this code depends on
set(solver_sources "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Bounds.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Constraints.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Indexlist.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Matrices.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/MessageHandling.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Options.c"
                   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Flipper.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/QProblem.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/QProblemB.c"
		   "${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src/Utils.c"
)
set(include_directories "${CMAKE_CURRENT_SOURCE_DIR}"
 			"${CMAKE_CURRENT_SOURCE_DIR}/codegen"
			"${CMAKE_CURRENT_SOURCE_DIR}/qpoases3"
			"${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/include"
			"${CMAKE_CURRENT_SOURCE_DIR}/qpoases3/src"
			"${EIGEN3_INCLUDE_DIRS}"
			"${Boost_INCLUDE_DIRS}"
			"${OpenCV_INCLUDE_DIRS}"
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -finline-functions -D__CODE_GENERATION__")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -finline-functions")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -finline-functions -D__CODE_GENERATION__")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -finline-functions")

set(GLOBAL_HEADERS ${GLOBAL_HEADERS} ${binary_headers} PARENT_SCOPE)

add_library( mpc SHARED ${mpc_sources} ${solver_sources} )
target_link_libraries( mpc ${library_dependencies} )
target_include_directories( mpc PUBLIC ${include_directories})

set_target_properties( mpc PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra -D__CODE_GENERATION__ -DQPOASES_CUSTOM_INTERFACE=\"acado_qpoases3_interface.h\"" ) # enable debugging
