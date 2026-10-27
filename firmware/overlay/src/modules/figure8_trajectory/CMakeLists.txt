px4_add_module(
	MODULE modules__figure8_trajectory
	MAIN figure8_trajectory
	SRCS
		Figure8Trajectory.cpp
		Figure8Trajectory.hpp
		Figure8Mission.hpp
	DEPENDS
		px4_work_queue
)
