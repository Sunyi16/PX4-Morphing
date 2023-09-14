px4_add_module(
	MODULE examples_my_workq
	MAIN my_workq
	COMPILE_FLAGS
	SRCS
		my_workq.cpp
	DEPENDS
		px4_work_queue
	)
