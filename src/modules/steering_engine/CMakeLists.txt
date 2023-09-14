px4_add_module(
	MODULE module__steering_engine
	MAIN steering_engine
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		steering_engine.cpp
)
