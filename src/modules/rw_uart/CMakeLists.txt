px4_add_module(
	MODULE module_rw_uart
	MAIN rw_uart
	STACK_MAIN 2000
	COMPILE_FLAGS
	SRCS
		rw_uart.c
)
