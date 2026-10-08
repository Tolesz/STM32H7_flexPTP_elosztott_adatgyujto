target_sources(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    main.c
    syscall.c
    sysmem.c
    system_stm32h7xx.c
    capture_handler.c
    ptp_tim_sync.c
    cmds.c
    cmds.h
    stm32h7xx_it.c
    stm32h7xx_hal_msp.c
    stm32h7xx_hal_timebase_tim.c
    etherlib_options.c
)

target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    .
)

add_subdirectory(standard_output)
add_subdirectory(cliutils)
add_subdirectory(ethernet)