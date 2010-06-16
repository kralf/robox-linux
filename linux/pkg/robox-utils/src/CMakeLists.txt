remake_find_library(tulibs-timer timer.h PACKAGE Tulibs)
remake_find_library(tulibs-serial serial.h PACKAGE Tulibs)

remake_add_executables(LINK ${TULIBS_TIMER_LIBRARY} ${TULIBS_SERIAL_LIBRARY})
