remake_find_library(tulibs-timer timer.h PACKAGE tulibs)
remake_find_library(tulibs-serial serial.h PACKAGE tulibs)

remake_add_executables(LINK ${TULIBS_TIMER_LIBRARY} ${TULIBS_SERIAL_LIBRARY})
