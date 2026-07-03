/* Host-test stub, not part of the firmware build.
   Native (gcc) unit tests link real Framework/Business .c files, which
   include the real FreeRTOS.h for critical sections. There is no RTOS on
   the host and the tests are single-threaded, so this stub shadows the
   real Third_Party FreeRTOS headers (put -I Tests/Stub before -I Framework/Inc)
   and collapses critical sections to no-ops via task.h. */
#ifndef FREERTOS_STUB_H
#define FREERTOS_STUB_H
#endif
