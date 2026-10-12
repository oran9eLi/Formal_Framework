#ifndef M2_TEST_FF_H
#define M2_TEST_FF_H
#include <stdint.h>
typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef unsigned long FSIZE_t;
typedef struct { int unused; } FATFS;
typedef struct { FSIZE_t size; } FIL;
typedef enum { FR_OK, FR_DISK_ERR, FR_INT_ERR, FR_NOT_READY, FR_NO_FILE, FR_NO_PATH, FR_INVALID_NAME, FR_DENIED } FRESULT;
#define FA_OPEN_ALWAYS 0x10
#define FA_WRITE 2
#define f_size(f) ((f)->size)
FRESULT f_mount(FATFS *, const char *, BYTE);
FRESULT f_open(FIL *, const char *, BYTE);
FRESULT f_close(FIL *);
FRESULT f_sync(FIL *);
FRESULT f_lseek(FIL *, FSIZE_t);
FRESULT f_write(FIL *, const void *, UINT, UINT *);
#endif
