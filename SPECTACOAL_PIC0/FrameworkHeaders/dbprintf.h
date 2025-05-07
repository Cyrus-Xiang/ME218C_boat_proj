#include "ES_Port.h"
void DB_printf(const char *Format, ...);

// Note: these definitions are for a little Endian processor
//#define LOWORD(l) (*((unsigned int *)(&l)))
//#define HIWORD(l) (*(((unsigned int *)(&l))+1))

#define printf    DB_printf