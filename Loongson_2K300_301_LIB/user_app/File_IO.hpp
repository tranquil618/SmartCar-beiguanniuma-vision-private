#ifndef File_IO_HPP
#define File_IO_HPP

#include <stdint.h>


int8_t file_read_string(const char *path, char *str);
int8_t file_write_string(const char *path, char *str);

#endif
