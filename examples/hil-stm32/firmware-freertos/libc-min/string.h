#ifndef LIBC_MIN_STRING_H
#define LIBC_MIN_STRING_H
#include <stddef.h>
void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);
char *strcpy(char *dst, const char *src);
size_t strlen(const char *s);
#endif
