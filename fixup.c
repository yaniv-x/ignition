/*
    Copyright (c) 2013 Yaniv Kamay,
    All rights reserved.

    Source code is provided for evaluation purposes only. Modification or use in
    source code for any other purpose is prohibited.

    Binary code (i.e. the binary form of source code form) is allowed to use for
    evaluation purposes only. Modification or use in binary code for any other
    purpose is prohibited.

    Redistribution, in source form or in binary form, with or without modification,
    are not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTOR BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>


const char* prog_name;
int bin_fd;
struct stat bin_stat;
char* map;


#define ERROR(format, ...) {                                        \
    fprintf(stderr, "%s: " format "\n", prog_name, ## __VA_ARGS__); \
    exit(-1);                                                       \
}


static char* skip_white(char* str)
{
    while (*str && isblank(*str)) {
        str++;
    }

    return str;
}


static void trunc_white(char* str)
{
    char* end = str + strlen(str) - 1;

    while (end >= str && isblank(*end)) {
        *end =0;
        end--;
    }
}


static char* clean_white(char* str)
{
    str = skip_white(str);
    trunc_white(str);
    return str;
}


static uint get_unsigned(char* str, char** end_ptr)
{
    unsigned long num = strtoul(str, end_ptr, 0);

    if (*end_ptr == str || errno || num > UINT_MAX) {
        ERROR("invalid number str \"%s\"", str);
    }

    *end_ptr = skip_white(*end_ptr);

    return num;
}


static int8_t checksum8(void *start, uint size)
{
    uint8_t res = 0;
    uint8_t *now = (uint8_t*)start;
    uint8_t *end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}


static void read_all(int fd, off_t from, void* in_dest, uint size)
{
    uint8_t* dest = (uint8_t*)in_dest;

    if (lseek(fd, from, SEEK_SET) != from) {
        ERROR("seek failed");
    }

    while (size) {
        ssize_t n = read(fd, dest, size);
        if (n <= 0) {
             if (n == 0 || errno != EINTR) {
                 ERROR("read failed");
             }
             continue;
        }

        size -= n;
        dest += n;
    }
}


static void write_all(int fd, off_t to, void* in_src, uint size)
{
    uint8_t* src = (uint8_t*)in_src;

    if (lseek(fd, to, SEEK_SET) != to) {
        ERROR("seek failed");
    }

    while (size) {
        ssize_t n = write(fd, src, size);
        if (n == -1) {
             if (errno != EINTR) {
                 ERROR("write failed");
             }
             continue;
        }

        size -= n;
        src += n;
    }
}


static void do_fixup(const char* symbol, uint size, uint sum_offset)
{
    const char *pos = map;
    unsigned long address;

    for (;;) {
        const char* address_str;
        char* end_ptr;

        pos = strstr(pos, symbol);

        if (!pos) {
            ERROR("symbol \"%s\" not found", symbol);
        }

        if (pos[strlen(symbol)] != '\n') {
            pos++;
            continue;
        }

        for (address_str = pos; *(address_str - 1) != '\n'; --address_str);

        address = strtoul(address_str, &end_ptr, 16);

        if (end_ptr == address_str || errno || address > UINT_MAX) {
            pos++;
            continue;
        }

        if (*end_ptr == '+' || *end_ptr == '*') {
            ++end_ptr;
        }

        end_ptr = skip_white(end_ptr);


        if (end_ptr != pos) {
            pos++;
            continue;
        }

        break;
    }

    address -= 0xe0000;

    if (address + size > bin_stat.st_size) {
        ERROR("size 0x%x if to large fo symbol \"%s\" @0x%lx", size, symbol, address);
    }

    uint8_t * buf = malloc(size);

    if (!buf) {
        ERROR("alloc 0x%lx for sum buff failed", bin_stat.st_size);
    }

    read_all(bin_fd, address, buf, size);
    int8_t invers = checksum8(buf, size);
    free(buf);

    write_all(bin_fd, address + sum_offset, &invers, 1);
}


static void init_bin(const char *file_name)
{
    bin_fd = open(file_name, O_RDWR);

    if (bin_fd == -1) {
        ERROR("open file \"%s\" failed", file_name);
    }


    if (fstat(bin_fd, &bin_stat) == -1) {
        ERROR("fstat file \"%s\" failed", file_name);
    }
}


static void init_map(const char *file_name)
{
    int map_fd = open(file_name, O_RDWR);
    struct stat stat;

    map_fd = open(file_name, O_RDWR);

    if (map_fd == -1) {
        ERROR("open file \"%s\" failed", file_name);
    }

    if (fstat(map_fd, &stat) == -1) {
        ERROR("fstat file \"%s\" failed", file_name);
    }

    map = malloc(stat.st_size + 3);

    if (!map) {
        ERROR("alloc map 0x%lx failed", stat.st_size);
    }

    read_all(map_fd, 0, map + 1, stat.st_size);

    map[0] = '\n';
    map[stat.st_size] = '\n';
    map[stat.st_size + 1] = 0;
}


int main(int argc, const char** argv)
{
    const char** argv_end = argv + argc;
    prog_name = *argv++;

    if (argc < 3) {
        ERROR("missing arguments");
    }

    init_bin(*argv++);
    init_map(*argv++);

    for (; argv < argv_end; argv++) {
        char *str = strdup(*argv);
        char* end = strchr(str, ',');

        if (!end) {
            ERROR("invalid fixup str \"%s\"", *argv);
        }

        *end = 0;

        char* symbol = clean_white(str);

        char* num_arg = end + 1;
        char *end_ptr;

        uint size = get_unsigned(num_arg, &end_ptr);
        num_arg = end_ptr;

        if (*num_arg != ',') {
            ERROR("invalid fixup str \"%s\"", *argv);
        }

        uint offset = get_unsigned(num_arg + 1, &end_ptr);

        if (strlen(end_ptr)) {
            ERROR("invalid fixup str \"%s\"", *argv);
        }

        if (offset >= size) {
            ERROR("invalid  offset \"%s\"", *argv);
        }

        do_fixup(symbol, size, offset);

        free(str);
    }

    return 0;
}

