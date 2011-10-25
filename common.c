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

static inline void outb(uint16_t port, uint8_t val)
{
    _asm {
        mov dx, port
        mov al, val
        out dx, al
    }
}

static inline void outw(uint16_t port, uint16_t val)
{
    _asm {
        mov dx, port
        mov ax, val
        out dx, ax
    }
}


static inline uint8_t inb(uint16_t port)
{
    uint8_t val;

    _asm {
        mov dx, port
        in al, dx
        mov val, al
    }

    return val;
}

static inline uint16_t inw(uint16_t port)
{
    uint16_t val;

    _asm {
        mov dx, port
        in ax, dx
        mov val, ax
    }

    return val;
}


static inline void post(uint8_t code)
{
    outb(IO_PORT_POST_CODE, code);
}


static inline void restart()
{
    outb(IO_PORT_SYSCTRL, 1 << SYSCTRL_RESET_BIT);
}



static int find_lsb_32(uint32_t val)
{
    int i;

    for (i = 0; i < 32; i++) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return -1;
}


static int find_msb_32(uint32_t val)
{
    int i;

    for (i = 31; i >=0; i--) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return i;
}


static int find_lsb_64(uint64_t val)
{
    int i;

    for (i = 0; i < 64; i++) {
        if (val & (1ULL << i)) {
            return i;
        }
    }

    return -1;
}

