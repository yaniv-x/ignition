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

#ifndef _H_DEFS
#define _H_DEFS

#define IO_POST_CODE 0x80
#define IO_PORT_A20 0x92

#define POST_CODE_START16   1
#define POST_CODE_INIT16    2
#define POST_CODE_START32   3
#define POST_CODE_INIT32    4

#define NULL 0

#define CR0_PE (1 << 0)

#define SD_CS (0x18 << 8)
#define SD_DS (0x12 << 8)
#define SD_PRESENT (1 << 15)
#define SD_32 (1 << 22)
#define SD_4K (1 << 23)
#define SD_HLIMIT_SHIFT 16

#define CODE_SEGMENT_SELECTOR (1 << 3)
#define DATA_SEGMENT_SELECTOR (2 << 3)

#define PROTECTED_START_ADDRESS ((1024 - 128) * 1024)

#endif

