#pragma once

// TODO: clean this up. it's for interop with the panda version
#ifndef CANPACKET_HEAD_SIZE

#include "opendbc/safety/board/can_declarations.h"

static const unsigned char dlc_to_len[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

#endif

// Define helper macros if not already defined
#ifndef GET_BUS
#define GET_BUS(msg) ((msg)->bus)
#endif

#ifndef GET_ADDR
#define GET_ADDR(msg) ((msg)->addr)
#endif

#ifndef GET_BYTE
#define GET_BYTE(msg, idx) ((msg)->data[idx])
#endif

#ifndef GET_LEN
#define GET_LEN(msg) (dlc_to_len[(msg)->data_len_code])
#endif
