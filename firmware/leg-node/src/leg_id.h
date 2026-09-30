#ifndef LEG_ID_H
#define LEG_ID_H

#include <stdint.h>

/* Node ID from the ID jumpers, side << 2 | position (see stm32/board.c).
   Position 1 is the cell next to the CAN-in connector, 3 the outer end. The right board is
   the same PCB turned 180°, so its position 1 sits at the other end of the body: it counts
   backwards there, and R1 is the front leg on both sides (legs.yaml).
   Left 1, 2, 3 -> L1–L3 = 1–3; right 1, 2, 3 -> R3, R2, R1 = 6, 5, 4. 0: position not set. */
static inline uint8_t leg_node_from_straps(uint8_t straps)
{
    uint8_t pos = straps & 3;
    if (!pos)
        return 0;
    return (straps & 4) ? (uint8_t)(7 - pos) : pos;
}

#endif
