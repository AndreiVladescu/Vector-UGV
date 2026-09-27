#include "vector_can.h"

#include <math.h>
#include <string.h>

static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }

static void put32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++)
        p[i] = (v >> (8 * i)) & 0xff;
}

static uint32_t get32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int16_t centideg(float deg)
{
    long v = lroundf(deg * 100.0f);
    if (v > 18000) v = 18000;
    if (v < -18000) v = -18000;
    return (int16_t)v;
}

static void frame(struct can_frame_t *f, uint32_t id, uint8_t len)
{
    memset(f, 0, sizeof(*f));
    f->id = id;
    f->len = len;
}

void can_pack_sync(struct can_frame_t *f, const struct sync_msg *m)
{
    frame(f, CAN_SYNC, 4);
    put16(f->data, m->counter);
    f->data[2] = (m->mode & 0x0f) | (m->estop ? 0x10 : 0);
}

void can_pack_leg_cmd(struct can_frame_t *f, uint8_t node, const struct leg_cmd_msg *m)
{
    frame(f, CAN_LEG_CMD | node, 8);
    for (int j = 0; j < 3; j++)
        put16(f->data + 2 * j, (uint16_t)centideg(m->deg[j]));
    f->data[6] = m->enable;
    f->data[7] = m->counter;
}

void can_pack_leg_state(struct can_frame_t *f, uint8_t node, const struct leg_state_msg *m)
{
    frame(f, CAN_LEG_STATE | node, 8);
    for (int j = 0; j < 3; j++)
        put16(f->data + 2 * j, (uint16_t)centideg(m->deg[j]));
    put16(f->data + 6, m->current_ma);
}

void can_pack_leg_status(struct can_frame_t *f, uint8_t node, const struct leg_status_msg *m)
{
    uint16_t vbat = m->vbat_mv / 10, rail = m->rail_mv / 10;
    if (vbat > 0xfff) vbat = 0xfff;
    if (rail > 0xfff) rail = 0xfff;
    frame(f, CAN_LEG_STATUS | node, 8);
    put16(f->data, m->tof_mm);
    f->data[2] = vbat & 0xff;
    f->data[3] = (vbat >> 8) | ((rail & 0x0f) << 4);
    f->data[4] = rail >> 4;
    f->data[5] = (uint8_t)m->temp_c;
    f->data[6] = m->faults;
    f->data[7] = m->state & 0x0f;
}

void can_pack_leg_cfg(struct can_frame_t *f, uint32_t function, uint8_t node, const struct leg_cfg_msg *m)
{
    frame(f, function | node, 8);
    f->data[0] = m->joint;
    f->data[1] = m->key;
    put32(f->data + 2, (uint32_t)m->value);
    f->data[6] = m->seq;
    f->data[7] = m->op;
}

bool can_unpack_sync(const struct can_frame_t *f, struct sync_msg *m)
{
    if (f->id != CAN_SYNC || f->len < 3)
        return false;
    m->counter = get16(f->data);
    m->mode = f->data[2] & 0x0f;
    m->estop = f->data[2] & 0x10;
    return true;
}

bool can_unpack_leg_cmd(const struct can_frame_t *f, struct leg_cmd_msg *m)
{
    if (can_function(f->id) != CAN_LEG_CMD || f->len < 8)
        return false;
    for (int j = 0; j < 3; j++)
        m->deg[j] = (int16_t)get16(f->data + 2 * j) / 100.0f;
    m->enable = f->data[6] & 1;
    m->counter = f->data[7];
    return true;
}

bool can_unpack_leg_state(const struct can_frame_t *f, struct leg_state_msg *m)
{
    if (can_function(f->id) != CAN_LEG_STATE || f->len < 8)
        return false;
    for (int j = 0; j < 3; j++)
        m->deg[j] = (int16_t)get16(f->data + 2 * j) / 100.0f;
    m->current_ma = get16(f->data + 6);
    return true;
}

bool can_unpack_leg_status(const struct can_frame_t *f, struct leg_status_msg *m)
{
    if (can_function(f->id) != CAN_LEG_STATUS || f->len < 8)
        return false;
    m->tof_mm = get16(f->data);
    m->vbat_mv = (f->data[2] | ((f->data[3] & 0x0f) << 8)) * 10;
    m->rail_mv = ((f->data[3] >> 4) | (f->data[4] << 4)) * 10;
    m->temp_c = (int8_t)f->data[5];
    m->faults = f->data[6];
    m->state = f->data[7] & 0x0f;
    return true;
}

bool can_unpack_leg_cfg(const struct can_frame_t *f, struct leg_cfg_msg *m)
{
    uint32_t fn = can_function(f->id);
    if ((fn != CAN_LEG_CONFIG && fn != CAN_LEG_REPLY) || f->len < 8)
        return false;
    m->joint = f->data[0];
    m->key = f->data[1];
    m->value = (int32_t)get32(f->data + 2);
    m->seq = f->data[6];
    m->op = f->data[7];
    return true;
}
