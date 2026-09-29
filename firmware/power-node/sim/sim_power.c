/*
 * The power board's real logic against the chip models, with a battery that drains and
 * charges, on a SocketCAN interface. Stands in for the power board next to sim_legs:
 *
 *   sim_power vcan0                    on, 70 %, no charger
 *   sim_power vcan0 --soc 12 --load-ma 4000
 *
 * The simulated CM5 "halts" --halt-s seconds after the board asks it to. SIGUSR1 gives the
 * power button a short press (turns a charging board on), SIGUSR2 plugs or unplugs the
 * charger. The program ends when the board has switched itself off.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <linux/can.h>
#include <net/if.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "power_sim.h"

static volatile sig_atomic_t running = 1, press = 0, plug = 0;
static int sock = -1;

static void stop(int sig) { (void)sig; running = 0; }
static void on_usr1(int sig) { (void)sig; press = 1; }
static void on_usr2(int sig) { (void)sig; plug = 1; }

static void tx(void *user, const struct can_frame_t *f)
{
    (void)user;
    struct can_frame cf = {.can_id = f->id, .len = f->len};
    memcpy(cf.data, f->data, f->len);
    if (write(sock, &cf, sizeof(cf)) != sizeof(cf)) {
        /* bus full: the next frame goes out */
    }
}

static int open_can(const char *ifname)
{
    int s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    struct ifreq ifr = {0};
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (s < 0 || ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "%s: %s\n", ifname, strerror(errno));
        return -1;
    }
    struct sockaddr_can addr = {.can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex};
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "bind %s: %s\n", ifname, strerror(errno));
        return -1;
    }
    return s;
}

/* resting cell voltage for a state of charge, the inverse of the gauge's table */
static float ocv_mv(float soc)
{
    float lo = 3000, hi = 4200;
    for (int i = 0; i < 30; i++) {
        float mid = (lo + hi) / 2;
        if (gauge_ocv_soc((uint16_t)mid) < soc)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

struct battery {
    float soc, capacity_mah, load_ma;
};

/* every 100 ms: net current from what the board has switched on, cells from SoC and I*R */
static void battery_step(struct battery *b, struct power_sim *s)
{
    uint8_t fets = bq76942_sim_fets(&s->bms);
    float load = 0, charge = 0;
    if (fets & BQ_FET_DSG) {
        if (!s->outs[OUT_5V_OFF])
            load += 400; /* CM5, camera, radios at pack voltage */
        load += (!s->outs[OUT_SIDE_L_OFF] + !s->outs[OUT_SIDE_R_OFF]) * b->load_ma / 2;
        load += 30;
    }
    if ((fets & BQ_FET_CHG) && (s->chg.vac1 || s->chg.vac2) && (s->chg.regs[CHG_CTRL0] & 0x20)) {
        charge = bq25798_sim_reg16(&s->chg, CHG_ICHG) * 10.0f;
        if (b->soc > 95)
            charge *= (100 - b->soc) / 5; /* constant-voltage taper */
        s->chg.done = b->soc >= 99.5f;
        if (s->chg.done)
            charge = 0;
    }
    float net = charge - load;
    b->soc += net * 0.1f / 3600.0f / b->capacity_mah * 100.0f;
    b->soc = b->soc < 0 ? 0 : b->soc > 100 ? 100 : b->soc;
    s->bms.current_ma = (int16_t)net;
    bq76942_sim_cells(&s->bms, (uint16_t)(ocv_mv(b->soc) + net * 0.01f)); /* 10 mΩ per 3P group */
    s->chg.vbat_mv = (uint16_t)(4 * ocv_mv(b->soc));
    s->chg.vac_mv = 20000;
}

int main(int argc, char **argv)
{
    const char *ifname = "vcan0";
    struct battery bat = {.soc = 70, .capacity_mah = 8000, .load_ma = 3000};
    float halt_s = 3;
    bool charger = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--soc") && i + 1 < argc)
            bat.soc = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--load-ma") && i + 1 < argc)
            bat.load_ma = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--halt-s") && i + 1 < argc)
            halt_s = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--charger"))
            charger = true;
        else if (argv[i][0] != '-')
            ifname = argv[i];
        else {
            fprintf(stderr, "usage: %s [ifname] [--soc %%] [--load-ma mA] [--halt-s s] [--charger]\n", argv[0]);
            return 2;
        }
    }
    if ((sock = open_can(ifname)) < 0)
        return 1;
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    signal(SIGUSR1, on_usr1);
    signal(SIGUSR2, on_usr2);

    static struct power_sim sim;
    power_sim_init(&sim, true);
    sim.tx = tx;
    sim.chg.vac1 = charger;
    /* the saved SoC is what the gauge will trust, as after a normal power-off */
    config_defaults(&sim.flash[0]);
    sim.flash[0].soc = bat.soc;
    config_seal(&sim.flash[0]);
    battery_step(&bat, &sim);
    sim.button = true; /* woken by the power button */
    power_sim_boot(&sim);
    printf("sim_power on %s, %.0f %%, %s\n", ifname, bat.soc, charger ? "charger in" : "no charger");
    fflush(stdout);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    unsigned long ticks = 0, press_until = 0, halting_since = 0;
    enum power_state last = POWER_BOOT;
    while (running && power_sim_mcu_on(&sim)) {
        struct can_frame cf;
        while (read(sock, &cf, sizeof(cf)) == sizeof(cf)) {
            if (cf.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG))
                continue;
            struct can_frame_t f = {.id = cf.can_id & CAN_SFF_MASK, .len = cf.len};
            memcpy(f.data, cf.data, cf.len);
            power_frame(&sim.p, &f);
        }
        if (ticks == 300)
            sim.button = false;
        if (press) {
            press = 0;
            sim.button = true;
            press_until = ticks + 200;
        }
        if (press_until && ticks >= press_until) {
            sim.button = false;
            press_until = 0;
        }
        if (plug) {
            plug = 0;
            sim.chg.vac1 = !sim.chg.vac1;
            printf("charger %s\n", sim.chg.vac1 ? "plugged in" : "unplugged");
        }
        if (sim.p.state == POWER_HALTING) {
            if (!halting_since)
                halting_since = ticks;
            sim.halted = ticks - halting_since >= halt_s * 1000;
        } else {
            halting_since = 0;
            sim.halted = false;
        }
        if (ticks % 100 == 0)
            battery_step(&bat, &sim);
        power_sim_run(&sim, 1);

        if (sim.p.state != last) {
            printf("state %d -> %d\n", last, sim.p.state);
            last = sim.p.state;
        }
        if (++ticks % 10000 == 0)
            printf("%.1f %% %.2f V %+.2f A state %d faults %02x flags %x\n", bat.soc, sim.p.bat.stack_mv / 1000.0,
                   sim.bms.current_ma / 1000.0, sim.p.state, sim.p.faults, sim.p.flags);
        fflush(stdout);

        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    if (!power_sim_mcu_on(&sim))
        printf("power board off\n");
    close(sock);
    return 0;
}
