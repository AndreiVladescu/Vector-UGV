/*
 * Six leg cells running the real leg logic against simulated servos, on a SocketCAN
 * interface. Stands in for the side boards:
 *
 *   sim_legs vcan0                 legs start uncalibrated, like fresh boards
 *   sim_legs vcan0 --calibrated    legs already know their servos
 *   --limits c0,c1,f0,f1,t0,t1     joint limits in degrees (from legs.yaml); each joint's
 *                                  centre is set so the middle of its range is at 1500 us
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
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

#include "leg_sim.h"

#define LEGS 6

static volatile sig_atomic_t running = 1;
static int sock = -1;
static unsigned long tx_count, rx_count;

static void stop(int sig) { (void)sig; running = 0; }

static void tx(void *user, const struct can_frame_t *f)
{
    (void)user;
    struct can_frame cf = {.can_id = f->id, .len = f->len};
    memcpy(cf.data, f->data, f->len);
    if (write(sock, &cf, sizeof(cf)) == sizeof(cf))
        tx_count++;
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

static float spread(unsigned *seed, float lo, float hi)
{
    *seed = *seed * 1103515245u + 12345u;
    return lo + (hi - lo) * ((*seed >> 8) & 0xffff) / 65535.0f;
}

int main(int argc, char **argv)
{
    const char *ifname = "vcan0";
    bool calibrated = false;
    float lim[6] = {-80, 80, -80, 80, -80, 80};
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--calibrated"))
            calibrated = true;
        else if (!strcmp(argv[i], "--limits") && i + 1 < argc &&
                 sscanf(argv[++i], "%f,%f,%f,%f,%f,%f", &lim[0], &lim[1], &lim[2], &lim[3], &lim[4], &lim[5]) == 6)
            ;
        else if (argv[i][0] != '-')
            ifname = argv[i];
        else {
            fprintf(stderr, "usage: %s [ifname] [--calibrated] [--limits c0,c1,f0,f1,t0,t1]\n", argv[0]);
            return 2;
        }
    }
    if ((sock = open_can(ifname)) < 0)
        return 1;
    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    static struct leg_sim legs[LEGS];
    unsigned seed = 7;
    for (int n = 0; n < LEGS; n++) {
        struct servo_sim servo[JOINTS];
        struct leg_config cfg;
        config_defaults(&cfg);
        for (int j = 0; j < JOINTS; j++) {
            /* spread like the real 18 (docs/servos.md) */
            servo_sim_init(&servo[j], spread(&seed, 1522, 1646), spread(&seed, 1.366f, 1.489f), 1500);
            cfg.joint[j].min_deg = lim[2 * j];
            cfg.joint[j].max_deg = lim[2 * j + 1];
            cfg.joint[j].center_us = 1500.0f - (lim[2 * j] + lim[2 * j + 1]) / 2 * cfg.joint[j].us_per_deg;
            if (calibrated) {
                cfg.joint[j].mid_mv = servo[j].mid_mv;
                cfg.joint[j].slope = servo[j].slope;
                cfg.joint[j].calibrated = true;
            }
        }
        config_seal(&cfg);
        leg_sim_init(&legs[n], n + 1, &cfg);
        memcpy(legs[n].servo, servo, sizeof(servo));
        legs[n].tx = tx;
    }
    printf("sim_legs on %s, 6 legs, %s\n", ifname, calibrated ? "calibrated" : "uncalibrated");
    fflush(stdout);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    unsigned long ticks = 0;
    while (running) {
        struct can_frame cf;
        while (read(sock, &cf, sizeof(cf)) == sizeof(cf)) {
            if (cf.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG))
                continue;
            struct can_frame_t f = {.id = cf.can_id & CAN_SFF_MASK, .len = cf.len};
            memcpy(f.data, cf.data, cf.len);
            rx_count++;
            for (int n = 0; n < LEGS; n++)
                leg_frame(&legs[n].leg, &f);
        }
        for (int n = 0; n < LEGS; n++)
            leg_sim_tick(&legs[n]);

        if (++ticks % 2000 == 0) {
            printf("rx %4lu/s tx %4lu/s  states", rx_count / 2, tx_count / 2);
            for (int n = 0; n < LEGS; n++)
                printf(" %d:%d/%02x", n + 1, legs[n].leg.state, legs[n].leg.faults);
            printf("\n");
            fflush(stdout);
            rx_count = tx_count = 0;
        }

        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    close(sock);
    return 0;
}
