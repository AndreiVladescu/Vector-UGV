#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* Median of n values (n <= 64). Sorts buf in place. */
uint16_t median_u16(uint16_t *buf, int n);

struct linfit {
    double n, sx, sy, sxx, sxy;
};

void linfit_reset(struct linfit *f);
void linfit_add(struct linfit *f, double x, double y);
/* y = a + b * x. Returns 0 if the fit is degenerate. */
int linfit_solve(const struct linfit *f, double *a, double *b);

#endif
