#include "filter.h"

uint16_t median_u16(uint16_t *buf, int n)
{
    for (int i = 1; i < n; i++) {
        uint16_t v = buf[i];
        int k = i - 1;
        while (k >= 0 && buf[k] > v) {
            buf[k + 1] = buf[k];
            k--;
        }
        buf[k + 1] = v;
    }
    return n & 1 ? buf[n / 2] : (uint16_t)((buf[n / 2 - 1] + buf[n / 2]) / 2);
}

void linfit_reset(struct linfit *f)
{
    f->n = f->sx = f->sy = f->sxx = f->sxy = 0;
}

void linfit_add(struct linfit *f, double x, double y)
{
    f->n += 1;
    f->sx += x;
    f->sy += y;
    f->sxx += x * x;
    f->sxy += x * y;
}

int linfit_solve(const struct linfit *f, double *a, double *b)
{
    double d = f->n * f->sxx - f->sx * f->sx;
    if (f->n < 2 || d == 0)
        return 0;
    *b = (f->n * f->sxy - f->sx * f->sy) / d;
    *a = (f->sy - *b * f->sx) / f->n;
    return 1;
}
