/********************************************************************
 * qofmath128.c -- an 128-bit integer library                       *
 * Copyright (C) 2004 Linas Vepstas <linas@linas.org>               *
 *                                                                  *
 * This program is free software; you can redistribute it and/or    *
 * modify it under the terms of the GNU General Public License as   *
 * published by the Free Software Foundation; either version 2 of   *
 * the License, or (at your option) any later version.              *
 *                                                                  *
 * This program is distributed in the hope that it will be useful,  *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of   *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the    *
 * GNU General Public License for more details.                     *
 *                                                                  *
 * You should have received a copy of the GNU General Public License*
 * along with this program; if not, contact:                        *
 *                                                                  *
 * Free Software Foundation           Voice:  +1-617-542-5942       *
 * 51 Franklin Street, Fifth Floor    Fax:    +1-617-542-2652       *
 * Boston, MA  02110-1301,  USA       gnu@gnu.org                   *
 *                                                                  *
 *******************************************************************/

#include <stdint.h>
#include "math128.h"

/* =============================================================== */
/*
 *  Quick-n-dirty 128-bit integer math lib.   Things seem to mostly
 *  work, and have been tested, but not comprehensively tested.
 */

#define HIBIT (0x8000000000000000ULL)

qofint128
mult128 (int64_t a, int64_t b)
{
    qofint128 prod;
    uint64_t a0, a1;
    uint64_t b0, b1;
    uint64_t d, d0, d1;
    uint64_t e, e0, e1;
    uint64_t f, f0, f1;
    uint64_t g, g0, g1;
    uint64_t sum, carry, roll, pmax;

    prod.isneg = 0;
    if (0 > a)
    {
        prod.isneg = !prod.isneg;
        a = -a;
    }

    if (0 > b)
    {
        prod.isneg = !prod.isneg;
        b = -b;
    }

    a1 = a >> 32;
    a0 = a - (a1 << 32);

    b1 = b >> 32;
    b0 = b - (b1 << 32);

    d = a0 * b0;
    d1 = d >> 32;
    d0 = d - (d1 << 32);

    e = a0 * b1;
    e1 = e >> 32;
    e0 = e - (e1 << 32);

    f = a1 * b0;
    f1 = f >> 32;
    f0 = f - (f1 << 32);

    g = a1 * b1;
    g1 = g >> 32;
    g0 = g - (g1 << 32);

    sum = d1 + e0 + f0;
    carry = 0;
    /* Can't say 1<<32 cause cpp will goof it up; 1ULL<<32 might work */
    roll = 1 << 30;
    roll <<= 2;

    pmax = roll - 1;
    while (pmax < sum)
    {
        sum -= roll;
        carry ++;
    }

    prod.lo = d0 + (sum << 32);
    prod.hi = carry + e1 + f1 + g0 + (g1 << 32);
    // prod.isbig = (prod.hi || (sum >> 31));
    prod.isbig = prod.hi || (prod.lo >> 63);

    return prod;
}

qofint128
shift128 (qofint128 x)
{
    uint64_t sbit = x.hi & 0x1;
    x.hi >>= 1;
    x.lo >>= 1;
    x.isbig = 0;
    if (sbit)
    {
        x.lo |= HIBIT;
        x.isbig = 1;
        return x;
    }
    if (x.hi)
    {
        x.isbig = 1;
    }
    return x;
}

qofint128
shiftleft128 (qofint128 x)
{
    uint64_t sbit;
    sbit = x.lo & HIBIT;
    x.hi <<= 1;
    x.lo <<= 1;
    x.isbig = 0;
    if (sbit)
    {
        x.hi |= 1;
        x.isbig = 1;
        return x;
    }
    if (x.hi)
    {
        x.isbig = 1;
    }
    return x;
}

qofint128
inc128 (qofint128 a)
{
    if (0 == a.isneg)
    {
        a.lo ++;
        if (0 == a.lo)
        {
            a.hi ++;
        }
    }
    else
    {
        if (0 == a.lo)
        {
            a.hi --;
        }
        a.lo --;
    }

    a.isbig = (a.hi != 0) || (a.lo >> 63);
    return a;
}

qofint128
div128 (qofint128 n, int64_t d)
{
    qofint128 quotient;
    int i;
    uint64_t remainder = 0;

    quotient = n;
    if (0 > d)
    {
        d = -d;
        quotient.isneg = !quotient.isneg;
    }

    /* Use grade-school long division algorithm */
    for (i = 0; i < 128; i++)
    {
        uint64_t sbit = HIBIT & quotient.hi;
        remainder <<= 1;
        if (sbit) remainder |= 1;
        quotient = shiftleft128 (quotient);
        if (remainder >= d)
        {
            remainder -= d;
            quotient.lo |= 1;
        }
    }

    /* compute the carry situation */
    quotient.isbig = (quotient.hi || (quotient.lo >> 63));

    return quotient;
}

int64_t
rem128 (qofint128 n, int64_t d)
{
    qofint128 quotient = div128 (n, d);

    qofint128 mu = mult128 (quotient.lo, d);

    int64_t nn = 0x7fffffffffffffffULL & n.lo;
    int64_t rr = 0x7fffffffffffffffULL & mu.lo;
    return nn - rr;
}

unsigned
equal128 (qofint128 a, qofint128 b)
{
    if (a.lo != b.lo) return 0;
    if (a.hi != b.hi) return 0;
    if (a.isneg != b.isneg) return 0;
    return 1;
}

int
cmp128 (qofint128 a, qofint128 b)
{
    if ((0 == a.isneg) && b.isneg) return 1;
    if (a.isneg && (0 == b.isneg)) return -1;
    if (0 == a.isneg)
    {
        if (a.hi > b.hi) return 1;
        if (a.hi < b.hi) return -1;
        if (a.lo > b.lo) return 1;
        if (a.lo < b.lo) return -1;
        return 0;
    }

    if (a.hi > b.hi) return -1;
    if (a.hi < b.hi) return 1;
    if (a.lo > b.lo) return -1;
    if (a.lo < b.lo) return 1;
    return 0;
}

uint64_t
gcf64(uint64_t num, uint64_t denom)
{
    uint64_t   t;

    t =  num % denom;
    num = denom;
    denom = t;

    /* The strategy is to use Euclid's algorithm */
    while (0 != denom)
    {
        t = num % denom;
        num = denom;
        denom = t;
    }
    /* num now holds the GCD (Greatest Common Divisor) */
    return num;
}

qofint128
lcm128 (uint64_t a, uint64_t b)
{
    uint64_t gcf = gcf64 (a, b);
    b /= gcf;
    return mult128 (a, b);
}

qofint128
add128 (qofint128 a, qofint128 b)
{
    qofint128 sum;
    if (a.isneg == b.isneg)
    {
        sum.isneg = a.isneg;
        sum.hi = a.hi + b.hi;
        sum.lo = a.lo + b.lo;
        if ((sum.lo < a.lo) || (sum.lo < b.lo))
        {
            sum.hi ++;
        }
        sum.isbig = sum.hi || (sum.lo >> 63);
        return sum;
    }
    if ((b.hi > a.hi) ||
            ((b.hi == a.hi) && (b.lo > a.lo)))
    {
        qofint128 tmp = a;
        a = b;
        b = tmp;
    }

    sum.isneg = a.isneg;
    sum.hi = a.hi - b.hi;
    sum.lo = a.lo - b.lo;

    if (sum.lo > a.lo)
    {
        sum.hi --;
    }

    sum.isbig = sum.hi || (sum.lo >> 63);
    return sum;
}


#ifdef TEST_128_BIT_MULT

static void pr (int64_t a, int64_t b)
{
    qofint128 prod = mult128 (a, b);
    printf ("%" G_GINT64_FORMAT " * %" G_GINT64_FORMAT " = %"
            G_GUINT64_FORMAT " %" G_GUINT64_FORMAT " (0x%"
            G_GINT64_MODIFIER "x %" G_GINT64_MODIFIER "x) %hd\n",
            a, b, prod.hi, prod.lo, prod.hi, prod.lo, prod.isbig);
}

static void prd (int64_t a, int64_t b, int64_t c)
{
    qofint128 prod = mult128 (a, b);
    qofint128 quot = div128 (prod, c);
    int64_t rem = rem128 (prod, c);
    printf ("%" G_GINT64_FORMAT " * %" G_GINT64_FORMAT " / %" G_GINT64_FORMAT
            " = %" G_GUINT64_FORMAT " %" G_GUINT64_FORMAT " + %"
            G_GINT64_FORMAT " (0x%" G_GINT64_MODIFIER "x %"
            G_GINT64_MODIFIER "x) %hd\n",
            a, b, c, quot.hi, quot.lo, rem, quot.hi, quot.lo, quot.isbig);
}

int main (int argc, char *argv[])
{
    int64_t x;
    qofint128 n;
    int64_t d;
    qofint128 quot;
    int i;

    pr (2, 2);

    x = 1 << 30;
    x <<= 2;

    pr (x, x);
    pr (x + 1, x);
    pr (x + 1, x + 1);

    pr (x, -x);
    pr (-x, -x);
    pr (x - 1, x);
    pr (x - 1, x - 1);
    pr (x - 2, x - 2);

    x <<= 1;
    pr (x, x);
    pr (x, -x);

    pr (1000000, G_GINT64_CONSTANT(10000000000000));

    prd (x, x, 2);
    prd (x, x, 3);
    prd (x, x, 4);
    prd (x, x, 5);
    prd (x, x, 6);

    x <<= 29;
    prd (3, x, 3);
    prd (6, x, 3);
    prd (99, x, 3);
    prd (100, x, 5);
    prd (540, x, 5);
    prd (777, x, 7);
    prd (1111, x, 11);

    /* Really test division */
    n.hi = 0xdd91;
    n.lo = 0x6c5abefbb9e13480ULL;

    d = 0x2ae79964d3ae1d04ULL;

    for (i = 0; i < 20; i++)
    {

        quot = div128 (n, d);
        printf ("%d result = %" G_GINT64_MODIFIER "x %" G_GINT64_MODIFIER "x\n",
                i, quot.hi, quot.lo);
        d >>= 1;
        n = shift128 (n);
    }
    return 0;
}

#endif /* TEST_128_BIT_MULT */

