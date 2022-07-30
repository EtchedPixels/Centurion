/********************************************************************
 * qofmath128.h -- an 128-bit integer library                       *
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

#ifndef QOF_MATH_128_H
#define QOF_MATH_128_H


typedef struct
{
    uint64_t hi;
    uint64_t lo;
    short isneg;    
    short isbig;    
} qofint128;

extern unsigned equal128 (qofint128 a, qofint128 b);
extern int cmp128 (qofint128 a, qofint128 b);
extern qofint128 shift128 (qofint128 x);
extern qofint128 shiftleft128 (qofint128 x);
extern qofint128 inc128 (qofint128 a);
extern qofint128 add128 (qofint128 a, qofint128 b);
extern qofint128 mult128 (int64_t a, int64_t b);
extern qofint128 div128 (qofint128 n, int64_t d);
extern int64_t rem128 (qofint128 n, int64_t d);
extern uint64_t gcf64(uint64_t num, uint64_t denom);
extern qofint128 lcm128 (uint64_t a, uint64_t b);

#endif
