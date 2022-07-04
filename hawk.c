
#include "hawk.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void hawk_seek(struct hawk_unit* unit, unsigned cyl, unsigned head, unsigned sec)
{
    // The hawk unit only has 9 lines for cylinder addr, so address really
    // should get masked.
    // OR, is DSK expected to throw an error before seeking?

    // In reality, the hawk unit doesn't receive the sector, it only seeks
    // to a cylinder and selects the correct head.
    // It then streams out raw bits off the disk, along with pluses for
    // sector/track begin and sector index
    // The controller is expected to wait for it's request sector, verify
    // the address marker and then catch the data.
    //
    // But we simplify things slightly.

	unit->on_cyl = 0;
	unit->addr_ack = 0;
	unit->addr_int = 0;
    unit->current_track = cyl << 1 | head;

    unsigned offset = (cyl << 5) | (head << 4) | sec;
    offset *= HAWK_SECTOR_SIZE;

    if (cyl >= HAWK_NUM_CYLINDERS) {
        // Tried to seek past end of disk
        unit->addr_int = 1;
        return;
    }

    if (lseek(unit->fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "hawk position failed (%d,%d,%d) = %lx.\n",
            cyl, head, sec, (long) offset);

        // Lets treat this as some kind of seek error
        unit->seek_error = 1;
    } else {
        // Successful seek
        unit->on_cyl = 1;
        unit->addr_ack = 1;
    }
}

void hawk_rtz(struct hawk_unit* unit)
{
    // According to documentation, The Hawk drive unit will clear any seek
    // errors and faults on RTZS
    unit->seek_error = 0;
    unit->fault = 0;

    hawk_seek(unit, 0, 0, 0);
}