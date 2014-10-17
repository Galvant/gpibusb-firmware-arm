/******************************************************************************
 * Simple ringbuffer implementation from open-bldc's libgovernor that
 * you can find at:
 * https://github.com/open-bldc/open-bldc/tree/master/source/libgovernor
 *****************************************************************************/

#include "ring.h"

void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size)
{
    ring->data = buf;
    ring->size = size;
    ring->begin = 0;
    ring->end = 0;
}

int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
    if (((ring->end + 1) % ring->size) != ring->begin) {
        ring->data[ring->end++] = ch;
        ring->end %= ring->size;
        return (uint32_t)ch;
    }

    return -1;
}

int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size)
{
    int32_t i;

    for (i = 0; i < size; i++) {
        if (ring_write_ch(ring, data[i]) < 0)
            return -i;
    }

    return i;
}

int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
    int32_t ret = -1;

    if (ring->begin != ring->end) {
        ret = ring->data[ring->begin++];
        ring->begin %= ring->size;
        if (ch)
            *ch = ret;
    }

    return ret;
}

int32_t ring_read(struct ring *ring, uint8_t *data, ring_size_t size)
{
    int32_t i;

    for (i = 0; i < size; i++) {
        if (ring_read_ch(ring, data + i) < 0)
            return i;
    }

    return -i;
}

int32_t ring_full(struct ring *ring) {
    if (((ring->end + 1) % ring->size) != ring->begin) {
        return 0; // Not full yet
    }
    else {
        return 1; // Don't add any more bytes to this ring right now
    }
}
