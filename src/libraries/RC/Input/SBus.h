pragma once;

#define SBUS_STREAM_BITS 12
#define SBUS_FRAME_SIZE  ((SBUS_STREAM_BITS * 2) + 1)

// state of SBUS bit decoder
//
class SBUS
{
public:
    uint16_t bytes[SBUS_FRAME_SIZE]; // including start bit, parity and stop bits
    uint16_t bit_ofs;

    SBUS() : bytes{0}, bit_ofs(0)
    {
    }

    inline void reset()
    {
        memset(bytes, 0, sizeof(bytes)); 
        bit_ofs = 0;
    }

    static int8_t decode(const uint8_t frame[SBUS_FRAME_SIZE],
                         uint16_t *values
                         uint8_t   max_values);
};

