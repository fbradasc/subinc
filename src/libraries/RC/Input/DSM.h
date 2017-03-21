pragma once;

#define DSM_FRAME_BITS     10      // DSM stream size in bits
#define DSM_FRAME_SIZE     16      // DSM frame size in bytes
#define DSM_FRAME_CHANNELS  7      // Max supported DSM channels

class DSM
{
public:
    uint16_t bytes[DSM_FRAME_SIZE]; // including start bit, parity and stop bits
    uint16_t bit_ofs;

    DSM() : bytes{0}, bit_ofs(0)
    {
    }

    inline void reset()
    {
        memset(bytes, 0, sizeof(bytes)); 
        bit_ofs = 0;
    }

    static uint16_t decode(uint64_t frame_time,
                           const uint8_t frame[DSM_FRAME_SIZE], 
                           uint16_t *values, 
                           uint16_t max_values);

private:
    static bool decode_channel(uint16_t raw, uint8_t shift, uint16_t &channel, uint16_t &value);
    static void guess_format(bool reset, const uint8_t frame[DSM_FRAME_SIZE]);
};
