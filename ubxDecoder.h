#ifndef UBXDECODER_H
#define UBXDECODER_H

#include <cstdint>
#include <vector>

class ubxDecoder
{
public:
    ubxDecoder(uint8_t hdr0, uint8_t hdr1, uint16_t buf_len);
    int receive(uint8_t x);

    const uint8_t *raw() const;
    uint16_t length() const;

private:
    uint8_t _hdr0;
    uint8_t _hdr1;
    int _count;
    std::vector<uint8_t> _buf;
    uint8_t *_raw;
    uint16_t _length;
    uint8_t _cs[2],__cs[2];
};

#endif // UBXDECODER_H
