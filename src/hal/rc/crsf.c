#include "hal/rc/crsf.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t _lut[256];

void Crc8_init(uint8_t poly)
{
    for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        _lut[idx] = crc & 0xff;
    }
}

uint8_t calc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = _lut[crc ^ *data++];
    }
    return crc;
}

void packetChannelsPacked(crsf_decoder_t* decoder)
{
    const crsf_header_t *p = (crsf_header_t*)decoder->_rxBuf;
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    decoder->_channels[0] = ch->ch0;
    decoder->_channels[1] = ch->ch1;
    decoder->_channels[2] = ch->ch2;
    decoder->_channels[3] = ch->ch3;
    decoder->_channels[4] = ch->ch4;
    decoder->_channels[5] = ch->ch5;
    decoder->_channels[6] = ch->ch6;
    decoder->_channels[7] = ch->ch7;
    decoder->_channels[8] = ch->ch8;
    decoder->_channels[9] = ch->ch9;
    decoder->_channels[10] = ch->ch10;
    decoder->_channels[11] = ch->ch11;
    decoder->_channels[12] = ch->ch12;
    decoder->_channels[13] = ch->ch13;
    decoder->_channels[14] = ch->ch14;
    decoder->_channels[15] = ch->ch15;

    for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        decoder->_channels[i] = map(decoder->_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
}

void processPacketIn(crsf_decoder_t* decoder)
{
        const crsf_header_t *hdr = (crsf_header_t *)decoder->_rxBuf;
    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS:
            //packetGps(hdr);
            break;
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            packetChannelsPacked(decoder);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            //packetLinkStatistics(hdr);
            break;
        }
    } // CRSF_ADDRESS_FLIGHT_CONTROLLER
}

bool crsf_update(crsf_decoder_t* decoder)
{
    const crsf_header_t *hdr = (crsf_header_t *)decoder->_rxBuf;
    if(hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        uint8_t len = hdr->frame_size;
        // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
        // assumes there never will be a CRSF message that just has a type and no data (X)
        if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
        {
            decoder->_rxBufPos = 0;
        }
        else
        {
            uint8_t inCrc = hdr->data[2 + len - 1];
            uint8_t crc = calc(&(hdr->data[2]), len - 1);
            if (crc == inCrc)
            {
                processPacketIn(decoder);
                decoder->_rxBufPos = 0;
                decoder->crsf_data_ready = true;
            }
        }
    }
    else
    {
        decoder->_rxBufPos = 0;
    }
}

void crsf_input(crsf_decoder_t* decoder, const uint8_t* values)
{
    decoder->_rxBuf[decoder->_rxBufPos++] = *values;
    if (decoder->_rxBufPos == (sizeof(decoder->_rxBuf)/sizeof(decoder->_rxBuf[0])))
    {
        decoder->_rxBufPos = 0;
    }
}

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder)
{
    if (decoder == NULL) {
        return RT_EINVAL;
    }
    rt_memset(decoder, 0, sizeof(crsf_decoder_t));
    Crc8_init(0xd5);
    return RT_EOK;
}