

#define CRSF_BAUDRATE           420000
#define CRSF_NUM_CHANNELS 16
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_2000 1792

#define CRSF_MAX_PACKET_SIZE 64 // max declared len is 62+DEST+LEN on top of that = 64
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_PACKET_SIZE - 4) // Max size of payload in [dest] [len] [type] [payload] [crc8]

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_e;

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;


typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    uint8_t data[0];
} PACKED crsf_header_t;

typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;



long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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


void packetChannelsPacked(const crsf_header_t *p)
{
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        _channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
}


void handleByteReceived()
{
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rxBufPos > 1)
        {
            uint8_t len = _rxBuf[1];
            // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
            // assumes there never will be a CRSF message that just has a type and no data (X)
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
            {
                shiftRxBuffer(1);
                reprocess = true;
            }

            else if (_rxBufPos >= (len + 2))
            {
                uint8_t inCrc = _rxBuf[2 + len - 1];
                uint8_t crc = calc(&_rxBuf[2], len - 1);
                if (crc == inCrc)
                {
                    processPacketIn(len);
                    shiftRxBuffer(len + 2);
                    reprocess = true;
                }
                else
                {
                    shiftRxBuffer(1);
                    reprocess = true;
                }
            }  // if complete packet
        } // if pos > 1
    } while (reprocess);
}

void processPacketIn(uint8_t len)
{
    const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS:
            //packetGps(hdr);
            break;
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            packetChannelsPacked(hdr);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            //packetLinkStatistics(hdr);
            break;
        }
    } // CRSF_ADDRESS_FLIGHT_CONTROLLER
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

    decoder->crsf_decode_state = crsf_DECODE_STATE_DESYNC;
    decoder->max_channels = MAX_crsf_CHANNEL;

    decoder->crsf_rb = ringbuffer_create(2048);
    if (decoder->crsf_rb == NULL) {
        printf("Fail to create crsf ring buffer!\n");
        return RT_ENOMEM;
    }

    return RT_EOK;
}