
typedef struct {
    uint16_t rc_count;
    uint16_t max_channels;

    bool crsf_failsafe;
    bool crsf_frame_drop;
    uint32_t crsf_frame_drops;
    uint32_t partial_frame_count;

    uint32_t last_rx_time;
    uint32_t last_frame_time;

    bool crsf_data_ready;
    uint8_t crsf_lock;

    crsf_DECODE_STATE crsf_decode_state;
    ringbuffer* crsf_rb;
    uint8_t crsf_frame[crsf_FRAME_SIZE + (crsf_FRAME_SIZE / 2)];
    uint16_t crsf_val[MAX_crsf_CHANNEL];


    uint32_t last_rx_time;
    uint32_t last_frame_time;
    bool crsf_data_ready;
    uint8_t crsf_lock;


    uint8_t _lut[256];
    int _channels[CRSF_NUM_CHANNELS];
    uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
    uint8_t _rxBufPos;

} crsf_decoder_t;

rt_inline void crsf_lock(crsf_decoder_t* decoder)
{
    decoder->crsf_lock = 1;
}

rt_inline void crsf_unlock(crsf_decoder_t* decoder)
{
    decoder->crsf_lock = 0;
}

rt_inline uint8_t crsf_islock(crsf_decoder_t* decoder)
{
    return decoder->crsf_lock;
}

rt_inline uint8_t crsf_data_ready(crsf_decoder_t* decoder)
{
    return decoder->crsf_data_ready;
}

rt_inline void crsf_data_clear(crsf_decoder_t* decoder)
{
    decoder->crsf_data_ready = 0;
}

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder);
void crsf_input(crsf_decoder_t* decoder, const uint8_t* values);
bool crsf_update(crsf_decoder_t* decoder);