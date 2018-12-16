typedef struct {
    // uint8_t command;
    uint16_t seq;
    uint32_t timestamp;
    uint8_t crc;
}Message;