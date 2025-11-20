#ifdef AIS_FORWARD_XXX
#include "windparse.h"

// listen on serial port (NMEA0183serial), read sentences, transmit broadcast on UDP
// for 1-way forwarding of AIS traffic
// SignalK and Navionics will be listeners
// based on https://github.com/alvra/nmea-bridge/tree/main

uint16_t nmea_sentences_received = 0;
uint16_t nmea_sentences_sent = 0;

#define UDP_SENTENCE_BUFFER_SIZE 64

/* The buffer should be long enough for about two sentences
** so we can handle the case where a (single) newline is lost.
*/
#define NMEA_SENTENCE_BUFFER_SIZE MAX_NMEA0183_MSG_BUF_LEN
char nmea_sentence_buffer[NMEA_SENTENCE_BUFFER_SIZE + 1];
uint8_t nmea_sentence_buffer_filled = 0;  // number of chars in buffer

void handle_outgoing_sentence(char *sentence, size_t length);
void transmit_outgoing_over_udp(char *sentence, size_t length);

void handle_serial_event() {
    // on incoming data over the hardware serial port
    int serial_character_int;
    char serial_character;
    while (1) {
        serial_character_int = NMEA0183serial.read();

        if (serial_character_int < 0) {
            // no more data available
            return;
        }

        serial_character = (char)serial_character_int;
        // Debug output disabled - was causing garbled serial output
        // Serial.print(serial_character);

        // TODO sometimes this gives a large number of zero bytes
        // TODO then we also don't get the extra newline

        nmea_sentence_buffer[nmea_sentence_buffer_filled] = serial_character;
        nmea_sentence_buffer_filled ++;
        if (serial_character == '\n') {

        } else if (nmea_sentence_buffer_filled >= NMEA_SENTENCE_BUFFER_SIZE) {
            Serial.println("nmea sentence buffer overflow");
            // A newline must have been lost since at least two sentences
            // should fit in the buffer, so send all buffered data immedately
            // to let the client decide how to handle this.
            nmea_sentence_buffer[NMEA_SENTENCE_BUFFER_SIZE] = '\n';
            nmea_sentence_buffer_filled = NMEA_SENTENCE_BUFFER_SIZE + 1;
        } else {
            continue;
        }
        handle_outgoing_sentence(
            nmea_sentence_buffer, nmea_sentence_buffer_filled);
        nmea_sentence_buffer_filled = 0;
    }
}

void handle_outgoing_sentence(char *sentence, size_t length) {
    /* data is received from the nmea device
    **   sentence: always includes the newline and must be newline terminted
    */
    transmit_outgoing_over_udp(sentence, length);

    nmea_sentences_received ++;
}

void transmit_outgoing_over_udp(char *sentence, size_t length) {
    int result = udp_server.beginPacket(INADDR_NONE, UDP_FORWARD_PORT);
    udp_server.write((const uint8_t*)sentence, length);
    udp_server.endPacket();
}

#if NOTDEF  // keeping in case I want to change to multicast in future
void transmit_outgoing_over_udp(char *sentence, size_t length) {
    if (config.tx_mode == TransmitMode::multicast) {
        int result = udp_server.beginPacketMulticast(
            effective_tx_address, config.tx_port,
            get_device_ip_address());
    } else {
        int result = udp_server.beginPacket(
            effective_tx_address, config.tx_port);
    }
    udp_server.write(sentence, length);
    udp_server.endPacket();
}
#endif

#endif // AIS_FORWARD