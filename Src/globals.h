#ifndef HEADER_FILE
#define HEADER_FILE

#include <stdint.h>
#include "usbd_def.h"

extern volatile int volume; //loudness of music. controlled in main, used in timer callback
extern const int maxPower;

extern volatile uint8_t nextBuffer;
extern volatile uint8_t currentBuffer;

#define musicSamples 100
#define musicBufferSize 64*musicSamples //should be a multiple of USB_FS_MAX_PACKET_SIZE
extern uint8_t musicBuffer[musicSamples][64];
extern volatile unsigned int musicIndex;
extern volatile uint8_t state;

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//void memcopy(volatile unsigned char* dst, unsigned char* Buf, int length);
#endif
