




/*Hardware Dependent Item*/
#define TXD1_READY			bit_is_set(UCSR1A,5) //(UCSR1A_Bit5)
#define TXD1_DATA			(UDR1)
#define RXD1_READY			bit_is_set(UCSR1A,7)
#define RXD1_DATA			(UDR1)

#define TXD0_READY			bit_is_set(UCSR0A,5)
#define TXD0_DATA			(UDR0)
#define RXD0_READY			bit_is_set(UCSR0A,7)
#define RXD0_DATA			(UDR0)


uint8_t PingDevice(uint8_t bCount);
