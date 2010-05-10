#define SERIAL_PORT0 0
#define SERIAL_PORT1 1
#define RX_INTERRUPT 0x01
#define TX_INTERRUPT 0x02


#define cbi(REG8,BITNUM) REG8 &= ~(_BV(BITNUM))
#define sbi(REG8,BITNUM) REG8 |= _BV(BITNUM)



/** @brief crc-ccitt truncated polynomial. */
#define POLY 0x1021          
/** @brief crc-ccitt initial value. */
#define INITIAL_VALUE 0xFFFF

   
#define MAXBUF_RX		10
#define MAXBUF_TX		200

/**CommsSend Mode*/
#define	NO_BLOCK		0x01
#define	MRG_BLOCK		0x02
#define	FUL_BLOCK		0x04



