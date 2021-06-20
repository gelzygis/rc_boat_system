#ifndef _CCPACKET_H
#define _CCPACKET_H

/**
 * Buffer and data lengths
 */
#define CCPACKET_BUFFER_LEN        64
#define CCPACKET_DATA_LEN          CCPACKET_BUFFER_LEN - 3

/**
 * Class: CCPACKET
 *
 * Description:
 * CC1101 data packet class
 */
typedef struct CCPACKET
{
	/**
	 * Data length
	 */
	unsigned char length;

	/**
	 * Data buffer
	 */
	unsigned char data[CCPACKET_DATA_LEN];

	/**
	 * CRC OK flag
	 */
	bool crc_ok;

	/**
	 * Received Strength Signal Indication
	 */
	unsigned char rssi;

	/**
	 * Link Quality Index
	 */
	unsigned char lqi;
} CCPACKET;

#endif
