#include <c1101.h>
#include <stdint.h>
#include <stdbool.h>
#include <ccpacket.h>
#include <main.h>
#include <string.h>

/**
 * Macros
 */
int test = CCPACKET_BUFFER_LEN;
#define cc1101_Select()  HAL_GPIO_WritePin(SS_PORT, SS, GPIO_PIN_RESET)
//#define cc1101_Select()  digitalWrite(SS, LOW)
// Deselect (SPI) CC1101
//#define cc1101_Deselect()  digitalWrite(SS, HIGH)
#define cc1101_Deselect()  HAL_GPIO_WritePin(SS_PORT, SS, GPIO_PIN_SET)
// Wait until SPI MISO line goes low
//#define wait_Miso()  while(digitalRead(MISO)>0)
#define wait_Miso() while(HAL_GPIO_ReadPin(MISO_PORT, MISO) > 0)
// Get GDO0 pin state
//#define getGDO0state()  digitalRead(CC1101_GDO0)
#define getGDO0state() HAL_GPIO_ReadPin(GDO0_PORT, GDO0)
// Wait until GDO0 line goes high
#define wait_GDO0_high()  while(!getGDO0state())
// Wait until GDO0 line goes low
#define wait_GDO0_low()  while(getGDO0state())

 /**
  * PATABLE
  */
//const byte paTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};
//Microseconds delay https://controllerstech.com/create-1-microsecond-delay-stm32/
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/**
 * CC1101
 *
 * Class constructor
 */
void CC1101(void)
{
  carrierFreq = CFREQ_433;
  channel = CC1101_DEFVAL_CHANNR;
  syncWord[0] = CC1101_DEFVAL_SYNC1;
  syncWord[1] = CC1101_DEFVAL_SYNC0;
  devAddress = CC1101_DEFVAL_ADDR;
}

/**
 * wakeUp
 *
 * Wake up CC1101 from Power Down state
 */
void WakeUp(void)
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeReg
 *
 * Write single register into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void WriteReg(uint8_t regAddr, uint8_t value)
{
  //cc1101_Select();                      // Select CC1101 Do I need to do it manually?
  //wait_Miso();                          // Wait until MISO goes low
  //SPI.transfer(regAddr);                // Send register address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&regAddr, 1, 100); // Send register address
  //SPI.transfer(value);                  // Send value
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&value, 1, 100); // Send value
  //cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeBurstReg
 *
 * Write multiple registers into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void WriteBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{
  uint8_t addr;

  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  //cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 100); // Send value
  //SPI.transfer(addr);                   // Send register address

  HAL_SPI_Transmit(&hspi1, buffer, strlen((const char*)buffer), 100); // Send value
  //for(i=0 ; i<len ; i++)
  //  SPI.transfer(buffer[i]);            // Send value

  //cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cmdStrobe
 *
 * Send command strobe to the CC1101 IC via SPI
 *
 * 'cmd'	Command strobe
 */
void CmdStrobe(uint8_t cmd)
{
  //cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&cmd, 1, 100); // Send value
  //SPI.transfer(cmd);                    // Send strobe command
  //cc1101_Deselect();                    // Deselect CC1101
}

/**
 * readReg
 *
 * Read CC1101 register via SPI
 *
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 *
 * Return:
 * 	Data byte returned by the CC1101 IC
 */
uint8_t ReadReg(uint8_t regAddr, uint8_t regType)
{
  uint8_t addr, val;

  addr = regAddr | regType;
  //cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  //SPI.transfer(addr);                   // Send register address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 100); // Send value
  HAL_SPI_Receive(&hspi1, (uint8_t*)&val, 1, 100);   // Read result
  //val = SPI.transfer(0x00);             // Read result
  //cc1101_Deselect();                    // Deselect CC1101

  return val;
}

/**
 * readBurstReg
 *
 * Read burst data from CC1101 via SPI
 *
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void ReadBurstReg(uint8_t * buffer, uint8_t regAddr, uint8_t len)
{
  uint8_t addr;

  addr = regAddr | READ_BURST;
  //cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 100); // Send value
  //SPI.transfer(addr);                   // Send register address
  HAL_SPI_Receive(&hspi1, buffer, strlen((const char*)buffer), 100);   // Read result
  //for(i=0 ; i<len ; i++)
  //  buffer[i] = SPI.transfer(0x00);     // Read result byte by byte
  //cc1101_Deselect();                    // Deselect CC1101
}

/**
 * reset
 *
 * Reset CC1101
 */
void CC1101_reset(void)
{
  cc1101_Deselect();                    // Deselect CC1101
  delay_us(5);
  cc1101_Select();                      // Select CC1101
  delay_us(10);
  cc1101_Deselect();                    // Deselect CC1101
  delay_us(41);
  cc1101_Select();                      // Select CC1101

  //wait_Miso();                          // Wait until MISO goes low
  HAL_SPI_Transmit(&hspi1, (uint8_t*)CC1101_SRES, 1, 100); // Send value
  //SPI.transfer(CC1101_SRES);            // Send reset command strobe
  //wait_Miso();                          // Wait until MISO goes low

  //cc1101_Deselect();                    // Deselect CC1101

  setCCregs();                          // Reconfigure CC1101
}

/**
 * setCCregs
 *
 * Configure CC1101 registers
 */
void CC1101_setCCregs(void)
{
  writeReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
  writeReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);

  // Set default synchronization word
  setSyncWord(syncWord);

  // Set default device address
  setDevAddress(devAddress);

  // Set default frequency channel
  setChannel(channel);

  writeReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);

  // Set default carrier frequency = 868 MHz
  setCarrierFreq(carrierFreq);

  // RF speed
  if (workMode == MODE_LOW_SPEED)
    writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4_4800);
  else
    writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4_38400);

  writeReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);
  writeReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
  writeReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
  writeReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
  writeReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
  writeReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
  writeReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
  writeReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
  writeReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
  writeReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);

  // Send empty packet
  CCPACKET packet;
  packet.length = 0;
  HAL_SPI_Transmit(&hspi1, packet.data, strlen((const char *)packet.data), 100); // Send value
  //sendData(packet);
}

/**
 * init
 *
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void CC1101_init(uint8_t freq, uint8_t mode)
{
  carrierFreq = freq;
  workMode = mode;
 #ifdef ESP32
  pinMode(SS, OUTPUT);					             // Make sure that the SS Pin is declared as an Output
 #endif
  //SPI.begin();                          // Initialize SPI interface
  MX_SPI1_Init();                         // Initialize SPI interface
  //pinMode(CC1101_GDO0, INPUT);          // Config GDO0 as input

  reset();                              // Reset CC1101

  // Configure PATABLE
  setTxPowerAmp(PA_LowPower);
}

/**
 * setSyncWord
 *
 * Set synchronization word
 *
 * 'syncH'	Synchronization word - High byte
 * 'syncL'	Synchronization word - Low byte
 */
/*
void CC1101_setSyncWord(uint8_t syncH, uint8_t syncL)
{
  writeReg(CC1101_SYNC1, syncH);
  writeReg(CC1101_SYNC0, syncL);
  syncWord[0] = syncH;
  syncWord[1] = syncL;
}
*/
/**
 * setSyncWord (overriding method)
 *
 * Set synchronization word
 *
 * 'syncH'	Synchronization word - pointer to 2-byte array
 */
void CC1101_setSyncWord(uint8_t sync) //pakeiciau is
{
	//padaryti kad
	writeReg(CC1101_SYNC1, sync & 0xF0); //higher part of the byte
	writeReg(CC1101_SYNC0, sync & 0x0F); //lower part of the byte
	//syncWord[0] = syncH;
	//syncWord[1] = syncL;
	//CC1101_setSyncWord(sync[0], sync[1]);
}

/**
 * setDevAddress
 *
 * Set device address
 *
 * @param addr	Device address
 */
void CC1101_setDevAddress(uint8_t addr)
{
  writeReg(CC1101_ADDR, addr);
  devAddress = addr;
}

/**
 * setChannel
 *
 * Set frequency channel
 *
 * 'chnl'	Frequency channel
 */
void CC1101_setChannel(uint8_t chnl)
{
  writeReg(CC1101_CHANNR,  chnl);
  channel = chnl;
}

/**
 * setCarrierFreq
 *
 * Set carrier frequency
 *
 * 'freq'	New carrier frequency
 */
void CC1101_setCarrierFreq(uint8_t freq)
{
  switch(freq)
  {
    case CFREQ_915:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_915);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_915);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_915);
      break;
    case CFREQ_433:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_433);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_433);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_433);
      break;
    case CFREQ_918:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_918);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_918);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_918);
      break;
    default:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);
      break;
  }

  carrierFreq = freq;
}

/**
 * setPowerDownState
 *
 * Put CC1101 into power-down state
 */
void CC1101_setPowerDownState()
{
  // Comming from RX state, we need to enter the IDLE state first
  cmdStrobe(CC1101_SIDLE);
  // Enter Power-down state
  cmdStrobe(CC1101_SPWD);
}

/**
 * sendData
 *
 * Send data packet via RF
 *
 * 'packet'	Packet to be transmitted. First byte is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool CC1101_sendData(CCPACKET packet)
{
	uint8_t marcState;
  bool res = false;

  // Declare to be in Tx state. This will avoid receiving packets whilst
  // transmitting
  rfState = RFSTATE_TX;

  // Enter RX state
  setRxState();

  int tries = 0;
  // Check that the RX state has been entered
  while (tries++ < 1000 && ((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D)
  {
    if (marcState == 0x11)        // RX_OVERFLOW
      flushRxFifo();              // flush receive queue
  }
  if (tries >= 1000) {
    // TODO: MarcState sometimes never enters the expected state; this is a hack workaround.
    return false;
  }

  delay_us(500);

  if (packet.length > 0)
  {
    // Set data length at the first position of the TX FIFO
    writeReg(CC1101_TXFIFO,  packet.length);
    // Write data into the TX FIFO
    writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

    // CCA enabled: will enter TX state only if the channel is clear
    setTxState();
  }

  // Check that TX state is being entered (state = RXTX_SETTLING)
  marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
  if((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
  {
    setIdleState();       // Enter IDLE state
    flushTxFifo();        // Flush Tx FIFO
    setRxState();         // Back to RX state

    // Declare to be in Rx state
    rfState = RFSTATE_RX;
    return false;
  }

  // Wait for the sync word to be transmitted
  wait_GDO0_high();

  // Wait until the end of the packet transmission
  wait_GDO0_low();

  // Check that the TX FIFO is empty
  if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
    res = true;

  setIdleState();       // Enter IDLE state
  flushTxFifo();        // Flush Tx FIFO

  // Enter back into RX state
  setRxState();

  // Declare to be in Rx state
  rfState = RFSTATE_RX;

  return res;
}

/**
 * receiveData
 *
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 *
 * Return:
 * 	Amount of bytes received
 */
uint8_t CC1101_receiveData(CCPACKET * packet)
{
	uint8_t val;
	uint8_t rxBytes = readStatusReg(CC1101_RXBYTES);

  // Any byte waiting to be read and no overflow?
  if (rxBytes & 0x7F && !(rxBytes & 0x80))
  {
    // Read data length
    packet->length = readConfigReg(CC1101_RXFIFO);
    // If packet is too long
    if (packet->length > CCPACKET_DATA_LEN)
      packet->length = 0;   // Discard packet
    else
    {
      // Read data packet
      readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
      // Read RSSI
      packet->rssi = readConfigReg(CC1101_RXFIFO);
      // Read LQI and CRC_OK
      val = readConfigReg(CC1101_RXFIFO);
      packet->lqi = val & 0x7F;
      uint8_t last_bit = val & 0x80; 	// 1000 0000 in binary
      last_bit = last_bit >> 7;			// shift so it would be either 1 or 0
      packet->crc_ok = last_bit;
    }
  }
  else
    packet->length = 0;

  setIdleState();       // Enter IDLE state
  flushRxFifo();        // Flush Rx FIFO
  //cmdStrobe(CC1101_SCAL);

  // Back to RX state
  setRxState();

  return packet->length;
}

/**
 * setRxState
 *
 * Enter Rx state
 */
void CC1101_setRxState(void)
{
  cmdStrobe(CC1101_SRX);
  rfState = RFSTATE_RX;
}

/**
 * setTxState
 *
 * Enter Tx state
 */
void CC1101_setTxState(void)
{
  cmdStrobe(CC1101_STX);
  rfState = RFSTATE_TX;
}

