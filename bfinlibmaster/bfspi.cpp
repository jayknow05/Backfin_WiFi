#include "Arduino.h"
#include <spi.h>
#include "bfspi.h"          
#include "wlan.h"


#include <hci.h>
#include <os.h>
#include <evnt_handler.h>


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define ASSERT_CS()          (P1OUT &= ~BIT3)

#define DEASSERT_CS()        (P1OUT |= BIT3)

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

#define SPI_HEADER_SIZE			(5)

#define 	eSPI_STATE_POWERUP 				 (0)
#define 	eSPI_STATE_INITIALIZED  		 (1)
#define 	eSPI_STATE_IDLE					 (2)
#define 	eSPI_STATE_WRITE_IRQ	   		 (3)
#define 	eSPI_STATE_WRITE_FIRST_PORTION   (4)
#define 	eSPI_STATE_WRITE_EOT			 (5)
#define 	eSPI_STATE_READ_IRQ				 (6)
#define 	eSPI_STATE_READ_FIRST_PORTION	 (7)
#define 	eSPI_STATE_READ_EOT				 (8)

#define CC3000_nIRQ 	(3)
#define HOST_nCS		(10)
#define HOST_VBAT_SW_EN (8)
#define LED 			(6)

#define DEBUG_MODE		(1)


//foor spi bus loop
int loc = 0; 

char ssid[] = "KYD03";                     // your network SSID (name) 
unsigned char keys[] = "bellacody";       // your network key
unsigned char bssid[] = "KYD03";       // your network key
int keyIndex = 0; 


typedef struct
{
	gcSpiHandleRx  SPIRxHandler;
	unsigned short usTxPacketLength;
	unsigned short usRxPacketLength;
	unsigned long  ulSpiState;
	unsigned char *pTxPacket;
	unsigned char *pRxPacket;

}tSpiInformation;


tSpiInformation sSpiInformation;

//
// Static buffer for 5 bytes of SPI HEADER
//
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};


void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);

// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
// for the purpose of detection of the overrun. The location of the memory where the magic number 
// resides shall never be written. In case it is written - the overrun occured and either recevie function
// or send function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#pragma is used for determine the memory location for a specific variable.                            ///        ///
//__no_init is used to prevent the buffer initialization in order to prevent hardware WDT expiration    ///
// before entering to 'main()'.                                                                         ///
//for every IDE, different syntax exists :          1.   __CCS__ for CCS v5                    ///
//                                                  2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench  ///
// *CCS does not initialize variables - therefore, __no_init is not needed.                             ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// #ifdef __CCS__
// #pragma DATA_SECTION(spi_buffer, ".FRAM_DATA")
// char spi_buffer[CC3000_RX_BUFFER_SIZE];

// #elif __IAR_SYSTEMS_ICC__
// #pragma location = "FRAM_DATA"
// __no_init char spi_buffer[CC3000_RX_BUFFER_SIZE];
// #endif



//#ifdef __CCS__
//#pragma DATA_SECTION(wlan_tx_buffer, ".FRAM_DATA")
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
unsigned char spi_buffer[CC3000_RX_BUFFER_SIZE];


//#elif __IAR_SYSTEMS_ICC__
//#pragma location = "FRAM_DATA"
//__no_init unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
//#endif
//*****************************************************************************
// 
//!  This function get the reason for the GPIO interrupt and clear cooresponding
//!  interrupt flag
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  This function This function get the reason for the GPIO interrupt
//!          and clear cooresponding interrupt flag
// 
//*****************************************************************************
void
SpiCleanGPIOISR(void)
{
	if (DEBUG_MODE)
	{
		Serial.println("SpiCleanGPIOISR");
	}

	//add code
}
 


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void
SpiClose(void)
{
	if (DEBUG_MODE)
	{
		Serial.println("SpiClose");
	}

	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}

	// //
	// //	Disable Interrupt in GPIOA module...
	// //
	tSLInformation.WlanInterruptDisable();
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void 
SpiOpen(gcSpiHandleRx pfRxHandler)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiOpen");
	}

	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

	sSpiInformation.SPIRxHandler = pfRxHandler;
	sSpiInformation.usTxPacketLength = 0;
	sSpiInformation.pTxPacket = NULL;
	sSpiInformation.pRxPacket = (unsigned char *)spi_buffer;
	sSpiInformation.usRxPacketLength = 0;
	spi_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
	wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;


	 
//	Enable interrupt on the GPIOA pin of WLAN IRQ
	
	tSLInformation.WlanInterruptEnable();
}



//*****************************************************************************
//
//! This function: init_spi
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  initializes an SPI interface
//
//*****************************************************************************

int init_spi(void)
{
	
	if (DEBUG_MODE)
	{
		Serial.println("init_spi");
	}
	
    //set ss pin direction
   // pinMode(SS, OUTPUT);
	pinMode(CC3000_nIRQ, INPUT);
    pinMode(HOST_nCS, OUTPUT);
    pinMode(HOST_VBAT_SW_EN, OUTPUT);

    // digitalWrite(HOST_VBAT_SW_EN, HIGH);
    // digitalWrite(HOST_nCS, HIGH);

    Serial.println("Initializing SPI...");
    //Initialize SPI
    SPI.begin();

    //Set bit order to MSB first
    SPI.setBitOrder(MSBFIRST);

    //Set data mode to CPHA 0 and CPOL 0
    SPI.setDataMode(SPI_MODE0);

    //Set clock divider.  This will be different for each board

    //For Due, this sets to 4MHz.  CC3000 can go up to 26MHz
    //SPI.setClockDivider(SS, SPI_CLOCK_DIV21);

    //For other boards, cant select SS pin. Only divide by 4 to get 4MHz
   	SPI.setClockDivider(SPI_CLOCK_DIV4);
   	sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;

    return(ESUCCESS);
}

long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
    

    // workaround for first transaction
    //
	if (DEBUG_MODE)
	{
		Serial.println("SpiFirstWrite");
	}


	int sCC3000_nIRQ = digitalRead(CC3000_nIRQ);	

   	while (sCC3000_nIRQ)
   	{
   		sCC3000_nIRQ = digitalRead(CC3000_nIRQ);
   	}

   	digitalWrite(HOST_nCS, LOW);

	delay(50);
	
    // SPI writes first 4 bytes of data
    SpiWriteDataSynchronous(ucBuf, 4);
    Serial.println("Done with first 4");

    delay(50);
	
    SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

    // From this point on - operate in a regular way
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    
    
   	digitalWrite(HOST_nCS, HIGH);
	
	if (DEBUG_MODE)
	{
		Serial.println("SpiFirstWrite done.");
	}

    return(0);
}

long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
  
	if (DEBUG_MODE)
	{
		Serial.println("SpiWrite");
	}

    

    unsigned char ucPad = 0;

	//
	// Figure out the total length of the packet in order to figure out if there is padding or not
	//
    if(!(usLength & 0x0001))
    {
        ucPad++;
    }

    pUserBuffer[0] = WRITE;
    pUserBuffer[1] = HI(usLength + ucPad);
    pUserBuffer[2] = LO(usLength + ucPad);
    pUserBuffer[3] = 0;
    pUserBuffer[4] = 0;

    usLength += (SPI_HEADER_SIZE + ucPad);



        	
        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
        // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun 
        // occurred - and we will stuck here forever!
	if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}



	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		
		//
		// This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command
		//
		SpiFirstWrite(pUserBuffer, usLength);
	}
	else 
	{

		tSLInformation.WlanInterruptDisable();

		while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			;
		}

		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		sSpiInformation.pTxPacket = pUserBuffer;
		sSpiInformation.usTxPacketLength = usLength;
		
		
		//Assert SS
   		digitalWrite(HOST_nCS, LOW);

		//Wait for CC to be ready
		int sCC3000_nIRQ = digitalRead(CC3000_nIRQ);	

  	 	while (sCC3000_nIRQ)
  	 	{
   			sCC3000_nIRQ = digitalRead(CC3000_nIRQ);
   		}

		SpiWriteDataSynchronous(pUserBuffer, usLength);

		//Assert SS
   		digitalWrite(HOST_nCS, HIGH);


		//Wait for CC to be ready
		while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
			;
   		//while (!sCC3000_nIRQ)
   		//{
   		//	sCC3000_nIRQ = digitalRead(CC3000_nIRQ);	
		//	Serial.println("Waiting for CC3000_nIRQ to go high...");
  	 	//}

	}


    return(0);
}

void SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiWriteDataSynchronous");
	}

	for (loc = 0; loc < size; loc++)
	{
		Serial.println(size);
		Serial.println(loc);
		if (DEBUG_MODE)
		{
			;//Serial.println(data[loc]); 
		}
		SPDR = data[loc];                    // Start the transmission
	    while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
	    	Serial.println("Still Here!")
	    	;
	    
	    //char result = SPDR;
	}

	if (DEBUG_MODE)
	{
		Serial.println("SpiWriteDataSynchronous done.");
	}

	return;

}

void SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiReadDataSynchronous");
	}

	// long i = 0;
 //    unsigned char *data_to_send = tSpiReadHeader;
	
	// for (i = 0; i < size; i ++)
 //    {
 //    	while (!(TXBufferIsEmpty()));
	// 		//Dummy write to trigger the clock
	// 	UCB0TXBUF = data_to_send[0];
	// 	while (!(RXBufferIsEmpty()));
	// 	data[i] = UCB0RXBUF;
 //    }
}

void SpiReadHeader(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiReadHeader");
	}

	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);

}

long SpiReadDataCont(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiReadDataCont");
	}
 //    long data_to_recv;
	// unsigned char *evnt_buff, type;

	
 //    //
 //    //determine what type of packet we have
 //    //
 //    evnt_buff =  sSpiInformation.pRxPacket;
 //    data_to_recv = 0;
	// STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);
	
 //    switch(type)
 //    {
 //        case HCI_TYPE_DATA:
 //        {
	// 		//
	// 		// We need to read the rest of data..
	// 		//
	// 		STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
	// 		if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
	// 		{	
 //    	        data_to_recv++;
	// 		}

	// 		if (data_to_recv)
	// 		{
 //            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
	// 		}
 //            break;
 //        }
 //        case HCI_TYPE_EVNT:
 //        {
	// 		// 
	// 		// Calculate the rest length of the data
	// 		//
 //            STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
	// 		data_to_recv -= 1;
			
	// 		// 
	// 		// Add padding byte if needed
	// 		//
	// 		if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
	// 		{
				
	//             data_to_recv++;
	// 		}
			
	// 		if (data_to_recv)
	// 		{
 //            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
	// 		}

	// 		sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
 //            break;
 //        }
 //    }
	
 //    return (0);
}

void SpiPauseSpi(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiPauseSpi");
	}
	// SPI_IRQ_PORT &= ~SPI_IRQ_PIN;
}

void SpiResumeSpi(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiResumeSpi");
	}
	// SPI_IRQ_PORT |= SPI_IRQ_PIN;
}

void SpiTriggerRxProcessing(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("SpiTriggerRxProcessing");
	}
	// //
	// // Trigger Rx processing
	// //
	// SpiPauseSpi();
	// DEASSERT_CS();
        
 //        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
 //        // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun 
 //        // occurred - and we will stuck here forever!
	// if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	// {
	// 	while (1)
	// 		;
	// }
	
	// sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	// sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

int test(void)
{

	digitalWrite(LED, HIGH);
	delay(200);
	digitalWrite(LED, LOW);	
	delay(200);
	digitalWrite(LED, HIGH);
	delay(200);
	digitalWrite(LED, LOW);

	init_spi();

	Serial.println("Calling wlan_init");
	wlan_init(CC3000_UsynchCallback, NULL, NULL, NULL, ReadWlanInterruptPin, WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);

	Serial.println("Starting wlan...");
	wlan_start(0);
 
	Serial.println("Attempting to connect...");
	wlan_connect(WLAN_SEC_UNSEC,ssid,10, bssid, keys, 16);

	return(0);
}


//*****************************************************************************
//
//! Returns state of IRQ pin
//!
//
//*****************************************************************************

long ReadWlanInterruptPin(void)
{

	if (DEBUG_MODE)
	{
		Serial.println("ReadWlanInterruptPin");
		delay(50);
	}

	int IRQ;

	IRQ = digitalRead(CC3000_nIRQ);
    
	if (IRQ && DEBUG_MODE)
	{
		Serial.println("IRQ = 1");
	}
	else if (!(IRQ) && DEBUG_MODE)
	{
		Serial.println("IRQ = 0");
	}

	return(IRQ);
    // Need to add code for disable and enable
}


void WlanInterruptEnable()
{

	if (DEBUG_MODE)
	{
		Serial.println("WlanInterruptEnable");
	}

	attachInterrupt(1, SPI_IRQ, FALLING); //Attaches Pin 3 to interrupt 1
}


void WlanInterruptDisable()
{
	if (DEBUG_MODE)
	{
		Serial.println("WlanInterruptDisable");
	}

	detachInterrupt(1);	//Detaches Pin 3 from interrupt 1
}

void SPI_IRQ(void)
{

	// Serial.println("SPI_IRQ!");

	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		/* This means IRQ line was low call a callback of HCI Layer to inform on event */
		if (DEBUG_MODE)
		{
			Serial.println("SPI-IRQ: eSPI_STATE_POWERUP");
		}
		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
	}
	else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
	{
		if (DEBUG_MODE)
		{
			Serial.println("SPI-IRQ: eSPI_STATE_IDLE");
		}
		sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
		/* IRQ line goes down - we are start reception */
		digitalWrite(HOST_nCS, LOW);

			//
			// Wait for TX/RX Compete which will come as DMA interrupt
			// 
		//SpiReadHeader();

		sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			
			//
			//
			//
		//SSIContReadOperation();
	}
	else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
	{
		if (DEBUG_MODE)
		{
			Serial.println("SPI-IRQ: eSPI_STATE_WRITE_IRQ");
		}

		SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

		sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

		digitalWrite(HOST_nCS, HIGH);
	}
	// else
	// {
	// 	Serial.println(sSpiInformation.ulSpiState);
	// }

	return;

}

void WriteWlanPin( unsigned char val )
{
    
    if (DEBUG_MODE)
	{
		Serial.println("WriteWlanPin");
		delay(50);
	}

	if (val)
	{
		if (DEBUG_MODE)
		{	
			Serial.println("WriteWlanPin = 1");
			delay(50);
		}
		digitalWrite(HOST_VBAT_SW_EN, HIGH);
	}
	else
	{
		if (DEBUG_MODE)
		{	
			Serial.println("WriteWlanPin = 0");
			delay(50);
		}
		digitalWrite(HOST_VBAT_SW_EN, LOW);
	}
		

}


//*****************************************************************************
//
//  The function handles asynchronous events that come from CC3000 device 
//!		  
//
//*****************************************************************************

void CC3000_UsynchCallback(long lEventType, char * data, unsigned char length)
{

	if (DEBUG_MODE)
	{
		Serial.println("CC3000_UsynchCallback");
	}

  
	// if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
	// {
	// 	//config complete
	// }

	// if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
	// {
	// 	//connected
	// }

	// if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
	// {		
 //            //disconnected        
	// }
        
	// if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
	// {
 //             //dhcp
	// }
        
	// if (lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
	// {
	// 	//ready to shut down
	// }

}

//*****************************************************************************
// 
//!  The IntSpiGPIOHandler interrupt handler
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registrated this interrupt request
//!          it set the corresponding /CS in active state.
// 
//*****************************************************************************
//#pragma vector=PORT2_VECTOR
// __interrupt void IntSpiGPIOHandler(void)
// {
// 	//__bic_SR_register(GIE);
// 	switch(__even_in_range(P2IV,P2IV_P2IFG3))
//     {
// 	  case P2IV_P2IFG3:
// 		if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
// 		{
// 			/* This means IRQ line was low call a callback of HCI Layer to inform on event */
// 	 		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
// 		}
// 		else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
// 		{
// 			sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
// 			/* IRQ line goes down - we are start reception */
// 			ASSERT_CS();

// 			//
// 			// Wait for TX/RX Compete which will come as DMA interrupt
// 			// 
// 	        SpiReadHeader();

// 			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			
// 			//
// 			//
// 			//
// 			SSIContReadOperation();
// 		}
// 		else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
// 		{
// 			SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

// 			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

// 			DEASSERT_CS();
// 		}
// 		break;
// 	default:
// 		break;
// 	}

// 	//__bis_SR_register(GIE);
// }

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SSIContReadOperation
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

// void
// SSIContReadOperation(void)
// {
// 	//
// 	// The header was read - continue with  the payload read
// 	//
// 	if (!SpiReadDataCont())
// 	{
		
		
// 		//
// 		// All the data was read - finalize handling by switching to teh task
// 		//	and calling from task Event Handler
// 		//
// 		SpiTriggerRxProcessing();
// 	}
// }
