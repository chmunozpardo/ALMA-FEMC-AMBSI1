/*!	\file	main.c
	\brief	AMBSI1 firmware code

	This is the firmware to be loaded into the AMBSI1 to allow the AMBSI1 to work as a
	bridge between the CAN bus contained in the AMB and the ARCOM embedded controller
	handling the front end hardware.

	\todo
		- Find a way to figure out if the first request for the special RCAs range was completed with
		  success. Right now if there is any problem in the communication, even a timeout, the returned
		  value is considered a valid value and the special address range is registered anyway.
		  This cause the following messages querying for the monitor and control address ranges to be
		  sent at bogous addresses.
		- Make a more roboust handshaking so that there are no hardware timing problems. If full
		  handshake is used though, the communication is very very slow. Maybe is enough to add extra
		  _nop() in critical points.

	This is the code running in the AMBSI1 which handles the incoming CAN messages and communicates with
	the ARCOM Pegasus board.
	
	The AMBSI1 is used as a simple bridge to connect the CAN bus to the ARCOM Pegasus board. This choice
	was forced because of the decision to make the AMBSI1 board a standard interface for ALMA. Unfortunately
	the AMBSI1 board doesn't have the power to perform the operation required to control the front end
	subsystem.

*/
/* Defines */

//! Full handshake mode
//! Fully handshaken communication with the ARCOM board (slow but failproof).
#define FULL_HANDSHAKE

//! Longest timeout allowed
/*! Every time during communication the AMBSI1 is waiting for an aknowledment by the ARCOM board a 
	timeout is implemented to avoid hung-ups. The timeout is a counter that counts from \p MAX_TIMEOUT
	down to zero unless an aknowledgment is received. */
#define MAX_TIMEOUT 0xFFFF // About 70 ms. Give time to the ARCOM to print debug info on the screen

//! Is the firmware using the 48 ms pulse?
/*! Defines if the 48ms pulse is used to trigger the correponding interrupt.
	If yes then P8.0 will not be available for use as a normal I/O pin since
	the assigned pin on the C167 will be jumpered to receive the 48ms pulse.

	\note	To be able to use the 48 ms pulse, it is necessary to program the
	 	  	xilinx chip to allow the incoming pulse to be passed throught. */
// #define USE_48MS

#define MAX_CAN_MSG_PAYLOAD			8		// Max CAN message payload size. Used to determine if error occurred


//! \b 0x20000 -> Base address for the special monitor RCAs
/*! This is the starting relative %CAN address for the special monitor
    requests available in the firmware.

    \warning    This is the only RCA that has to be specified in \b both
                firmwares to allow the system to work. If it is necessary to
                change this address then a change \b has to be made in the
                ARCOM firmware to reflect the new value and vice-versa.
                This adress is used to get informations about the firmware
				version. */
#define BASE_SPECIAL_MONITOR_RCA    0x20000L
#define GET_AMBSI1_VERSION_INFO     0x20000L    //!< \b BASE+0x00 -> AMBSI1 dedicated message to get firmware version. It should never be received from this software
#define GET_SETUP_INFO              0x20001L    //!< \b BASE+0x01 -> AMBSI1 dedicated message to get setup info. It should never be received from this software
#define GET_ARCOM_VERSION_INFO      0x20002L	//!< \b BASE+0x02 -> Information about the ARCOM Pegasus firware version. This is the first addressable special RCA (Don't change the offset of this RCA)
#define GET_SPECIAL_MONITOR_RCAS    0x20003L	//!< \b BASE+0x03 -> Information about the special monitor RCA range (Don't change the offset of this RCA)
#define GET_SPECIAL_CONTROL_RCAS    0x20004L	//!< \b BASE+0x04 -> Information about the special control RCA range (Don't change the offset of this RCA)
#define GET_MONITOR_RCAS            0x20005L	//!< \b BASE+0x05 -> Information about the monitor RCA range (Don't change the offset of this RCA)
#define GET_CONTROL_RCAS            0x20006L	//!< \b BASE+0x06 -> Information about the control RCA range (Don't change the offset of this RCA)
#define GET_MON_TIMERS1_RCA         0x20010L    //!< \b BASE+0x10 -> Monitor timing 
#define GET_MON_TIMERS2_RCA         0x20011L    //!< \b BASE+0x11 -> Monitor timing 

/* Version Info */
#define VERSION_MAJOR 01	//!< Major Version
#define VERSION_MINOR 01	//!< Minor Revision
#define VERSION_PATCH 00	//!< Patch Level

/* Uses serial port */
#include <reg167.h>
#include <intrins.h>

/* include library interface */
#include "..\libraries\amb\amb.h"
#include "..\libraries\ds1820\ds1820.h"

/* Set aside memory for the callbacks in the AMB library */
static CALLBACK_STRUCT idata cb_memory[7];

/* CAN message callbacks */
int ambient_msg(CAN_MSG_TYPE *message); 	//!< Called to get the board temperature temperature
int controlMsg(CAN_MSG_TYPE *message);  	//!< Called to handle CAN control messages
int monitorMsg(CAN_MSG_TYPE *message);  	//!< Called to handle CAN monitor messages 
int getSetupInfo(CAN_MSG_TYPE *message);  	//!< Called to get the AMBSI1 <-> ARCOM link/setup information 
int getVersionInfo(CAN_MSG_TYPE *message);	//!< Called to get firmware version informations 
int getMonTimers1(CAN_MSG_TYPE *message);    //!< Retrieve last monitor message timers
int getMonTimers2(CAN_MSG_TYPE *message);    //!< Retrieve last monitor message timers

/* A global for the last read temperature */
static ubyte idata ambient_temp_data[4];

/* External bus control signal buffer chip enable is on P4.7 */
sbit  DISABLE_EX_BUF	= P4^7;

/* Arcom Parallel port connection lines */
sbit  WRITE				= P2^2;
sbit  DSTROBE			= P2^3;
sbit  WAIT				= P2^8;
sbit  INT				= P2^7;
sbit  SELECT			= P2^10;
sbit  INIT				= P2^5;

/* Separate timers for each phase of monitor transaction */
static unsigned int monTimer1;
static unsigned int monTimer2;
static unsigned int monTimer3;
static unsigned int monTimer4;
static unsigned int monTimer5;
static unsigned int monTimer6;
static unsigned int monTimer7;

/* Macro to implement FULL_HANDSHAKE */
// Wait for Data Strobe to go low
#define IMPL_HANDSHAKE(TIMER) for(TIMER = MAX_TIMEOUT; TIMER && DSTROBE; TIMER--) {}

/* RCAs address ranges */
static unsigned long idata lowestMonitorRCA,highestMonitorRCA,
						   lowestControlRCA,highestControlRCA,
                           lowestSpecialMonitorRCA,highestSpecialMonitorRCA,
						   lowestSpecialControlRCA,highestSpecialControlRCA;

/* A global to fake CAN messages */
static CAN_MSG_TYPE idata myCANMessage;

/* Globals to check for initialization of RCA */
static bit idata ready;			// is the communication between the ARCOM and AMBSI ready?
static bit idata initialized;	// have the RCAs been initialized?

//! MAIN
/*! Takes care of initializing the AMBSI1, the CAN subrutine and globally enables interrupts. */ 
void main(void) {
	#ifdef USE_48MS
	  // Setup the CAPCOM2 unit to receive the 48ms pulse from the Xilinx
		P8&=0xFE; // Set value of P8.0 to 0
		DP8&=0xFE; // Set INPUT direction for P8.0
		CCM4&=0xFFF0; // Clear setup for CCMOD16
		CCM4|=0x0001; // Set CCMOD16 to trigger on rising edge
		CC16IC=0x0078; // Interrupt: ILVL=14, GLVL=0;
	#endif // USE_48MS

	/* Make sure that external bus control signal buffer is disabled */
	DP4 |= 0x01;
	DISABLE_EX_BUF = 1;

	/* Initialise the slave library */
	if (amb_init_slave((void *) cb_memory) != 0) 
		return;

	/* Register callbacks for CAN events */
	if (amb_register_function(0x30003, 0x30003, ambient_msg) != 0)
		return;

	/* Register callbacks for CAN version information event (RCA -> 0x20000) */
	if (amb_register_function(GET_AMBSI1_VERSION_INFO, GET_AMBSI1_VERSION_INFO, getVersionInfo) != 0)
		return;

	/* Initialize ports for communication */
	DP7=0x00;
	DP7=0xFF;
	P2=0x0000;
	SELECT=1;
	INIT=0;
	DP2=0x0580; 

	/* Register callbacks for CAN events (RCA -> 0x20001) */
	if (amb_register_function(GET_SETUP_INFO, GET_SETUP_INFO, getSetupInfo) != 0)
		return;

    /* Register callbacks for monitor timers (RCA -> 0x20010, 0x20011) */
    //if (amb_register_function(GET_MON_TIMERS1_RCA, GET_MON_TIMERS1_RCA, getMonTimers1) != 0)
    //    return;

    //if (amb_register_function(GET_MON_TIMERS2_RCA, GET_MON_TIMERS2_RCA, getMonTimers2) != 0)
    //    return;

	/* globally enable interrupts */
  	amb_start();

	/* Handshake readiness status with ARCOM board */
	ready=0;
	SELECT = 0; // Select line to 0
	while(INIT){ // Wait of init line to go to 0. In the mean time read the temperature
		ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
	}
	ready=1;

	/* Never return */
	while (1){
		ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
	}
}



/*! This function will return the firmware version for the AMBSI1 board.
	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int getVersionInfo(CAN_MSG_TYPE *message){
	message->data[0]=VERSION_MAJOR;
	message->data[1]=VERSION_MINOR;
	message->data[2]=VERSION_PATCH;
	message->len=3;

	return 0;
}

int getMonTimers1(CAN_MSG_TYPE *message) {
    message->data[1] = (unsigned char) (monTimer1);
    message->data[0] = (unsigned char) (monTimer1 >> 8);
    message->data[3] = (unsigned char) (monTimer2);
    message->data[2] = (unsigned char) (monTimer2 >> 8);
    message->data[5] = (unsigned char) (monTimer3);
    message->data[4] = (unsigned char) (monTimer3 >> 8);
    message->data[7] = (unsigned char) (monTimer4);
    message->data[6] = (unsigned char) (monTimer4 >> 8);
    message->len=8;
    return 0;
}

int getMonTimers2(CAN_MSG_TYPE *message) {
    message->data[1] = (unsigned char) (monTimer5);
    message->data[0] = (unsigned char) (monTimer5 >> 8);
    message->data[3] = (unsigned char) (monTimer6);
    message->data[2] = (unsigned char) (monTimer6 >> 8);
    message->data[5] = (unsigned char) (monTimer7);
    message->data[4] = (unsigned char) (monTimer7 >> 8);
    message->data[7] = (unsigned char) (MAX_TIMEOUT);
    message->data[6] = (unsigned char) (MAX_TIMEOUT >> 8);
    message->len=8;
    return 0;
}

/*! This function get the RCAs info from the ARCOM board and register the appropriate CAN functions.
	
	This function will return a CAN message with 1 byte (uchar) payload. The meaning of the payload
	are as follows:
		- 0x00 -> No Error	
	   	- 0x01 -> Error in the registration of the special monitor RCA
	   	- 0x02 -> Error in the registration of the special control RCA
	   	- 0x03 -> Error in the registration of the monitor RCA
	    - 0x04 -> Error in the registration of the control RCA
	   	- 0x05 -> Warning RCA already initialized
	   	- 0x06 -> Communication between ARCOM and AMBSI not yet established
		- 0x07 -> Timeout while forwarding CAN message to the ARCOM board

	\param	*message	a CAN_MSG_TYPE
	\return
		- 0  -> Everything went OK
		- -1 -> ERROR */
int getSetupInfo(CAN_MSG_TYPE *message){

	/* The initialization message has to be a monitor message */
	if(message->dirn==CAN_CONTROL){
		return -1;
	}
	
	/* Return message size: 1 byte */
	message->len = 1;

	/* If not ready to communicate, return the message and wait */
	if(!ready){
		message->data[0]=0x06; // Error 0x06: communication between ARCOM and AMBSI not yet established
		return -1;
	}

	/* If already initialized do not inizialize again */
	if(initialized){
		message->data[0]=0x05; // Error 0x05: RCA already initialized
		return -1;
	}

	/* SPECIAL MONITOR RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_MONITOR_RCAS; // 0x20003 -> RCA: special address to retrieve the special monitor RCAs informations
	if(monitorMsg(&myCANMessage)){ // Send the monitor request.
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
	amb_register_function(lowestSpecialMonitorRCA, highestSpecialMonitorRCA, monitorMsg);


	/* SPECIAL CONTROL RCAs */
	/* Get the information on the available special control RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_CONTROL_RCAS; // 0x20004 -> RCA: special address to retrieve the special control RCAs informations
	if(monitorMsg(&myCANMessage)){ // Send the monitor request.
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		/* Unregister previously succesfully registered functions */
		amb_unregister_last_function(); // SPECIAL MONITOR RCAs
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special control RCA messages */
	amb_register_function(lowestSpecialControlRCA, highestSpecialControlRCA, controlMsg);


	/* MONITOR RCAs */
	/* Get the information on the available monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_MONITOR_RCAS; // 0x20005 -> RCA: special address to retrieve the monitor RCAs informations
	if(monitorMsg(&myCANMessage)){ // Send the monitor request.
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		/* Unregister previously succesfully registered functions */
		amb_unregister_last_function(); // SPECIAL CONTROL RCAs
		amb_unregister_last_function(); // SPECIAL MONITOR RCAs
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
	amb_register_function(lowestMonitorRCA, highestMonitorRCA, monitorMsg);


	/* CONTROL RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_CONTROL_RCAS; // 0x20006 -> RCA: special address to retrieve the special control RCAs informations
	if(monitorMsg(&myCANMessage)){ // Send the monitor request.
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		/* Unregister previously succesfully registered functions */
		amb_unregister_last_function(); // MONITOR RCAs
		amb_unregister_last_function(); // SPECIAL CONTROL RCAs
		amb_unregister_last_function(); // SPECIAL MONITOR RCAs
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
	amb_register_function(lowestControlRCA, highestControlRCA, controlMsg);


	/* No error */
	initialized=1; // Remember that the RCA have already been initialized
	message->data[0]=0;

    return 0;
}



/*! Return the temperature of the AMBSI as measured by the DS1820 onboard chip.

	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int ambient_msg(CAN_MSG_TYPE *message) {

	if (message->dirn == CAN_MONITOR) {  /* Should only be a monitor requests */
		message->len = 4;
		message->data[0] = ambient_temp_data[0];
		message->data[1] = ambient_temp_data[1];
		message->data[2] = ambient_temp_data[2];
		message->data[3] = ambient_temp_data[3];
	} 

	return 0;
}



/* Triggers every 48ms pulse */
void received_48ms(void) interrupt 0x30{
// Put whatever you want to be execute at the 48ms clock.
// Remember that right now this interrupt has higher priority than the CAN.
// Also to be able to use the 48ms, the Xilinx has to be programmed to connect the
// incoming pulse on (pin31) to the cpu (pin28).
}



/*! This function will be called in case a CAN control message is received.
	It will start communication with the ARCOM board triggering the parallel port
	interrupt and the sending the CAN message information to the ARCOM board.

	Since a CAN control request doesn't require any aknowledgment, this function
	will then return.

	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int controlMsg(CAN_MSG_TYPE *message){

	unsigned char counter;
	unsigned int timer;

	if(message->dirn==CAN_MONITOR){
		monitorMsg(message);
		return 0;
	}

	/* Trigger interrupt */
	INT = 1;

	/* Send RCA */
    IMPL_HANDSHAKE(timer)
	P7 = (uword) (message->relative_address);	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0; 	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */
	#ifdef FULL_HANDSHAKE
		IMPL_HANDSHAKE(timer)
	#else
		_nop_();  // One nop to wait for following data strobe to go low
	#endif
	P7 = (uword) (message->relative_address>>8); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0; 	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(timer)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = (uword) (message->relative_address>>16); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(timer)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = (uword) (message->relative_address>>24); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

	/* Send payload size (0 -> monitor message) */
    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(timer)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = message->len;  // Put data on port (0 -> monitor message)
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

	for(counter=0;counter<message->len;counter++){
        #ifdef FULL_HANDSHAKE
            IMPL_HANDSHAKE(timer)
        #else
            // The for cycle is slow enough for following data strobe to go low
        #endif
		P7 = message->data[counter];	// Put data on port (0 -> monitor message)
		WAIT = 1;	// Acknowledge with Wait going high
		WAIT = 0; 	/* Wait down as quick as possible for next message.
					   Any wait state will keep wait high too long and make the ARCOM believe it is an
					   aknowledgment to the following data strobe. */
	}

	/* Untrigger interrupt */
	INT = 0;

	return 0;
}



/*! This function will be called in case a CAN monitor message is received.
	It will start communication with the ARCOM board triggering the parallel port
	interrupt and the sending the CAN message information to the ARCOM board.

	Since a CAN monitor request does require response within 150us, this function
	will then wait for data to come back from the ARCOM board.

	\param	*message	a CAN_MSG_TYPE 
	\return
		- 0 -> Everything went OK
	    - -1 -> Time out during CAN message forwarding */
int monitorMsg(CAN_MSG_TYPE *message) {

	unsigned char counter;

	if(message->dirn==CAN_CONTROL){
		controlMsg(message);
		return 0;
	}

	/* Trigger interrupt */
	INT = 1;

	/* Send RCA */
	IMPL_HANDSHAKE(monTimer1)
	P7 = (uword) (message->relative_address);	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0; 	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(monTimer2)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = (uword) (message->relative_address>>8); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(monTimer3)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = (uword) (message->relative_address>>16); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(monTimer4)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = (uword) (message->relative_address>>24); 	// Put data on port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

	/* Send payload size (0 -> monitor message) */
    #ifdef FULL_HANDSHAKE
        IMPL_HANDSHAKE(monTimer5)
    #else
        _nop_();  // One nop to wait for following data strobe to go low
    #endif
	P7 = message->len;  // Put data on port (0 -> monitor message)
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */

	/* Set port to receive data */
	DP7 = 0x00;

	/* Receive monitor payload size */
	IMPL_HANDSHAKE(monTimer6)
	message->len = (ubyte) P7;	// Read data from port
	WAIT = 1;	// Acknowledge with Wait going high
	WAIT = 0;	/* Wait down as quick as possible for next message.
				   Any wait state will keep wait high too long and make the ARCOM believe it is an
				   aknowledgment to the following data strobe. */
	
	/* Get the payload */
	if(message->len<=MAX_CAN_MSG_PAYLOAD){ // If too many bytes of payload assume error and exit
		for(counter=0;counter<message->len;counter++){ 
            #ifdef FULL_HANDSHAKE
                IMPL_HANDSHAKE(monTimer7)
            #else
                // The for cycle is slow enough for following data strobe to go low
            #endif
			message->data[counter] = (ubyte) P7;	// Read data from port
			WAIT = 1;	// Acknowledge with Wait going high
			WAIT = 0;	/* Wait down as quick as possible for next message.
						   Any wait state will keep wait high too long and make the ARCOM believe it is an
						   aknowledgment to the following data strobe. */
		}
	}

	if(!monTimer7 || !monTimer6){
		// timed out communicating with the ARCOM:
		// We don't want to send back garbage data (as in earlier versions)
		// but there is no way to return a value which prevents transmitting the buffer.
		
		// Yucky workaround, tell the caller it's actually a control msg:		
		message->dirn = CAN_CONTROL;
		message->len=0;
	} else {
		// no timeout; Set port to transmit data:
		DP7 = 0xFF;
	}

	/* Untrigger interrupt */
	INT = 0;
	return 0;
}

