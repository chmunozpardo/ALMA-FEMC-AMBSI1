/*!	\file	main.c
	\brief	AMBSI1 firmware code for the FEMC module

	This is the firmware to be loaded into the AMBSI1 to allow the AMBSI1 to work as a
	bridge between the CAN bus contained in the AMB and the ARCOM embedded controller
	handling the front end hardware.
*/
/* Defines */

//! Longest timeout allowed waiting for acknowledgement from ARCOM board
/*! During each phase of monitoring, a count-down timer counts from \p MAX_TIMEOUT
	down to zero unless an acknowledgment is received. */
#define MAX_TIMEOUT 470
// about 500 microseconds based on 0xFFFF = 70 ms

//! Timeout for each byte when we are already sending or receiving a packet.
#define EPP_BYTE_TIMEOUT 30
// About 32 microseconds

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
    requests available in the firmware. */

#define BASE_SPECIAL_MONITOR_RCA    0x20000L
#define GET_AMBSI1_VERSION_INFO     0x20000L    //!< Get the firmware version of this firmware.
#define GET_SETUP_INFO              0x20001L    //!< In versions 1.0.0 and 1.0.1 a monitor request to this initiates communication between the AMBSI1 and the ARCOM.
                                                //!< In version 1.2.x communication is established automatically at power-up.  This request still sends a reply for compatibility with ALMA and FETMS software.
#define GET_ARCOM_VERSION_INFO      0x20002L	//!< Get the ARCOM Pegasus firware version.
#define GET_SPECIAL_MONITOR_RCAS    0x20003L	//!< Get the special monitor RCA range from ARCOM. DEPRECATED
#define GET_SPECIAL_CONTROL_RCAS    0x20004L	//!< Get the special control RCA range from ARCOM. DEPRECATED
#define GET_MONITOR_RCAS            0x20005L	//!< Get the standard monitor RCA range from the ARCOM firmware.
#define GET_CONTROL_RCAS            0x20006L	//!< Get the standard control RCA range from the ARCOM firmware.
#define GET_LO_PA_LIMITS_TABLE_ESN  0x20010L    //!< 0x20010 through 0x20019 return the PA LIMITS table ESNs.
#define GET_MON_TIMERS1_RCA         0x20020L    //!< Get monitor timing countdown registers 1-4.
#define GET_MON_TIMERS2_RCA         0x20021L    //!< Get monitor timing countdown registers 5-8.

/* Version Info */
#define VERSION_MAJOR 01	//!< Major Version
#define VERSION_MINOR 02	//!< Minor Revision
#define VERSION_PATCH 04	//!< Patch Level

/* Uses serial port */
#include <reg167.h>
#include <intrins.h>

/* include library interface */
#include "..\libraries\amb\amb.h"
#include "..\libraries\ds1820\ds1820.h"

/* memset, memcpy */
#include <string.h>

/* Set aside memory for the callbacks in the AMB library */
static CALLBACK_STRUCT idata cb_memory[9];

/* CAN message callbacks */
int ambient_msg(CAN_MSG_TYPE *message); 	//!< Called to get the board temperature temperature
int controlMsg(CAN_MSG_TYPE *message);  	//!< Called to handle CAN control messages
int monitorMsg(CAN_MSG_TYPE *message);  	//!< Called to handle CAN monitor messages 
int getSetupInfo(CAN_MSG_TYPE *message);  	//!< Called to get the AMBSI1 <-> ARCOM link/setup information 
int getVersionInfo(CAN_MSG_TYPE *message);	//!< Called to get firmware version informations 
int getMonTimers1(CAN_MSG_TYPE *message);    //!< Retrieve last monitor message timers
int getMonTimers2(CAN_MSG_TYPE *message);    //!< Retrieve last monitor message timers

/* implementation helpers */
int implMonitorSingle(CAN_MSG_TYPE *message, unsigned char sendReply);

/* A global for the last read temperature */
static ubyte idata ambient_temp_data[4];

/* External bus control signal buffer chip enable is on P4.7 */
sbit  DISABLE_EX_BUF	= P4^7;

/* ARCOM Parallel port connection lines */
sbit  EPPC_NWRITE       = P2^2;
sbit  EPPC_NDATASTROBE  = P2^3;
sbit  SPPC_INIT         = P2^5;
sbit  SPPC_NSELECT      = P2^6;
sbit  EPPS_INTERRUPT    = P2^7;   // output
sbit  EPPS_NWAIT        = P2^8;   // output
sbit  SPPS_SELECTIN     = P2^10;  // output

/* Separate timers for each phase of monitor and control transaction */
static unsigned int idata monTimer1, monTimer2, monTimer3, monTimer4, monTimer5, monTimer6, monTimer7,
                          cmdTimer1, cmdTimer2, cmdTimer3, cmdTimer4, cmdTimer5, cmdTimer6;

/* Macros to implement EPP handshake */
// Wait for Data Strobe to go low
#define IMPL_HANDSHAKE(TIMER, TIMEOUT) { \
    for(TIMER = MAX_TIMEOUT; TIMER && EPPC_NDATASTROBE; TIMER--) {} \
    TIMEOUT = !TIMER; }

#define EPP_BYTE_HANDSHAKE(TIMER, TIMEOUT) { \
    for(TIMER = EPP_BYTE_TIMEOUT; TIMER && EPPC_NDATASTROBE; TIMER--) {} \
    TIMEOUT = !TIMER; }

/* Macro to toggle WAIT high then low */
#define TOGGLE_NWAIT { EPPS_NWAIT = 1; EPPS_NWAIT = 0; }

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
/*! Takes care of initializing the AMBSI1, the CAN subrutine and globally enables interrupts.
    version 1.2.0: also performs AMBSI1 to ARCOM link setup. */ 
void main(void) {
    unsigned long timer;

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
	SPPS_SELECTIN=1;
	DP2=0x0580; 

	/* Register callbacks for CAN events (RCA -> 0x20001) */
	if (amb_register_function(GET_SETUP_INFO, GET_SETUP_INFO, getSetupInfo) != 0)
		return;

    /* Register callbacks for monitor timers (RCA -> 0x20010, 0x20011) */
    if (amb_register_function(GET_MON_TIMERS1_RCA, GET_MON_TIMERS1_RCA, getMonTimers1) != 0)
        return;

    if (amb_register_function(GET_MON_TIMERS2_RCA, GET_MON_TIMERS2_RCA, getMonTimers2) != 0)
        return;

	/* globally enable interrupts */
  	amb_start();

	/* Wait for readiness status with ARCOM board.
	 * Effectively this does nothing because the line is uncontrolled during boot up. */
	ready=0;
	while(SPPC_INIT){ // Wait of init line to go to 0. In the mean time read the temperature
		ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
	}
    ready=1;

    /* Loop until the AMBSI1 to ARCOM link is established */
    while(!initialized) {
        /* Process a fake GET_SETUP_INFO request */
        myCANMessage.dirn=CAN_MONITOR;
        myCANMessage.len=0;
        myCANMessage.relative_address=GET_SETUP_INFO;
        if(getSetupInfo(&myCANMessage)) {
            // if timed out, sleep a bit:
            for(timer = 100000L; timer; timer--) {}   // about 0.1 second
        }
    }

    SPPS_SELECTIN = 0; // Select line to 0

	/* Never return */
	while (1) {
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


/*! return the timers for phases 1 through 4 of the last monitor request handled. */
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

/*! return the timers for phases 5, 6, 7 of the last monitor request handled.
    the fourth byte is the starting countdown value MAX_TIMEOUT */
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
	   	- 0x05 -> No Error. Previous setup completed successfully.
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
	if(implMonitorSingle(&myCANMessage, FALSE)){ // Send the monitor request.
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
    if(implMonitorSingle(&myCANMessage, FALSE)){ // Send the monitor request.
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
    if(implMonitorSingle(&myCANMessage, FALSE)){ // Send the monitor request.
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
    if(implMonitorSingle(&myCANMessage, FALSE)){ // Send the monitor request.
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

/*! structures used in controlMsg */
static struct {
    unsigned long RCA;
    unsigned char dataLen;
    unsigned char data[8];
} idata controlRequest;

/*! This function will be called in case a CAN control message is received.
	It will start communication with the ARCOM board triggering the parallel port
	interrupt and the sending the CAN message information to the ARCOM board.

	Since a CAN control request doesn't require any aknowledgment, this function
	will then return.

	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int controlMsg(CAN_MSG_TYPE *message){

	unsigned char i, timeout;
	unsigned int timer;

	if(message->dirn==CAN_MONITOR){
		monitorMsg(message);
		return 0;
	}

    // C166 is little endian, same as CAN
	controlRequest.RCA = message -> relative_address;
	controlRequest.dataLen = message -> len;
	memset(&(controlRequest.data), 0, 8);
    memcpy(&(controlRequest.data), message -> data, 8);

	/* Trigger interrupt */
	EPPS_INTERRUPT = 1;

	/* Send controlRequest packet */
    IMPL_HANDSHAKE(timer, timeout);
    if (!timeout) {
        for (i = 0; i < sizeof(controlRequest); i++) {
            EPP_BYTE_HANDSHAKE(timer, timeout)
            P7 = (uword) ((unsigned char *) &controlRequest) + i;
            TOGGLE_NWAIT;
        }
    }

	/* Untrigger interrupt */
	EPPS_INTERRUPT = 0;

	return 0;
}

/*! structures used in implMonitorSingle */
static struct {
    unsigned long RCA;
    unsigned char dataLen;
} idata monitorRequest;

static struct {
    unsigned char dataLen;
    unsigned char data[8];
} idata monitorResponse;

/*! Implementation of one monitor transaction.
    Abstracted out so that monitorMsg below can retry

    \param  *message    a CAN_MSG_TYPE
    \param  sendReply   TRUE to send CAN replies.  FALSE to suppress them for messages sent in getSetupInfo().
    \return
        - 0 -> Everything went OK
        - -1 -> Time out during CAN message forwarding */

int implMonitorSingle(CAN_MSG_TYPE *message, unsigned char sendReply) {
    unsigned char i, timeout;

    // C166 is little endian, same as CAN
    monitorRequest.RCA = message -> relative_address;
    monitorRequest.dataLen = 0;

    /* Trigger interrupt */
    EPPS_INTERRUPT = 1;

    /* Send monitorRequest packet */
    IMPL_HANDSHAKE(monTimer1, timeout);
    if (!timeout) {
        for (i = 0; i < sizeof(monitorRequest); i++) {
            EPP_BYTE_HANDSHAKE(monTimer2, timeout)
            P7 = (uword) ((unsigned char *) &monitorRequest) + i;
            TOGGLE_NWAIT;
        }

        /* Set port to receive data */
        DP7 = 0x00;
        _nop_();
        _nop_();

        /* Receive response */
        IMPL_HANDSHAKE(monTimer3, timeout);
        if (!timeout) {
            for (i = 0; i < sizeof(monitorResponse); i++) {
                EPP_BYTE_HANDSHAKE(monTimer4, timeout)
                *((unsigned char *) &monitorResponse) = (ubyte) P7;
                TOGGLE_NWAIT;
            }
        }
    }

    //Set port to transmit data:
    DP7 = 0xFF;
    _nop_();
    _nop_();
    /* Untrigger interrupt */
    EPPS_INTERRUPT = 0;

    if (!timeout) {
        message -> len = monitorResponse.dataLen;
        memcpy(message -> data, &(monitorResponse.data), 8);
    }

    /* Handle timeout or reply suppressed */
    if (timeout || !sendReply) {
        // timed out communicating with the ARCOM:
        // We don't want to send back garbage data (as in earlier versions)
        // but there is no way to return a value which prevents transmitting the buffer.
        
        // Yucky workaround, tell the caller it's actually a control msg:       
        message->dirn = CAN_CONTROL;
        message->len=0;
    }
    if (timeout)
        return -1;
    else
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
    int ret = 0;

	if(message->dirn==CAN_CONTROL){
		controlMsg(message);
		return 0;
	}

    // Try 1:
    ret = implMonitorSingle(message, TRUE);

    if (ret != 0)
        // Retry once:
        ret = implMonitorSingle(message, TRUE);

	return ret;
}

