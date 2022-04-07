/*!	\file	main.c
	\brief	AMBSI1 firmware code for the FEMC module

	This is the firmware to be loaded into the AMBSI1 to allow the AMBSI1 to work as a
	bridge between the CAN bus and the ARCOM embedded controller handling the front end hardware.
	All CAN messages are forwarded to the ARCOM over GPIO pins on JP7 set up as an ISA parallel port.
*/
/* Defines */


//! Is the firmware using the 48 ms pulse?
/*! Defines if the 48ms pulse is used to trigger the corresponding interrupt.
	If yes then P8.0 will not be available for use as a normal I/O pin since
	the assigned pin on the C167 will be jumpered to receive the 48ms pulse.

	\note	To be able to use the 48 ms pulse, it is necessary to program the
	 	  	Xilinx chip to allow the incoming pulse to be passed through. */
// #define USE_48MS

#define MAX_CAN_MSG_PAYLOAD			8		// Max CAN message payload size. Used to determine if error occurred

//! \b 0x20000 -> Base address for the special monitor RCAs
/*! This is the starting relative CAN address for the special monitor
    requests available in the firmware. */

#define BASE_SPECIAL_MONITOR_RCA    0x20000L
#define GET_AMBSI1_VERSION_INFO     0x20000L    //!< Get the version of this firmware.
#define GET_SETUP_INFO              0x20001L    //!< In versions 1.0.0 and 1.0.1 a monitor request to this initiates communication between the AMBSI1 and the ARCOM.
                                                //!< Since version 1.2.x communication is established automatically at power-up.
                                                //!< This request still sends a reply for compatibility with ALMA and FETMS software.
#define GET_ARCOM_VERSION_INFO      0x20002L	//!< Get the ARCOM Pegasus firmware version.
#define GET_SPECIAL_MONITOR_RCAS    0x20003L	//!< Get the special monitor RCA range from ARCOM.
                                                //!< DEPRECATED in the FE ICD but still used by this app to set up ISR callbacks
#define GET_SPECIAL_CONTROL_RCAS    0x20004L	//!< Get the special control RCA range from ARCOM.
                                                //!< DEPRECATED in the FE ICD but still used by this app to set up ISR callbacks
#define GET_MONITOR_RCAS            0x20005L	//!< Get the standard monitor RCA range from the ARCOM firmware.
                                                //!< DEPRECATED in the FE ICD but still used by this app to set up ISR callbacks
#define GET_CONTROL_RCAS            0x20006L	//!< Get the standard control RCA range from the ARCOM firmware.
                                                //!< DEPRECATED in the FE ICD but still used by this app to set up ISR callbacks
#define GET_LO_PA_LIMITS_TABLE_ESN  0x20010L    //!< 0x20010 through 0x20019 return the PA LIMITS table ESNs.

// We carve out some of the special monitor RCAs for timers and debugging of this firmware:
#define BASE_AMBSI1_RESERVED        0x20020L    //!< Lowest special RCA served by this firmware not forwarded to ARCOM.
#define GET_MON_TIMERS1_RCA         0x20020L    //!< Get monitor timing countdown registers 1-4.
#define GET_MON_TIMERS2_RCA         0x20021L    //!< Get monitor timing countdown registers 5-7 and the value of EPP_MAX_TIMEOUT.
#define GET_CMD_TIMERS1_RCA         0x20022L    //!< Get command timing countdown registers 1-4.
#define GET_CMD_TIMERS2_RCA         0x20023L    //!< Get command timing countdown registers 5-6 and the value of EPP_MAX_TIMEOUT.
#define GET_PPORT_STATE             0x20024L    //!< Get the state of the parallel port lines and other state info
#define GET_QUEUE_STATE             0x20025L    //!< Get the queueWritePos, queueReadPos, and QUEUE_SIZE
#define INSPECT_QUEUE_HEAD_RCA      0x20026L    //!< Return the RCA last written to the queue
#define INSPECT_QUEUE_HEAD_DATA     0x20027L    //!< Return the data bytes last written to the queue
#define INSPECT_QUEUE_HEAD_LEN_DIRN 0x20028L    //!< Return the length (1 byte) and message direction (2 bytes) last written to the queue
#define INSPECT_CURRENT_MON_HEAD    0x20029L    //!< Return the last RCA monitor sent to the ARCOM and length, timeout of received data
#define INSPECT_CURRENT_MON_DATA    0x2002AL    //!< Return the data received from the last RCA monitor sent to the ARCOM
#define LAST_AMBSI1_RESERVED        0x2003FL    //!< Highest special RCA served by this firmware not forwarded to ARCOM.

/* Version Info */
#define VERSION_MAJOR 01    //!< Major Version
#define VERSION_MINOR 03    //!< Minor Revision
#define VERSION_PATCH 04    //!< Patch Level

/* Uses GPIO ports */
#include <reg167.h>
#include <intrins.h>
#include <string.h>

/* include library interfaces */
#include "..\libraries\amb\amb.h"
#include "..\libraries\ds1820\ds1820.h"

/* Set aside memory for the callbacks in the AMB library
   This is larger than the number of handlers because some handlers get registered for more than one range.
   There should be a slot here for each call to amb_register_function() in this program.
   This being too small caused a buffer overflow in 1.2.0 and before! */
static CALLBACK_STRUCT idata cb_memory[9];

/* A global CAN message used getting RCA ranges from ARCOM */
static CAN_MSG_TYPE idata myCANMessage;

/* CAN message queue */
#define QUEUE_SIZE 256
static CAN_MSG_TYPE near msgQueue[QUEUE_SIZE];  //!< circular buffer of CAN messages

static unsigned char idata queueWritePos = 0;   //!< next queue write position
static unsigned char idata queueReadPos = 0;    //!< next queue read position
static unsigned char idata prevReadPos = 0;     //!< previous queue read position.
static unsigned char idata queueOverflow = 0;   //!< latching detect queue overflow.
static unsigned char idata waitingArcom = 0;    //!< waiting for ARCOM monitor response status
static unsigned char idata monitorTimeout = 0;  //!< ARCOM monitor response timeout
static CAN_MSG_TYPE idata *currentArcomMonitor; //!< pointer to last monitor data sent to the ARCOM

// Copy a message into the queue at queueWritePos.
// It is critical that the memcpy() happens before ++queueWritePos.
// Queue has overflowed if the write position catches up to the read position.
#define QUEUE_WRITE(MSG_P) { \
    memcpy(msgQueue + queueWritePos, MSG_P, sizeof(CAN_MSG_TYPE)); \
    if (++queueWritePos == queueReadPos) queueOverflow = 1; }

// Get a pointer to the message in the queue at queueReadPos, or NULL if there is no new message.
// It is critical that the MSG_P assignment happens before the queueReadPos++
#define QUEUE_READ(MSG_P) { \
    if (queueReadPos == queueWritePos) MSG_P = NULL; else { \
        MSG_P = msgQueue + queueReadPos; \
        prevReadPos = queueReadPos++; }}

/* If there is an interrupt just before queueReadPos++,
 * then ++queueWritePos will cause queueOverflow == 1.
 *
 * If there is an interrupt while we are sending data over the parallel port
 * then timing of that could get disturbed causing EPP timeouts.
 */

/* forward declare CAN message callbacks and helpers */
int ambient_msg(CAN_MSG_TYPE *message);       //!< Called to read get the board temperature sensor
int messageHandler(CAN_MSG_TYPE *message);    //!< Callback from AMB lib to handle all CAN monitor and control messages
int implControlSingle(CAN_MSG_TYPE *message); //! helper function
void implMonitorSingle(CAN_MSG_TYPE *message); //! helper function
int getSetupInfo(CAN_MSG_TYPE *message);      //!< Called to get the AMBSI1 <-> ARCOM link/setup information
int getVersionInfo(CAN_MSG_TYPE *message);    //!< Called to get firmware version informations
int getReservedMsg(CAN_MSG_TYPE *message);    //!< Monitor timers and debugging info from this firmware

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

//! Longest timeout allowed waiting for acknowledgment from ARCOM board
/*! During each phase of monitoring, a count-down timer counts from EPP_MAX_TIMEOUT
    down to zero unless an acknowledgment is received. */

#define EPP_MAX_TIMEOUT 1000
// about 1 millisecond based on 0xFFFF = 70 ms
// This is intentionally much longer than it should ever take because recovery from timeouts is messy.

// Macro to implement handshake with ARCOM board:
// Wait for Data Strobe to go low
#define IMPL_HANDSHAKE(TIMER) for (TIMER = EPP_MAX_TIMEOUT; TIMER && EPPC_NDATASTROBE; TIMER--) {}

// Macro to rapidly toggle EPPS_NWAIT:
//   Acknowledge with Wait going high.
//   Then go low as quickly as possible for next message.
//   If we keep high too long it will make the ARCOM believe it is an acknowledgment to the following data strobe.
#define TOGGLE_NWAIT { EPPS_NWAIT = 1; EPPS_NWAIT = 0; }

/* Macros to use when debugging timers.
 * Sets the timer countdowns so that a subsequent call to GET_MON_TIMERS1_RCA etc.
 *   will show if a timeout occurred before a transaction phase showing 0xFFFF.
 * Not used in production code.
 */
#define RESET_MON_TIMERS { \
    monTimer1 = monTimer2 = monTimer3 = monTimer4 = monTimer5 = monTimer6 = monTimer7 = 0xFFFF; }

#define RESET_CMD_TIMERS { \
    cmdTimer1 = cmdTimer2 = cmdTimer3 = cmdTimer4 = cmdTimer5 = cmdTimer6 = 0xFFFF; }

/* RCA address ranges */
static unsigned long idata lowestMonitorRCA, highestMonitorRCA,
						   lowestControlRCA, highestControlRCA,
                           lowestSpecialMonitorRCA, highestSpecialMonitorRCA,
						   lowestSpecialControlRCA, highestSpecialControlRCA;

/* Globals to check for initialization of RCA */
static bit idata ready;			// This gets set to true after GPIO ports and CAN callbacks for the AMBSI board are initialized.
static bit idata initialized;	// This gets set to true after CAN callbacks for the ARCOM board are registered.

//! MAIN
/*! Takes care of initializing the AMBSI1, the CAN library and globally enables interrupts.
    Since version 1.2.0: also performs AMBSI1 to ARCOM link setup. */
void main(void) {
    CAN_MSG_TYPE idata *queueRead_p;
    unsigned long timer;

    // This gets set to true after GPIO ports and CAN calbacks for the AMBSI board are initialized:
    ready = FALSE;

    // This gets set to true after CAN callbacks for the ARCOM board are registered:
    initialized = FALSE;

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

	/* Initialise the AMB library */
	if (amb_init_slave((void *) cb_memory) != 0) 
		return;

    /* Register callback for the AMBSI1 ambient temperature sensor */
    if (amb_register_function(0x30003, 0x30003, ambient_msg) != 0)
        return;

	/* Register callback for CAN version information */
	if (amb_register_function(GET_AMBSI1_VERSION_INFO, GET_AMBSI1_VERSION_INFO, getVersionInfo) != 0)
		return;

	/* Register callback for special monitor requests reserved by this firmware */
	if (amb_register_function(BASE_AMBSI1_RESERVED, LAST_AMBSI1_RESERVED, getReservedMsg) != 0)
        return;

	/* Initialize ports for communication */
    DP7=0x00;   // receive
    DP7=0xFF;   // transmit
    P2=0x0000;  // zero the port
    DP2=0x0580; // set bits 7,8,10 to output (SPPC_INIT, EPPS_NWAIT, SPPS_SELECTIN)

    // Not ready
    SPPS_SELECTIN=1;

	/* Register callback for GET_SETUP_INFO monitor request. */
	if (amb_register_function(GET_SETUP_INFO, GET_SETUP_INFO, getSetupInfo) != 0)
		return;

	/* Read the ambient temperature once.  This will be read occasionally in the main loop below. */
	ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);

    /*
    *  P2.3 edge interrupt
    *  GPIO falling edge transition
    *  GPIO interrupt priority level(ILVL) = 13
    *  GPIO interrupt group level (GLVL) = 3
    */
    CCM0 = 0x2000;
    CC3IC = 0x0077;
    CC3IR = 0;

	/* globally enable interrupts */
    amb_start();

    // Signal that all callbacks for AMBSI1 are registered:
    ready=TRUE;

    /* Loop until the AMBSI1 to ARCOM link is established */
    while (!initialized) {
        /* Process a fake GET_SETUP_INFO request */
        myCANMessage.dirn=CAN_MONITOR;
        myCANMessage.len=0;
        myCANMessage.relative_address=GET_SETUP_INFO;
        if (getSetupInfo(&myCANMessage)) {
            // if timed out, sleep a bit:
            for (timer = 100000L; timer; timer--) {}   // about 0.1 second
        }
    }

    /* Signal on the Select-In line that the AMBSI1 is ready.
       Since ARCOM version 3.6.x, ARCOM no longer waits for this. */
    SPPS_SELECTIN = 0;

    /* set the count down timer for reading the ambient temperature */
    timer = 0;

    /* Never return */
    while (1) {
        QUEUE_READ(queueRead_p);
        if (queueRead_p) {
            // Forward control message to ARCOM:
            if (queueRead_p -> dirn == CAN_CONTROL) {
                implControlSingle(queueRead_p);

            // Forward queued monitor request to ARCOM:
            } else {
                // Change dirn back from CAN_MONITOR_QUEUED:
                queueRead_p -> dirn = CAN_MONITOR;
                // Try 1:
                implMonitorSingle(queueRead_p);
                while (waitingArcom);
                if (monitorTimeout) {
                    // Retry once:
                    implMonitorSingle(queueRead_p);
                    while (waitingArcom);
                }
                // Send the reply if successful:
                if (!monitorTimeout) {
                    amb_transmit_monitor(queueRead_p);
                }
            }
        } else {
            // Read the ambient temperature periodically.
            if (timer > 0) {
                timer--;

            } else {
                // returns 0 when all phases of the read are complete:
                if (ds1820_get_temp_async(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]) == 0) {
                    timer = 0xFFFFFF;  // Reset the timer - about 18 seconds
                }
            }
        }
    }
}

/*! This function will return the firmware version for the AMBSI1 board.
	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int getVersionInfo(CAN_MSG_TYPE *message) {
	message->data[0]=VERSION_MAJOR;
	message->data[1]=VERSION_MINOR;
	message->data[2]=VERSION_PATCH;
	message->len=3;
	return 0;
}

//! handle all the special monitor messages reserved for the AMBSI1 firmware.
//! These are to aid debugging
int getReservedMsg(CAN_MSG_TYPE *message) {
    int i;
    switch(message -> relative_address) {
        case GET_MON_TIMERS1_RCA:
            /*! return the timers for phases 1 through 4 of the last monitor request handled. */
            message -> data[0] = (unsigned char) (monTimer1 >> 8);
            message -> data[1] = (unsigned char) (monTimer1);
            message -> data[2] = (unsigned char) (monTimer2 >> 8);
            message -> data[3] = (unsigned char) (monTimer2);
            message -> data[4] = (unsigned char) (monTimer3 >> 8);
            message -> data[5] = (unsigned char) (monTimer3);
            message -> data[6] = (unsigned char) (monTimer4 >> 8);
            message -> data[7] = (unsigned char) (monTimer4);
            message -> len = 8;
            break;
        case GET_MON_TIMERS2_RCA:
            /*! return the timers for phases 5, 6, 7 of the last monitor request handled.
                the fourth byte is the starting countdown value MAX_TIMEOUT */
            message -> data[0] = (unsigned char) (monTimer5 >> 8);
            message -> data[1] = (unsigned char) (monTimer5);
            message -> data[2] = (unsigned char) (monTimer6 >> 8);
            message -> data[3] = (unsigned char) (monTimer6);
            message -> data[4] = (unsigned char) (monTimer7 >> 8);
            message -> data[5] = (unsigned char) (monTimer7);
            message -> data[6] = (unsigned char) (EPP_MAX_TIMEOUT >> 8);
            message -> data[7] = (unsigned char) (EPP_MAX_TIMEOUT);
            message -> len = 8;
            break;
        case GET_CMD_TIMERS1_RCA:
            /*! return the timers for phases 1 through 4 of the last command handled. */
            message -> data[0] = (unsigned char) (cmdTimer1 >> 8);
            message -> data[1] = (unsigned char) (cmdTimer1);
            message -> data[2] = (unsigned char) (cmdTimer2 >> 8);
            message -> data[3] = (unsigned char) (cmdTimer2);
            message -> data[4] = (unsigned char) (cmdTimer3 >> 8);
            message -> data[5] = (unsigned char) (cmdTimer3);
            message -> data[6] = (unsigned char) (cmdTimer4 >> 8);
            message -> data[7] = (unsigned char) (cmdTimer4);
            message -> len = 8;
            break;
        case GET_CMD_TIMERS2_RCA:
            /*! return the timers for phases 5, 6 of the last command handled.
                the fourth byte is the starting countdown value MAX_TIMEOUT */
            message -> data[0] = (unsigned char) (cmdTimer5 >> 8);
            message -> data[1] = (unsigned char) (cmdTimer5);
            message -> data[2] = (unsigned char) (cmdTimer6 >> 8);
            message -> data[3] = (unsigned char) (cmdTimer6);
            message -> data[5] = (unsigned char) 0;
            message -> data[4] = (unsigned char) 0;
            message -> data[6] = (unsigned char) (EPP_MAX_TIMEOUT >> 8);
            message -> data[7] = (unsigned char) (EPP_MAX_TIMEOUT);
            message -> len = 8;
            break;
        case GET_PPORT_STATE:
            // Return the parallel port control and status lines.
            message -> data[0] = (unsigned char) SPPC_NSELECT;
            message -> data[1] = (unsigned char) SPPS_SELECTIN;
            message -> data[2] = (unsigned char) SPPC_INIT;
            message -> data[3] = (unsigned char) EPPS_INTERRUPT;
            message -> data[4] = (unsigned char) DP7;
            message -> data[5] = (unsigned char) P7;
            message -> data[6] = (unsigned char) ready;
            message -> data[7] = (unsigned char) initialized;
            message -> len = 8;
            break;
        case GET_QUEUE_STATE:
            // Return the message queue indexes and size.
            message -> data[0] = (unsigned char) (queueWritePos);
            message -> data[1] = (unsigned char) (queueReadPos);
            message -> data[2] = (unsigned char) (prevReadPos);
            message -> data[3] = (unsigned char) (QUEUE_SIZE);
            message -> data[4] = (unsigned char) (queueOverflow);
            message -> data[5] = (unsigned char) 0;
            message -> data[6] = (unsigned char) 0;
            message -> data[7] = (unsigned char) 0;
            message -> len = 5;
            queueOverflow = 0; // reset overflow flag on read
            break;
        case INSPECT_QUEUE_HEAD_RCA:
            // Return the RCA from the message most recently read from the queue.
            message -> data[0] = ((unsigned char *) &msgQueue[prevReadPos].relative_address)[0];
            message -> data[1] = ((unsigned char *) &msgQueue[prevReadPos].relative_address)[1];
            message -> data[2] = ((unsigned char *) &msgQueue[prevReadPos].relative_address)[2];
            message -> data[3] = ((unsigned char *) &msgQueue[prevReadPos].relative_address)[3];
            message -> data[4] = (unsigned char) 0;
            message -> data[5] = (unsigned char) 0;
            message -> data[6] = (unsigned char) 0;
            message -> data[7] = (unsigned char) 0;
            message -> len = 4;
            break;

        case INSPECT_QUEUE_HEAD_DATA:
            // Return the data payload the message most recently read from the queue.
            for (i = 0; i < 8; i++) {
                if (i < msgQueue[prevReadPos].len)
                    message -> data[i] = msgQueue[prevReadPos].data[i];
                else
                    message -> data[i] = (unsigned char) 0;
            }
            message -> len = msgQueue[prevReadPos].len;
            break;

        case INSPECT_QUEUE_HEAD_LEN_DIRN:
            // Return the data length (uchar) and message direction (uint) from the message most recently read from the queue.
            message -> data[0] = ((unsigned char *) &msgQueue[prevReadPos].len)[0];
            message -> data[1] = ((unsigned char *) &msgQueue[prevReadPos].dirn)[0];
            message -> data[2] = ((unsigned char *) &msgQueue[prevReadPos].dirn)[1];
            message -> data[3] = (unsigned char) 0;
            message -> data[4] = (unsigned char) 0;
            message -> data[5] = (unsigned char) 0;
            message -> data[6] = (unsigned char) 0;
            message -> data[7] = (unsigned char) 0;
            message -> len = 3;
            break;

        case INSPECT_CURRENT_MON_DATA:
            // Return the data length (uchar) and message direction (uint) from the message most recently read from the queue.
            message->data[0] = currentArcomMonitor->data[0];
            message->data[1] = currentArcomMonitor->data[1];
            message->data[2] = currentArcomMonitor->data[2];
            message->data[3] = currentArcomMonitor->data[3];
            message->data[4] = currentArcomMonitor->data[4];
            message->data[6] = currentArcomMonitor->data[5];
            message->data[6] = currentArcomMonitor->data[6];
            message->data[7] = currentArcomMonitor->data[7];
            message->len = 8;
            break;
        case INSPECT_CURRENT_MON_HEAD:
            // Return the data length (uchar) and message direction (uint) from the message most recently read from the queue.
            message->data[0] = currentArcomMonitor->len;
            message->data[1] = (unsigned char)(currentArcomMonitor->dirn);
            message->data[2] = (unsigned char)(currentArcomMonitor->relative_address);
            message->data[3] = (unsigned char)(currentArcomMonitor->relative_address >> 8);
            message->data[4] = (unsigned char)(currentArcomMonitor->relative_address >> 16);
            message->data[5] = (unsigned char)(currentArcomMonitor->relative_address >> 24);
            message->data[6] = monitorTimeout;
            message->data[7] = 0;
            message->len = 8;
            break;

        default:
            message -> data[0] = (unsigned char) 0;
            message -> data[1] = (unsigned char) 0;
            message -> data[2] = (unsigned char) 0;
            message -> data[3] = (unsigned char) 0;
            message -> data[4] = (unsigned char) 0;
            message -> data[5] = (unsigned char) 0;
            message -> data[6] = (unsigned char) 0;
            message -> data[7] = (unsigned char) 0;
            message -> len = 0;
            break;
    }
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
int getSetupInfo(CAN_MSG_TYPE *message) {

	/* This can only handle a monitor message */
	if (message->dirn==CAN_CONTROL) {
		return -1;
	}
	
	/* Return message size: 1 byte */
	message->len = 1;

	/* If not ready to communicate, return the message and wait */
	if (!ready) {
		message->data[0]=0x06; // Error 0x06: communication between ARCOM and AMBSI not yet established
		return -1;
	}

	/* If already initialized do not initialize again */
	if (initialized) {
		message->data[0]=0x05; // No Error 0x05: Previous setup completed successfully.
		return -1;
	}

	/* SPECIAL MONITOR RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom CAN message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_MONITOR_RCAS; // 0x20003 -> RCA: special address to retrieve the special monitor RCAs informations
    implMonitorSingle(&myCANMessage); // Send the monitor request
    while (waitingArcom); // Wait for ARCOM response
    if (monitorTimeout) { // Check if the response was a timeout
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestSpecialMonitorRCA = 0;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialMonitorRCA = 0;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special monitor messages */
    amb_register_function(lowestSpecialMonitorRCA, BASE_AMBSI1_RESERVED - 1, messageHandler);
	amb_register_function(LAST_AMBSI1_RESERVED + 1, highestSpecialMonitorRCA, messageHandler);
	// 00020002 - 00020FFF

	/* SPECIAL CONTROL RCAs */
	/* Get the information on the available special control RCAs from the ARCOM board */
	/* Set up custom CAN message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_CONTROL_RCAS; // 0x20004 -> RCA: special address to retrieve the special control RCAs informations
    implMonitorSingle(&myCANMessage); // Send the monitor request
    while (waitingArcom); // Wait for ARCOM response
    if (monitorTimeout) { // Check if the response was a timeout
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously registered functions */
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestSpecialControlRCA = 0;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialControlRCA = 0;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callback for special control RCA messages */
    amb_register_function(lowestSpecialControlRCA, highestSpecialControlRCA, messageHandler);
	// 00021000 - 00021FFF

	/* MONITOR RCAs */
	/* Get the information on the available monitor RCAs from the ARCOM board */
	/* Set up custom CAN message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_MONITOR_RCAS; // 0x20005 -> RCA: special address to retrieve the monitor RCAs informations
    implMonitorSingle(&myCANMessage); // Send the monitor request
    while (waitingArcom); // Wait for ARCOM response
    if (monitorTimeout) { // Check if the response was a timeout
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously registered functions */
        amb_unregister_last_function(); // SPECIAL CONTROL RCAs
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestMonitorRCA = 0;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestMonitorRCA = 0;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for standard monitor messages */
	amb_register_function(lowestMonitorRCA, highestMonitorRCA, messageHandler);
	// 00000001 - 0000FFFF

	/* CONTROL RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom CAN message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_CONTROL_RCAS; // 0x20006 -> RCA: special address to retrieve the special control RCAs informations
    implMonitorSingle(&myCANMessage); // Send the monitor request
    while (waitingArcom); // Wait for ARCOM response
    if (monitorTimeout) { // Check if the response was a timeout
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously registered functions */
        amb_unregister_last_function(); // MONITOR RCAs
        amb_unregister_last_function(); // SPECIAL CONTROL RCAs
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestControlRCA = 0;
	highestControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestControlRCA = 0;
	lowestControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for standard control messages */
	amb_register_function(lowestControlRCA, highestControlRCA, messageHandler);
	// 00010000 - 0001FFFF

	/* No error */
	initialized = 1; // Remember that the ARCOM RCAs have already been registered
	message->data[0]=0;

    return 0;
}



/*! Return the temperature of the AMBSI as measured by the DS1820 onboard chip.

	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int ambient_msg(CAN_MSG_TYPE *message) {
    message->data[0] = ambient_temp_data[0];
    message->data[1] = ambient_temp_data[1];
    message->data[2] = ambient_temp_data[2];
    message->data[3] = ambient_temp_data[3];
    message->len = 4;
	return 0;
}


/* Triggers every 48ms pulse */
void received_48ms(void) interrupt 0x30{
// Put whatever you want to be execute at the 48ms clock.
// Remember that right now this interrupt has higher priority than the CAN.
// Also to be able to use the 48ms, the Xilinx has to be programmed to connect the
// incoming pulse on (pin31) to the cpu (pin28).
}


/*! This function is registered as the callback for all monitor and control messages.
    It queues the message and returns immediately.

    \param  *message    a CAN_MSG_TYPE
    \return 0 - Everything went OK */
int messageHandler(CAN_MSG_TYPE *message) {
    // If this is a monitor request change it to CAN_MONITOR_QUEUED so that the AMB library will NOT send a reply now.
    if (message -> dirn == CAN_MONITOR)
        message -> dirn = CAN_MONITOR_QUEUED;
    // Stuff it in the queue and return:
    QUEUE_WRITE(message);
    return 0;
}

/*! Implementation of one command transaction.

    It will start communication with the ARCOM board triggering the parallel port
    interrupt and the sending the CAN message information to the ARCOM board.

    Since a CAN control request doesn't require any acknowledgment, this function
    will then return.

    \param  *message    a CAN_MSG_TYPE
    \return 0 - Everything went OK */
int implControlSingle(CAN_MSG_TYPE *message) {
    unsigned char counter;

    /* Trigger interrupt */
    EPPS_INTERRUPT = 1;

    //Uncomment to debug timers:
    //RESET_CMD_TIMERS

    // Set port to transmit data:
    DP7 = 0xFF;

    /* Send RCA */
    IMPL_HANDSHAKE(cmdTimer1)
    P7 = (uword) (message->relative_address);       // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host
    if (cmdTimer1 == 0) {
        // FAILED TO SEND
        // Untrigger interrupt:
        EPPS_INTERRUPT = 0;
        return -1;
    }

    IMPL_HANDSHAKE(cmdTimer2)
    P7 = (uword) (message->relative_address>>8);    // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    IMPL_HANDSHAKE(cmdTimer3)
    P7 = (uword) (message->relative_address>>16);   // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    IMPL_HANDSHAKE(cmdTimer4)
    P7 = (uword) (message->relative_address>>24);   // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    // Send payload size:
    IMPL_HANDSHAKE(cmdTimer5)
    P7 = message->len;                              // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    for (counter=0;counter<message->len;counter++) {
        IMPL_HANDSHAKE(cmdTimer6)
        P7 = message->data[counter];                // Put data on port
        TOGGLE_NWAIT;                               // Trigger the read by host
    }

    /* Untrigger interrupt */
    EPPS_INTERRUPT = 0;
    return 0;
}

/*! Implementation of partial monitor transaction.
    Sends the requested RCA to the ARCOM.
    Does not send the reply.

    \param  *message    a CAN_MSG_TYPE */
void implMonitorSingle(CAN_MSG_TYPE *message) {

    /* Trigger interrupt */
    EPPS_INTERRUPT = 1;
    currentArcomMonitor = message; // Set pointer to global current monitor request

    //Uncomment to debug timers:
    //RESET_MON_TIMERS

    // Set port to transmit data:
    DP7 = 0xFF;

    /* Send RCA */
    IMPL_HANDSHAKE(monTimer1)
    P7 = (uword) (message->relative_address);       // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host
    if (monTimer1 == 0) {
        // FAILED TO SEND
        // Untrigger interrupt:
        EPPS_INTERRUPT = 0;
        return;
    }

    IMPL_HANDSHAKE(monTimer2)
    P7 = (uword) (message->relative_address>>8);    // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    IMPL_HANDSHAKE(monTimer3)
    P7 = (uword) (message->relative_address>>16);   // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    IMPL_HANDSHAKE(monTimer4)
    P7 = (uword) (message->relative_address>>24);   // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    /* Send payload size (0 -> monitor message) */
    IMPL_HANDSHAKE(monTimer5)
    P7 = message->len;                              // Put data on port
    TOGGLE_NWAIT;                                   // Trigger the read by host

    /* Set port to receive data */
    DP7 = 0x00;
    waitingArcom = TRUE;                            // Set status to waiting for ARCOM response
    return;
}

/*! Implementation of partial monitor transaction.
    Receives the requested RCA data from the ARCOM.
    It uses GPIO2.3 falling edge transition to start data flow.
    Does not send the reply. */
void recvMonitorSingle() interrupt CC3INT = 0x13 {
    unsigned char counter;
    if (!waitingArcom) return;                      // Check if we are waiting for ARCOM response
    currentArcomMonitor->len = (ubyte)P7;           // Read data from port
    TOGGLE_NWAIT;                                   // Trigger next write by host

    /* Error receiving payload size */
    monitorTimeout = FALSE;
    if (currentArcomMonitor->len > MAX_CAN_MSG_PAYLOAD) {
        DP7 = 0xFF;
        EPPS_INTERRUPT = 0;
        monitorTimeout = TRUE;                      // Set timeout status
        waitingArcom = FALSE;                       // Finished waiting for ARCOM
        return;                                     // Exit from ISR
    }

    /* Get the payload */
    for (counter = 0; !monitorTimeout && (counter < currentArcomMonitor->len); counter++) {
        IMPL_HANDSHAKE(monTimer7)
        currentArcomMonitor->data[counter] = (ubyte)P7;  // Read data from port
        TOGGLE_NWAIT;                               // Trigger next write by host
        if (monTimer7 == 0) {
            monitorTimeout = TRUE;                  // Finished waiting for ARCOM
        }
    }
    // Set port to transmit data:
    DP7 = 0xFF;

    /* Untrigger interrupt */
    EPPS_INTERRUPT = 0;

    waitingArcom = FALSE;                           // Finished waiting for ARCOM
    return;
}

