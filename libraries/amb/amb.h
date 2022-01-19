/*
 ****************************************************************************
 #
 # Copyright (C) 1999-2022
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: feic_na@nrao.edu
 ****************************************************************************
 */
/**
 ****************************************************************************
 *  AMB.H 
 *
 *  Header file for ALMA Monitor and Control Bus Slave library.  This
 *  file makes use of libraries for the support of Dallas Semiconductor device
 *  used for a unique serial number.  It is also intended to be the basis for
 *  slave implementations on hardware other than the suggested interface.
 *
 *****************************************************************************
 */

#ifndef AMB_H
	#define AMB_H
	/* 
	 * Some definitions of variable sizes to be used, which depend on
	 * the micro architecture 
	 */
	#ifdef C167_ARCH

		#define ulong unsigned long
		#define uword unsigned int
		#define ubyte unsigned char

	#elif PIC_ARCH

		#define ulong unsigned long
		#define uword unsigned int
		#define ubyte unsigned char

	#endif /* ARCHITECTURE SWITCH */

	#ifndef TRUE

		#define TRUE  1
		#define FALSE 0

	#endif /* TRUE */

	/* Define internal slave error codes */
	#define DUP_SLAVE_ADDR_E	0x01	/* Duplicate slave address detected */
	#define NO_DS1820_E			0x02	/* No DS1820 device found */
	#define NO_SN_E				0x03	/* No serial number read */
	#define ONEWIRE_CRC_E		0x04	/* CRC error on a 1-Wire bus transaction */

	/* An enum for CAN message direction */
	typedef enum {	CAN_MONITOR,
					CAN_CONTROL
	} CAN_DIRN_TYPE;

	/* Configuration and current data structure for CAN messages */
	typedef struct {
		ulong				relative_address;	/* CAN ID - Slave base address */
		ubyte				data[8];			/* Current value */
		ubyte				len;				/* Amount of data */
		CAN_DIRN_TYPE		dirn;				/* Direction of message */
	} CAN_MSG_TYPE;

	/* Callback function typedef */
	typedef int(*read_or_write_func)(CAN_MSG_TYPE *message);

	/* Callback info */
	typedef struct {
		ulong				low_address;	/* First RA in range */
		ulong				high_address;	/* Last RA in range */
		read_or_write_func	cb_func;		/* Function to call when message in range */
	} CALLBACK_STRUCT;

	/*
	 ***************************************************************************
	  Prototypes of global functions
	 ***************************************************************************
	 */

	/**
	 * Initialise slave node.  This routine must be called.  It reads the node
	 * address and serial number and configures the CAN interface.  The user should
	 * enable interrupts AFTER calling this routine.  The void * parameter should
	 * contain the starting address of an array of CALLBACK_STRUCT.  This array
	 * should contain enough elements for all of the functions to be registered.
	 */
	extern int amb_init_slave(void *cb_ops_memory);

	/**
	 * Register a callback function to be called when a CAN message with a 
	 * relative address between low_address and high_address is received.
	 * The callback is passed a pointer to a CAN_MSG_TYPE structure
	 * Note that if the message is a monitor message, the called function
	 * should set the message data length and data bytes before returning 
	 */
	extern int amb_register_function(ulong low_address, ulong high_address, read_or_write_func func);

	/**
	 * Unregister last registered function if any. This allows to roll back
     * in case of error during the registration of the callback functions.
     */
	extern int amb_unregister_last_function(void);

	/**
	 * Start handling CAN interrupts. Currently this routine enables all
	 * interrupts on the C167. 
	 */
	extern int amb_start();

#endif /* AMB_H */

