/*
 *****************************************************************************
 #
 # Copyright (C) 1999-2021
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: feic_na@nrao.edu
 ****************************************************************************
 *
 *  AMB.C 
 *
 *  Implementation file for ALMA Monitor and Control Bus Slave library.  This
 *  file makes use of libraries for the support of Dallas Semiconductor device
 *  used for a unique serial number.  It is also intended to be the basis for
 *  slave implementations on hardware other than the suggested interface.
 *
 *****************************************************************************
 */

/* Version of AMB protocol */
#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 1
#define PROTOCOL_VERSION_PATCH 2

/* Version of SOFTWARE */
#define SW_VERSION_MAJOR 1
#define SW_VERSION_MINOR 3
#define SW_VERSION_PATCH 4
/* Version of HARDWARE */
#define HW_VERSION_MAJOR 1
#define HW_VERSION_MINOR 6

/* REVISION HISTORY */
/* Version 01.03.04 - Christopher Muñoz chmunozpardo@gmail.com
                      Reduce priority of CAN ISR from 13 to 12
 * Version 01.03.03 - Morgan McLeod mmcleod@nrao.edu
                      Separate monitor response from request handling to enable
                      message queueing in client code.
                      Async read ambient temperature
 * Version 01.01.02 - Released as Ver_1_1_2
           01.02.03   Patch by Andrea Vaccari - NRAO NTC
		   			  Changed code to assure that any RCA is not serviced more than once in
					  case the register addresses overlap
 * Version 01.02.02 - Fixed bug to allow use of NodeID at RCA 0. Jeff Kern
 * Version 01.01.01 - Released as Ver_1_1_1
		   			  Patch by Andrea Vaccari - NRAO NTC
		   			  Added the function "amb_unregister_last_function" declaration in the
		   			  amb.h file.
 * Version 01.01.00 - Released as Ver_1_1_0
					  Minor change by Andrea Vaccari - NRAO NTC
					  Added the function "amb_unregister_last_function".
					  If any callback function has been registered, this function will
                      unregister the last registered function.
 * Version 01.00.01 - Released as Ver_1_0_1
		              Patch by Andrea Vaccari - NRAO Tucson
      		          This patch modified the location of all the global variables
                      and moved them from external memory to internal memory (IDATA).
		              The result was an increased speed of execution.
 * Version 01.00.00 - Original Version developed by Mick Brooks - NRAO Tucson
*/



 /* Include C167 register definitions */

	#include <reg167.h>
	#include <intrins.h>

#include "amb.h"

/* Include Dallas Semiconductor support for C167 */

#include "..\..\libraries\ds1820\ds1820.h"


/* Definitions of CAN controller structure */

 /* For Siemens C167CR */

	/* Locations of CAN controller registers */
	#define C1CSR   (*((uword volatile sdata *) 0xEF00)) /* Control/Status Register */
	#define C1IR    (*((uword volatile sdata *) 0xEF02)) /* Interrupt Register */
	#define C1BTR   (*((uword volatile sdata *) 0xEF04)) /* Bit Timing Register */
	#define C1GMS   (*((uword volatile sdata *) 0xEF06)) /* Global Mask Short */
	#define C1UGML  (*((uword volatile sdata *) 0xEF08)) /* Upper Global Mask Long */
	#define C1LGML  (*((uword volatile sdata *) 0xEF0A)) /* Lower Global Mask Long */
	#define C1UMLM  (*((uword volatile sdata *) 0xEF0C)) /* Upper Mask of Last Message */
	#define C1LMLM  (*((uword volatile sdata *) 0xEF0E)) /* Lower Mask of Last Message */
	/* 
	 * Structure for a single 82527 CAN object
	 * A total of 15 such object structures exists
	 */

	struct can_obj {
	  uword  MCR;       /* Message Control Register */
	  uword  UAR;       /* Upper Arbitration Register */
	  uword  LAR;       /* Lower Arbitration Register */
	  ubyte  MCFG;      /* Message Configuration Register */
	  ubyte  Data[8];   /* Message Data 0 .. 7 */
	  ubyte  Customer;  /* Reserved for application specific data */
	} ;

	/* Location of first CAN object */
	#define CAN_OBJ ((struct can_obj volatile sdata *) 0xEF10)


/*
 ****************************************************************************
 * Interrupt Vectors
 ****************************************************************************
 */

#define XP0INT   0x40

/* Local Function prototypes */
static ubyte 	amb_get_node_address();
static int		amb_get_serial_number();
static int		amb_setup_CAN_hw();
static void		amb_handle_transaction();

/* All pertinent slave data */

	static struct slave_node {

	ubyte 		serial_number[8];	/* From hardware device */
	ubyte 		node_address;		/* From DIP switch */
	ulong		base_address;		/* From node address */

	ubyte		revision_level[3];	/* Protocol version */
	ubyte		sw_revision_level[3];	/* Software version */
	ubyte		hw_revision_level[2];	/* Hardware version */
	uword		num_errors;			/* Number of CAN errors */
	ubyte		last_slave_error;	/* Last internal slave error */
	ulong		num_transactions;	/* Number of completed transactions */

	ubyte		identify_mode;		/* True when responding to identify broadcast */

	ubyte		num_cbs;			/* No of callbacks registered */
	CALLBACK_STRUCT	*cb_ops;		/* User supplied callbacks */
} idata slave_node;

/* Structure for sharing message data with callbacks */

	static CAN_MSG_TYPE idata current_msg;



/* Initialise routine */
int amb_init_slave(void *cb_ops_memory){
/* Point to callback memory */
	slave_node.cb_ops = (CALLBACK_STRUCT *) cb_ops_memory;

/* Initially we have no registered callbacks */
	slave_node.num_cbs = 0;

/* Get the address of this slave from the hardware */
	slave_node.node_address = amb_get_node_address();

/* Calculate the base address */
	slave_node.base_address = ((ulong) (slave_node.node_address + 1)) * 262144;

/* Try to get the serial number from the hardware */
	if (amb_get_serial_number(slave_node.serial_number) != 0) {
		return -1;
	}

	/* Initialise counters and status */
	slave_node.revision_level[0] = PROTOCOL_VERSION_MAJOR;
	slave_node.revision_level[1] = PROTOCOL_VERSION_MINOR;
	slave_node.revision_level[2] = PROTOCOL_VERSION_PATCH;
	slave_node.sw_revision_level[0] = SW_VERSION_MAJOR;
	slave_node.sw_revision_level[1] = SW_VERSION_MINOR;
	slave_node.sw_revision_level[2] = SW_VERSION_PATCH;
	slave_node.hw_revision_level[0] = HW_VERSION_MAJOR;
	slave_node.hw_revision_level[1] = HW_VERSION_MINOR;
	slave_node.num_errors = 0;
	slave_node.last_slave_error = 0x0;
	slave_node.num_transactions = 0;

	slave_node.identify_mode = FALSE;
	
/* Setup the CAN hardware */
	return amb_setup_CAN_hw();
}

/* Register callback routine */
int amb_register_function(ulong low_address, ulong high_address, read_or_write_func func){
/* Store callback info */
	slave_node.cb_ops[slave_node.num_cbs].low_address = low_address;
	slave_node.cb_ops[slave_node.num_cbs].high_address = high_address;
	slave_node.cb_ops[slave_node.num_cbs].cb_func = func;

/* Increment the number of callbacks */
	slave_node.num_cbs++;

/* Always succeeds */
	return 0;
}

/* Unregister last registered callback routine */
int amb_unregister_last_function(void){

/* If no function is registered, nothing to be done. */
	if(!slave_node.num_cbs){
		return 0;
	}

/* Decrement the number of callbacks. */
	slave_node.num_cbs--;

	return 0;
}

/* Startup routine */
int amb_start(){
	IEN = 1;

/* Always succeeds */
	return 0;
}

/* Read the slave address */
ubyte amb_get_node_address(){
	#ifdef AMBSI  /* Standard i/f, DIP switch on Port 3.1 to 3.6 */
		return (P3 & 0x7e) >> 1;
	#endif /* AMBSI */

	#ifdef SK167  /* Starter Kit test board, DIP switch on Port 7.1 to 7.6 */
		return (P7 & 0x7e) >> 1;
	#endif /* SK167 */

}

/* Read the serial number from the Dallas Semiconductor device */
int amb_get_serial_number(){

		/* Initialise the 1-wire port (library defined) */
		if (ds1820_init() != 0) {
			slave_node.last_slave_error = NO_DS1820_E;
			return -1;
		}

		/* read the serial number */
		if (ds1820_get_sn(slave_node.serial_number) != 0) {
			slave_node.last_slave_error = NO_SN_E;
			return -1;
		} else {
			return 0;
		}


}

/* Routine to setup an Intel 82527-like controller */
int amb_setup_CAN_hw(){

		ulong LAR, UAR;

		/* Set up for the various arbitration registers */

		/* calculate lower mask */
  		LAR = 0x00000000;
  		LAR += (slave_node.base_address & 0x0000001f) << 11;  /* ID  4.. 0 */
  		LAR += (slave_node.base_address & 0x00001fe0) >>  5;  /* ID 12.. 5 */

		/* calculate the upper mask */
  		UAR = 0x00000000;
  		UAR += (slave_node.base_address & 0x001fe000) >>  5;  /* ID 13..20 */
  		UAR += (slave_node.base_address & 0x1fe00000) >> 21;  /* ID 21..28 */

		/*  ------------ CAN Control/Status Register -------------- 
  		 *  start the initialization of the CAN Module 
		 */
  		C1CSR  = 0x0041;  /* set INIT and CCE */

	  	/*  ------------ Bit Timing Register ---------------------
  		 *  baudrate =  1000.000 KBaud
  		 *	 there are 5 time quanta before sample point
  		 *	 there are 4 time quanta after sample point
  		 *	 the (re)synchronization jump width is 2 time quanta
		 */
  		C1BTR  = 0x3440;  /* set Bit Timing Register */
  		C1GMS  = 0xE0FF;  /* set Global Mask Short Register */
  		C1UGML = 0xFFFF;  /* set Upper Global Mask Long Register */
  		C1LGML = 0xF8FF;  /* set Lower Global Mask Long Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 1 ---------------------------
		 *  --- This message object is used to receive the global identify ---------
		 *  --- broadcast message on ID 0x00000000 ---------------------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 1 is valid
  		 *  enable receive interrupt
		 */
  		CAN_OBJ[0].MCR  = 0x5599;    /* set Message Control Register */

	  	/*
  	 	 * message direction is receive
  		 * extended 29-bit identifier
    	 */
  		CAN_OBJ[0].MCFG = 0x04;      /* set Message Configuration Register */

	  	CAN_OBJ[0].UAR  = 0x0000;    /* set Upper Arbitration Register */
  		CAN_OBJ[0].LAR  = 0x0000;    /* set Lower Arbitration Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 2 ---------------------------
		 *  --- This message object is used as a transmit object for the serial ----
		 *  --- number.  It can be RTR'd quickly from the master -------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 2 is valid
		 */
  		CAN_OBJ[1].MCR  = 0x5695;    /* set Message Control Register */

	  	/* 
		 * message direction is transmit
  		 * extended 29-bit identifier
  		 * 8 valid data bytes
   		 */
  		CAN_OBJ[1].MCFG = 0x8C;      /* set Message Configuration Register */

	  	CAN_OBJ[1].UAR  = UAR;	 /* set Upper Arbitration Register of Object 2 */
  		CAN_OBJ[1].LAR  = LAR;	 /* set Lower Arbitration Register of Object 2 */	

	  	CAN_OBJ[1].Data[0] = slave_node.serial_number[0];   /* set data byte 0 */
  		CAN_OBJ[1].Data[1] = slave_node.serial_number[1];   /* set data byte 1 */
  		CAN_OBJ[1].Data[2] = slave_node.serial_number[2];   /* set data byte 2 */
  		CAN_OBJ[1].Data[3] = slave_node.serial_number[3];   /* set data byte 3 */
  		CAN_OBJ[1].Data[4] = slave_node.serial_number[4];   /* set data byte 4 */
  		CAN_OBJ[1].Data[5] = slave_node.serial_number[5];   /* set data byte 5 */
  		CAN_OBJ[1].Data[6] = slave_node.serial_number[6];   /* set data byte 6 */
  		CAN_OBJ[1].Data[7] = slave_node.serial_number[7];   /* set data byte 7 */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 3 ---------------------------
		 *  --- This message object is used to transmit all monitor data back to ---
		 *  --- the master. --------------------------------------------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 3 is valid
   		 */
	  	CAN_OBJ[2].MCR  = 0x5695;    /* set Message Control Register */

	  	/* 
		 * message direction is transmit
  		 * extended 29-bit identifier
  		 * 0 valid data bytes
      	 */
  		CAN_OBJ[2].MCFG = 0x0C;      /* set Message Configuration Register */

	  	CAN_OBJ[2].UAR  = 0x0000;    /* set Upper Arbitration Register */
  		CAN_OBJ[2].LAR  = 0x0000;    /* set Lower Arbitration Register */
  		
	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Objects 4 to 14 --------------------
		 *  --- These objects are not used at present ------------------------------
  		 *  ------------------------------------------------------------------------
		 */
  	   	CAN_OBJ[3].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[4].MCR  = 0x5555;    /* set Message Control Register */
	  	CAN_OBJ[5].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[6].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[7].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[8].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[9].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[10].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[11].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[12].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[13].MCR  = 0x5555;    /* set Message Control Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 15 --------------------------
		 *  --- This object is used in Basic CAN mode to receive all M&C requests --
  		 *  ------------------------------------------------------------------------
  		 *  Message object 15 is valid
  		 *  enable receive interrupt
   		 */
  		CAN_OBJ[14].MCR  = 0x5599;    /* set Message Control Register */

	  	/* 
		 * message direction is receive
  		 * extended 29-bit identifier
   	 	 */
  		CAN_OBJ[14].MCFG = 0x04;      /* set Message Configuration Register */

		/* Set the mask so that only the upper 11 bits are compared with the incoming identifier.
		   This ensures that messages are correctly filtered for this slaves range of identifiers */
 	 	C1UMLM = 0xE0FF;           /* set Upper Mask of Last Message */
  		C1LMLM = 0x0000;           /* set Lower Mask of Last Message */

	  	CAN_OBJ[14].UAR  = UAR; /* set Upper Arbitration Register of Basic CAN object */
  		CAN_OBJ[14].LAR  = LAR; /* set Lower Arbitration Register of Basic CAN object */

	    /*
		 *  enable CAN interrupt
  		 *  CAN interrupt priority level(ILVL) = 12
  	 	 *  CAN interrupt group level (GLVL) = 3
    	 */
  		XP0IC = 0x0073;

	  	/* ------------ CAN Control/Status Register --------------
  		 *  reset CCE and INIT
  		 * enable interrupt generation from CAN Module
  		 * enable interrupt generation on a change of bit BOFF or EWARN
		 * No status interrupts!
   		 */
  		C1CSR = 0x000A;

	/* Always succeeds */
	return 0;
}

/*
 ****************************************************************************
 *  This is the interrupt service routine for the CAN controller.
 *  It is executed if:
 *  - the busoff or the error warning status is reached 
 *    (EIE is set)
 *  - a message has been sent or received successfully or a bus 
 *    error occurred (SIE is set)
 *  - the bit INTPND (interrupt pending) in one of the message
 *    object control-registers is set (at Tx or Rx)
 * 
 ****************************************************************************
 */

	void amb_can_isr(void) interrupt XP0INT{
  	uword uwIntID;
  	uword uwStatus;

	  	while (uwIntID = C1IR & 0x00ff) {
	    	switch (uwIntID & 0x00ff) {
	     		case 1:  /* Status Change Interrupt
    	     	     	 * The CAN controller has updated (not necessarily changed)
        	    	  	 * the status in the Control Register.
						 */

					uwStatus = C1CSR;
		            if (uwStatus & 0x8000) { /* if BOFF */
        		    	/* 
						 * Indicates when the CAN controller is in busoff state.
						 * Increment error counter
					 	 */
						slave_node.num_errors++;
					}

		            if (uwStatus & 0x4000) { /* if EWRN */
        		       /* 
						* Indicates that the least one of the error counters in the
                		* EML has reached the error warning limit of 96.
					 	* Increment the error counter
					 	*/
						slave_node.num_errors++;
            		}

 		           if (uwStatus & 0x0800) { /* if TXOK */
              			/* 
					 	 * Indicates that a message has been transmitted successfully
              	 		 * (error free and acknowledged by at least one other node). 
					 	 */
              			uwStatus &= 0xf7ff;
              			C1CSR = uwStatus;     /* reset TXOK */

				 		/* If we are responding to the identify broadcast, then we are done */
						if (slave_node.identify_mode == TRUE) {
							/* Turn status interrupts off */
							C1CSR = 0x000A;
							slave_node.identify_mode = FALSE;
						}
        		    }

		            if (uwStatus & 0x1000) { /* if RXOK */
        		      	/* Indicates that a message has been received successfully. */
              			uwStatus &= 0xefff;
		              	C1CSR = uwStatus;     /* reset RXOK */
        		    }

		            if (uwStatus & 0x0700) { /* if LEC */
        			    switch ((uwStatus & 0x0700) >> 8) { /* LEC (Last Error Code) */
               				case 1: /* Stuff Error
                        			 * More than 5 equal bits in a sequence have occurred
			                         * in a part of a received message where this is not
                        			 * allowed. 
									 */
								/* Increment error counter */
								slave_node.num_errors++;
	               				break;

			               case 2: /* Form Error
            			            * A fixed format part of a received frame has the
                        			* wrong format. 
									*/
								/* Increment error counter */
				 			 	slave_node.num_errors++;
        			          	break;

			               case 3: /* Ack Error
            			            * The message this CAN controller transmitted was
                        			* not acknowledged by another node. 
									*/
								/* Increment error counter */
								slave_node.num_errors++;
                  				break;

			               case 4: /* Bit1 Error
                        			* During the transmission of a message (with the
                        			* exception of the arbitration field), the device
                        			* wanted to send a recessive level ("1"), but the
                        			* monitored bus value was dominant.
									*/
								/* Increment error counter */
					 			slave_node.num_errors++;

								/* 
								 * If we are responding to an identify request, this means a
								 * duplicate slave address was transmitted simultaneously
								 */
								if (slave_node.identify_mode == TRUE) {
									slave_node.last_slave_error = DUP_SLAVE_ADDR_E;
								}
				                break;

			               case 5: /* Bit0 Error
            			            * During the transmission of a message (or acknow-
                        			* ledge bit, active error flag, or overload flag),
                        			* the device wanted to send a dominant level ("0"),
                        			* but the monitored bus value was recessive. During
                        			* busoff recovery this staus is set each time a
                        			* sequence of 11 recessive bits has been monitored.
                        			* This enables the CPU to monitor the proceeding of
                        			* the busoff recovery sequence (indicating the bus
                        			* is not stuck at dominant or continously disturbed).
									*/
				               	if (uwStatus & 0x8000) { /* if Busoff status */
									/* Increment error counter */
						 			slave_node.num_errors++;
                  				} else {
									/* Increment error counter */
									slave_node.num_errors++;
                  				}
                  				break;

			               case 6: /* CRC Error
                        			* The CRC check sum was incorrect in the message
                        			* received.
									*/
								/* Increment error counter */
					 			slave_node.num_errors++;
                  				break;

			               default:
                  				break;
               			}
            		}
            		break;

				case 2: /* Message Object 15 Interrupt */
    	     		if ((CAN_OBJ[14].MCR & 0x0c00) == 0x0800) { /* if MSGLST set */
	        	    	/* 
						 * Indicates that the CAN controller has stored a new
   		     	    	 * message into object 15, while NEWDAT was still set,
        	    	 	 * ie. the previously stored message is lost.
					 	 */
           			 	CAN_OBJ[14].MCR = 0xf7ff;    /* reset MSGLST */

						/* 
						 * Messages in this object are probably M&C data, so 
						 * do something wih them  Increment error, because we missed
						 * a message 
						 */
						slave_node.num_errors++;

						if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
							amb_handle_transaction();
						}
    	     		} else {
        	       		/* 
						 * The CAN controller has stored a new message
        	   	     	 * into this object.
						 * Messages in this object are probably M&C data, so
						 * do something wih them.
						 */
						if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
							amb_handle_transaction();
						}
            		}
           			CAN_OBJ[14].MCR = 0x7dfd;      /* release buffer */
            		break;

				case 3: /* Message Object 1 Interrupt */
        		 	if ((CAN_OBJ[0].MCR & 0x0300) == 0x0200) {    /* if NEWDAT set */
             		 	if ((CAN_OBJ[0].MCR & 0x0c00) == 0x0800) { /* if MSGLST set */
               				/* 
					 		 * Indicates that the CAN controller has stored a new
               				 * message into this object, while NEWDAT was still set,
               				 * ie. the previously stored message is lost. 
							 */

			               	CAN_OBJ[0].MCR = 0xf7ff;  /* reset MSGLST */

							/* We are responding to the identify broadcast */
							slave_node.identify_mode = TRUE;

							/* Turn status interrupts on */
							C1CSR = 0x000E;

					  		/* Send the serial number in message object 2 */
							slave_node.num_transactions++;
							CAN_OBJ[1].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */

							/* This is an error, because we missed a message */
							slave_node.num_errors++;
        		        } else {
                			/* 
							 * The CAN controller has stored a new message
                 		  	 * into this object.
						 	 */

							/* We are responding to the identify broadcast */
							slave_node.identify_mode = TRUE;

							/* Turn status interrupts on */
							C1CSR = 0x000E;

							/* Send the serial number */
							slave_node.num_transactions++;
							CAN_OBJ[1].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */
		                }
					
						CAN_OBJ[0].MCR = 0xfdfd;  /* reset NEWDAT, INTPND */
         			}
	            	break;
	     		default:
    		        break;
			}
		}
	}

/* Routine to check if a callback should be run */

void amb_handle_transaction(){
	ulong incoming_ID;
	ubyte i;
	/* Get incoming ID from Object 15 */
	incoming_ID = 0x0;
  
	    incoming_ID += ((ulong) (CAN_OBJ[14].LAR & 0xf800)) >> 11;  /* ID  4.. 0 */
   		incoming_ID += ((ulong) (CAN_OBJ[14].LAR & 0x00ff)) <<  5;  /* ID 12.. 5 */
   		incoming_ID += ((ulong) (CAN_OBJ[14].UAR & 0xff00)) <<  5;  /* ID 13..20 */
  		incoming_ID += ((ulong) (CAN_OBJ[14].UAR & 0x00ff)) << 21;  /* ID 21..28 */

	/* Calculate relative address from base address */
	current_msg.relative_address = incoming_ID - slave_node.base_address;
 	/* Ignore messages that are outside our range (>3FFFF OR <0)*/
	if ((current_msg.relative_address > 262143) ||
		(current_msg.relative_address < 0))
		return;
 
	/* Get the message length */

		current_msg.len = (CAN_OBJ[14].MCFG & 0xf0) >> 4;
	/* This is a monitor request if data length is zero */
	if (current_msg.len != 0) {
		current_msg.dirn = CAN_CONTROL;
		/* Control message: get the data */
		for (i=0; i<current_msg.len; i++)
				current_msg.data[i] = CAN_OBJ[14].Data[i];
		switch (current_msg.relative_address) {	
			case 0x31000: /*Device or software reset */
				_trap_ (0x00);
				return;
				break;
			case 0x31001: /* Software reset */
				_trap_ (0x00);
				return;
				break;
		}
	} else {
			current_msg.dirn = CAN_MONITOR;
			/* Check for common monitor points */
		switch (current_msg.relative_address ) {

 			case 0x000: /* Slave hardware revision level */
				/* In order to avoid confusion between the interrupts I use message 
				   number 2 to respond to this request.
				   Added JSK 10/06/2005 */
				/* We are responding to the identify broadcast */
				slave_node.identify_mode = TRUE;

				/* Turn status interrupts on */
				C1CSR = 0x000E;

				/* Send the serial number */
				slave_node.num_transactions++;
				CAN_OBJ[1].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */
				return;
				break;

			case 0x30000: /* Slave protocol revision level */
				current_msg.len = 3;
				current_msg.data[0] = (ubyte) (slave_node.revision_level[0]);
				current_msg.data[1] = (ubyte) (slave_node.revision_level[1]);
				current_msg.data[2] = (ubyte) (slave_node.revision_level[2]);
				amb_transmit_monitor(&current_msg);
				slave_node.num_transactions++;
				return;
				break;
			case 0x30001: /* Number of errors and last error */
				current_msg.len = 4;
				current_msg.data[0] = (ubyte) (slave_node.num_errors>>8);
				current_msg.data[1] = (ubyte) (slave_node.num_errors);
				current_msg.data[2] = 0x0;
			 	/* LEC from CAN controller */
 				current_msg.data[3] = C1CSR >> 8;
				amb_transmit_monitor(&current_msg);
				slave_node.num_transactions++;
				return;
				break;
			case 0x30002: /* Number of transactions */
				current_msg.len = 4;
				current_msg.data[0] = (ubyte) (slave_node.num_transactions>>24);
				current_msg.data[1] = (ubyte) (slave_node.num_transactions>>16);
				current_msg.data[2] = (ubyte) (slave_node.num_transactions>>8);
				current_msg.data[3] = (ubyte) (slave_node.num_transactions);
				amb_transmit_monitor(&current_msg);
				slave_node.num_transactions++;
				return;
				break;
			case 0x30004: /* Slave software revision level */
				current_msg.len = 3;
				current_msg.data[0] = (ubyte) (slave_node.sw_revision_level[0]);
				current_msg.data[1] = (ubyte) (slave_node.sw_revision_level[1]);
				current_msg.data[2] = (ubyte) (slave_node.sw_revision_level[2]);
				amb_transmit_monitor(&current_msg);
				slave_node.num_transactions++;
				return;
				break;
			case 0x30005: /* Slave hardware revision level */
				current_msg.len = 2;
				current_msg.data[0] = (ubyte) (slave_node.hw_revision_level[0]);
				current_msg.data[1] = (ubyte) (slave_node.hw_revision_level[1]);
				amb_transmit_monitor(&current_msg);
				slave_node.num_transactions++;
				return;
				break;
		}
	}

	/* For each registered callback, see if this message was in range */
	for (i=0; i<slave_node.num_cbs; i++) {
		if ((current_msg.relative_address >= slave_node.cb_ops[i].low_address) &&
			(current_msg.relative_address <= slave_node.cb_ops[i].high_address)) {

			/* Increment the transaction counter */
			slave_node.num_transactions++;
			(slave_node.cb_ops[i].cb_func)(&current_msg);

			if (current_msg.dirn == CAN_MONITOR)
				amb_transmit_monitor(&current_msg);

			return;
		}	
	}
}

/* Routine to send monitor data back to master using CAN object 3 */
void amb_transmit_monitor(CAN_MSG_TYPE *monitor_msg){
  	ubyte i;
  	ulong TX_ID;
		ulong v;
  		CAN_OBJ[2].MCR = 0xfb7f;     /* set CPUUPD, reset MSGVAL */

	/* Recalculate CAN message from relative address */
	TX_ID = slave_node.base_address + (*monitor_msg).relative_address;

	/* Calculate the arbitration registers */

   		v = 0x00000000;
   		v += (TX_ID & 0x0000001f) << 11;  /* ID  4.. 0 */
   		v += (TX_ID & 0x00001fe0) >>  5;  /* ID 12.. 5 */
   		CAN_OBJ[2].LAR  = v;

	   	v = 0x00000000;
   		v += (TX_ID & 0x001fe000) >>  5;  /* ID 13..20 */
   		v += (TX_ID & 0x1fe00000) >> 21;  /* ID 21..28 */
   		CAN_OBJ[2].UAR  = v;


	/* set transmit direction and length */

   		CAN_OBJ[2].MCFG = 0x0c | ((*monitor_msg).len << 4);

	/* Copy data to CAN object 3 */
   	for(i = 0; i < (*monitor_msg).len; i++) {

      		CAN_OBJ[2].Data[i] = (*monitor_msg).data[i];
	}
  		CAN_OBJ[2].MCR  = 0xf6bf;  /* set NEWDAT, reset CPUUPD, set MSGVAL */
	
		/* Transmit the object */
  		CAN_OBJ[2].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */
}


