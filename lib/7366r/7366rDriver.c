/***********************************************************
* Class: CSC-615-01 Spring 2022
* Name: Robert Bierman
* Student ID: 
* GitHub ID: bierman
* Project: CSC615 Motor Encoder Board with LS7366R
*
* File: ls7336r.c
*
* Description: This file contains routine and a testbed for
*    using the LS7336 Quadrature Encode chip.
***********************************************************/
#include"7366rDriver.h"

unsigned char setMDR0[] = {WRITE_MODE0, FOURX_COUNT};
unsigned char setMDR1[] = {WRITE_MODE1, FOURBYTE_COUNTER};
unsigned char clearStatus[] = {CLEAR_STATUS};
unsigned char clearCounter[] = {CLEAR_COUNTER};
unsigned char readCounterMsg[] = { READ_COUNTER, 0, 0, 0, 0};
unsigned char BYTE_MODE[] = {ONEBYTE_COUNTER, TWOBYTE_COUNTER, 
                             THREEBYTE_COUNTER, FOURBYTE_COUNTER};
/*************************************************************************
 *   LS7336R Read Counter
 *
 *  int readLS7336RCounter (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value (4 byte) of the counter
 *
 *  Note that initLS7336RChip must be called prior to reading the counter
 *************************************************************************/
 
int readLS7336RCounter (int ChipEnable)
	{
	char dataFromChip[20];
    bbSPIXfer(ChipEnable, readCounterMsg, dataFromChip, 5);
    int result = (((int)dataFromChip[1]) << 24) + (((int)dataFromChip[2]) << 16) + 
    	                        (((int)dataFromChip[3]) << 8) + ((int)dataFromChip[4]);
    return (result);
	}


/*************************************************************************
 *   LS7336R Clear Counter
 *
 *  int clearLS7336RCounter (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value 0 if success, otherwise pigpio error number
 *
 *  Note that initLS7336RChip must be called prior to reading the counter
 *************************************************************************/
 
int clearLS7336RCounter (int ChipEnable)
    {
    int ret;
    char dataFromChip[20];
    
    // Clear the counter
    ret = bbSPIXfer(ChipEnable, clearCounter, dataFromChip, 1);
    if (ret >= 0) // xfer succeeded
        {
        ret = 0;
        }
    return (ret);
    }
 
/*************************************************************************
 *   LS7336R init
 *
 *  int initLS7336RChip (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value 0 if success, otherwise pigpio error number
 *
 *  initLS7336RChip initializes the pigpio SPI interface,
 *      it initializes the LS7336R chip by setting MDR0 to 4x Count Mode,
 *      setting MDR1 to 4 byte counter mode, clearing the status register,
 *      and clearing the counter.
 *************************************************************************/
 
int initLS7336RChip (int ChipEnable)
	{
	int ret;
	char dataFromChip[20];
	
    ret = bbSPIOpen(ChipEnable, SPI0_MISO, SPI0_MOSI, SPI0_SCLK, 100000, 0); //open SPI
    if (ret == 0)  // open succeeded
        {
    	usleep (10000);
    	//set MDR0 to 4x counter mode
        ret = bbSPIXfer(ChipEnable, setMDR0, dataFromChip, 2);  //Set MDR0 
        if (ret >= 0)  //xfer succeeded
            {
            usleep (10000);
            // set MDR1 to 4 byte counter mode
            ret = bbSPIXfer(ChipEnable, setMDR1, dataFromChip, 2);  //Set MDR1 
            if (ret >= 0)  //xfer succeeded
                {
                // Clear status
                ret = bbSPIXfer(ChipEnable, clearStatus, dataFromChip, 1);
                if (ret >= 0)  //xfer succeeded
                    {
                    // Clear the counter
                    ret = clearLS7336RCounter (ChipEnable);
                    }
                }
            }
        }
        
    return (ret);
	}	
	

