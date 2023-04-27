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

#ifndef SENSOR7366R__H__ 
#define SENSOR7366R__H__


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pigpio.h>
#include"../../definitions.h"

/*  Commands  */
#define	CLEAR_COUNTER	0x20		// 00 (WR) 100 (CNTR)
#define CLEAR_STATUS	0x30		// 00 (WR) 110 (STR)
#define READ_COUNTER	0x60
#define READ_STATUS		0x70
#define WRITE_MODE0		0x88
#define WRITE_MODE1		0x90
#define READ_MODE0		0x48
#define READ_MODE1		0x50

/*  Modes  */
#define FOURX_COUNT		0x03

#define FOURBYTE_COUNTER	0x00
#define THREEBYTE_COUNTER	0x01
#define TWOBYTE_COUNTER		0x02
#define ONEBYTE_COUNTER		0x03


int readLS7336RCounter (int ChipEnable);

int clearLS7336RCounter (int ChipEnable);


int initLS7336RChip (int ChipEnable);

#endif //SENSOR7366R__H__
