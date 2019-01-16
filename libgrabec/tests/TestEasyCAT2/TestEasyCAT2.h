#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project TestEasyCAT2.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	2
#define CUST_BYTE_NUM_IN	3
#define TOT_BYTE_NUM_ROUND_OUT	4
#define TOT_BYTE_NUM_ROUND_IN	4


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint16_t    Analog;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint8_t     LedRed;
		uint8_t     LedOrange;
		uint8_t     LedYellow;
	}Cust;
} PROCBUFFER_IN;

#endif