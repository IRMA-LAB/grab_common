#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project TestEasyCAT1.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	15
#define CUST_BYTE_NUM_IN	14
#define TOT_BYTE_NUM_ROUND_OUT	16
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		double      DoubleVarOut;
		float       FloatVarOut;
		uint8_t     LedBlue;
		uint8_t     LedOrange;
		uint8_t     LedRed;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		double      DoubleVarIn;
		float       FloatVarIn;
		uint16_t    Analog;
	}Cust;
} PROCBUFFER_IN;

#endif