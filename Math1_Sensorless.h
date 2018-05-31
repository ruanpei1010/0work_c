/** @file */  /* DoxyGen file declaration */
/** @addtogroup <Module_Name> 
 *  
 *  @{
 */
//------------------------------------------------------------------------------
//  $Header$
//
//  Company    : 
//
//  Project    : 
//
//  Filename   : 
//
//  Programmer : 
//                             
//  Description   : MRAS Based Estimation of Speed in Sensorless PMSM Drive
//
//              ***  Confidential property of lingdong Control Technologies ***
//                             Copyright(c) lingdong Control Technologies, 2014
//------------------------------------------------------------------------------

#ifndef MATH1_SENSORLESS_H
#define MATH1_SENSORLESS_H
//-------------------- include files ----------------------------------------
#include "typedef.h"

extern const sint16 SINTable1024Q15[];

typedef struct {
	sint32 iu;
	sint32 iv;
	sint32 iw;
} TPhCurt32;

//--------------ab结构体--------------
typedef struct {
	sint16 Axis_D;
	sint16 Axis_Q;
} TCplx16;
typedef struct {
	sint32 Axis_D;
	sint32 Axis_Q;
} TCplx32;

//--------------DQ结构体--------------
typedef struct {
	sint16 d;
	sint16 q;
} TDQ16;
typedef struct {
	sint32 d;
	sint32 q;
} TDQ32;

// 三角函数结构体
typedef struct {
	sint16 cos;
	sint16 sin;
} TTrigQ15;

											
void ClarkTrans32(TPhCurt32 *In, TCplx32 *Result);
void ParkTrans32T16(TCplx32 *In, TTrigQ15 *theta, TDQ32 *Result);
void IParkTrans32T16(TDQ32 *In, TTrigQ15 *theta, TCplx32 *Result);


//part0330---------------------------------------------------------------
typedef struct {  
					_iq  As;  		// Input: phase-a stator variable
				  _iq  Bs;			// Input: phase-b stator variable 
				  _iq  Alpha;		// Output: stationary d-axis stator variable 
				  _iq  Beta;		// Output: stationary q-axis stator variable
		 	 	  void  (*calc)();	// Pointer to calculation function
				 } CLARKE;	            

typedef CLARKE *CLARKE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
              			  (void (*)(uint32))clarke_calc }


typedef struct {  
					_iq  Alpha;  		// Input: stationary d-axis stator variable 
				  _iq  Beta;	 	// Input: stationary q-axis stator variable 
				  _iq  Angle;		// Input: rotating angle (pu) 
				  _iq  Ds;			// Output: rotating d-axis stator variable 
				  _iq  Qs;			// Output: rotating q-axis stator variable 
		 	 	  void  (*calc)();	// Pointer to calculation function
				 } PARK;	            

typedef PARK *PARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			  (void (*)(uint32))park_calc }

											
typedef struct {  
					_iq  Alpha;  		// Output: stationary d-axis stator variable
				  _iq  Beta;		// Output: stationary q-axis stator variable
				  _iq  Angle;		// Input: rotating angle (pu)
				  _iq  Ds;			// Input: rotating d-axis stator variable
				  _iq  Qs;			// Input: rotating q-axis stator variable
		 	 	  void  (*calc)();	// Pointer to calculation function 
				 } IPARK;	            

typedef IPARK *IPARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/                     
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			  (void (*)(uint32))ipark_calc }											
											
void clarke_calc(CLARKE_handle);
void park_calc(PARK_handle);
void ipark_calc(IPARK_handle);
											
											
typedef struct {  
					_iq  Ref;   			// Input: Reference input 
				  _iq  Fdb;   			// Input: Feedback input 
				  _iq  Err;				// Variable: Error
				  _iq  Kp;				// Parameter: Proportional gain
				  _iq  Up;				// Variable: Proportional output 
				  _iq  Ui;				// Variable: Integral output 
				  _iq  Ud;				// Variable: Derivative output 	
				  _iq  OutPreSat; 		// Variable: Pre-saturated output
				  _iq  OutMax;		    // Parameter: Maximum output 
				  _iq  OutMin;	    	// Parameter: Minimum output
				  _iq  Out;   			// Output: PID output 
				  _iq  SatErr;			// Variable: Saturated difference
				  _iq  Ki;			    // Parameter: Integral gain
				  _iq  Kc;		     	// Parameter: Integral correction gain
				  _iq  Kd; 		        // Parameter: Derivative gain
				  _iq  Up1;		   	    // History: Previous proportional output
		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } PIDREG3;	            

typedef PIDREG3 *PIDREG3_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG3_DEFAULTS { 0, \
                           0, \
                           0, \
                           _IQ(1.3), \
                           0, \
                           0, \
                           0, \
                           0, \
                           _IQ(1), \
                           _IQ(-1), \
                           0, \
                           0, \
                           _IQ(0.02), \
                           _IQ(0.5), \
                           _IQ(1.05), \
                           0, \
              			  (void (*)(uint32))pid_reg3_calc }											
											
void pid_reg3_calc(PIDREG3_handle);	
											
//-------------------------------------------------------------------------------------
typedef struct{
	_iq    In;
	_iq    Out;
	_iq    Kp_Gain;
	LPVOID Fliter_Update;
	}FLITER;

void LPF_calc(FLITER *);

#define  FLITER_DEFAULT { 0,0,_IQ(0.0), \
								(void (*)(uint32))LPF_calc}
	
/*=========================================================================================
// Formula: y+=tc*(x-wc*y)
// x---input,y---output
// tc--calculation period
// wc--cut down frequency
// Usage: yn=LPFI(yn,xn,Tc,Wc);
//---------------------------------------------------------------------------------------- */
#define LPFI(yn,xn,Tc,Wc) (yn+Tc*(xn-yn*Wc))



//#define FilterN(x,y) (y + (x-y)/N)   一阶滤波函数， 有静差,X和Y为同一Q格式
#define	Filter1(x,total)   (x)
#define	Filter2(x,total)   (( (((uint32)total)<<16) + (((uint32)x)<<15) - (((uint32)total)<<15) )>>16)
#define	Filter4(x,total)   (( (((uint32)total)<<16) + (((uint32)x)<<14) - (((uint32)total)<<14) )>>16)
#define	Filter8(x,total)   (( (((uint32)total)<<16) + (((uint32)x)<<13) - (((uint32)total)<<13) )>>16)
#define	Filter16(x,total)  (( (((uint32)total)<<16) + (((uint32)x)<<12) - (((uint32)total)<<12) )>>16)
#define	Filter32(x,total)  (( (((uint32)total)<<16) + (((uint32)x)<<11) - (((uint32)total)<<11) )>>16)
#define	Filter64(x,total)  (( (((uint32)total)<<16) + (((uint32)x)<<10) - (((uint32)total)<<10) )>>16)
#define Filter128(x,total) (( (((uint32)total)<<16) + (((uint32)x)<<9 ) - (((uint32)total)<<9 ) )>>16)
#define Filter256(x,total) (( (((uint32)total)<<16) + (((uint32)x)<<8 ) - (((uint32)total)<<8 ) )>>16)


//例如：	gIMTQ12.M = Filter2((gIMTQ24.M>>12), gIMTQ12.M); 滤波虑的很就选Filter256等较大的数
								
//part0330--------------------------------------------------------------------------------
																						
//-------------------- inline functions -------------------------------------

#endif /* MATH1_SENSORLESS_H */
//-----------------------End of file------------------------------------------
/** @}*/

