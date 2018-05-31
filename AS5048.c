/** @defgroup <Module_Name>
 *  @addtogroup <Module_Name>(If this is a sub-module, add it father module here)
 *  @design <Document link>
 *  @testspec <Document link>
 *
 *  @{
 */
//------------------------------------------------------------------------------
//  $Header$
//
//  Company    : lingdong Control Technologies, Co., Ltd.
//
//  Project    : luomandi
//
//  Filename   : x.c
//
//  Programmer : RP
//
//  Description   : permanent magnet motor control without speed sensors
//
//              ***  Confidential property of lingdong Control Technologies ***
//                             Copyright(c) lingdong Control Technologies, 2014
//------------------------------------------------------------------------------

#include "AS5048.h"
#include "hard_config.h"

volatile uint16 IC1_Value = 0;	
volatile uint16 IC2_Value = 0;
extern u8 TIM2_CH1_STA; //输入捕获的状态
extern u16 TIM2_CH1_VAL;//输入捕获值
extern u16 v;//输入捕获值
uint16 indata;
_iq ThetaEst;


/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
extern uint16_t DutyCycle_T2;

_iq  EleAng_Clc_T2(void)
{

	sint32 Q32_Motor_EleAngle;
	
//	Q32_Motor_EleAngle =(AS5048_TIME/14)-((DutyCycle_T2) % (AS5048_TIME/14));
//	Q32_Motor_EleAngle = _IQ(Q32_Motor_EleAngle*PI2/(AS5048_TIME/14));
	
	Q32_Motor_EleAngle =(AS5048_TIMEDIV)-(DutyCycle_T2 % AS5048_TIMEDIV);
	Q32_Motor_EleAngle = _IQ(Q32_Motor_EleAngle*PI2*AS5048_TIMEMUL);
	
	return Q32_Motor_EleAngle;//iq15
}
/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
extern uint16_t DutyCycle_T4;
_iq  EleAng_Clc_T4(void)
{

	sint32 Q32_Motor_EleAngle;
	
//	Q32_Motor_EleAngle =(AS5048_TIME/14)-((DutyCycle_T4) % (AS5048_TIME/14));
//	Q32_Motor_EleAngle = _IQ(Q32_Motor_EleAngle*PI2/(AS5048_TIME/14));
	
	Q32_Motor_EleAngle =(AS5048_TIMEDIV)-(DutyCycle_T4 % AS5048_TIMEDIV);
	Q32_Motor_EleAngle = _IQ(Q32_Motor_EleAngle*PI2*AS5048_TIMEMUL);
	
	return Q32_Motor_EleAngle;//iq15
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


