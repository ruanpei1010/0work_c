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
//  Project    : 
//
//  Filename   : 
//
//  Programmer : RP
//
//  Description   : permanent magnet motor control without speed sensors
//
//              ***  Confidential property of lingdong Control Technologies ***
//                             Copyright(c) lingdong Control Technologies, 2014
//------------------------------------------------------------------------------

#include "Math1_Sensorless.h"
#include "hard_config.h"
//常数定义
#define SQRT3INVQ31 0x49E69D16	//IQ31(0.57735) = IQ31(1/SQRT(3))

/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
 
/*********************************************************************************************************
** 函数名称: void ClarkTrans32(void)
** 功能描述: Clark变换，输入和输出均为32位
**
** 作　  者:
** 日　  期:
**********************************************************************************************************/
void ClarkTrans32(TPhCurt32 *In, TCplx32 *Result)
{
	Result->Axis_D = In->iu;

	Result->Axis_Q = -(sint32)(((llong)((In->iu>>1) + In->iw)*(llong)SQRT3INVQ31) >> 30);

}

/*********************************************************************************************************
** 函数名称: void ParkTrans3216(void)
** 功能描述: Park变换，输入和输出均为32位，正余弦值16位
**
** 作　  者:
** 日　  期:
**********************************************************************************************************/
void ParkTrans32T16(TCplx32 *In, TTrigQ15 *theta, TDQ32 *Result)
{                                                                                                     // sinx\cosx最大值班为1，最小值为 -1 （正余弦原理）
	Result->d = (sint32)(((llong)In->Axis_D*(llong)theta->cos + (llong)In->Axis_Q*(llong)theta->sin)>>15);  // >>15位，因为sinx\cosx=1时的值为32767
	Result->q = (sint32)(((llong)In->Axis_Q*(llong)theta->cos - (llong)In->Axis_D*(llong)theta->sin)>>15);  // sinx\cosx=-1时的值为-32767
}

/*********************************************************************************************************
** 函数名称: void IParkTrans3216(void)
** 功能描述: IPark变换，输入和输出均为32位，正余弦值16位
**
** 作　  者:
** 日　  期:
**********************************************************************************************************/
void IParkTrans32T16(TDQ32 *In, TTrigQ15 *theta, TCplx32 *Result)
{
	Result->Axis_D = (sint32)(((llong)In->d*(llong)theta->cos - (llong)In->q*(llong)theta->sin)>>15);
	Result->Axis_Q = (sint32)(((llong)In->d*(llong)theta->sin + (llong)In->q*(llong)theta->cos)>>15);
}


//---------------------------------------------------------------------------
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
//
//
//
/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
 
void clarke_calc(CLARKE *v)
{	
   v->Alpha = v->As;
   v->Beta = _IQmpy((v->As + _IQmpy(_IQ(2),v->Bs)),_IQ(0.57735026918963)); // 1/sqrt(3) = 0.57735026918963
}
/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
 
void park_calc(PARK *v)
{	
	_iq Cosine,Sine;

	// Using look-up IQ sine table
	Sine = _IQsinPU(v->Angle);
	Cosine = _IQcosPU(v->Angle);

	v->Ds = _IQmpy(v->Alpha,Cosine) + _IQmpy(v->Beta,Sine);
	v->Qs = _IQmpy(v->Beta,Cosine) - _IQmpy(v->Alpha,Sine);
}
/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
 
void ipark_calc(IPARK *v)
{	
	_iq Cosine,Sine;

	// Using look-up IQ sine table
	Sine = _IQsinPU(v->Angle);
	Cosine = _IQcosPU(v->Angle);

	v->Alpha = _IQmpy(v->Ds,Cosine) - _IQmpy(v->Qs,Sine);
	v->Beta = _IQmpy(v->Qs,Cosine) + _IQmpy(v->Ds,Sine);  
}

/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
void pid_reg3_calc(PIDREG3 *v)
{	
    // Compute the error
    v->Err = v->Ref - v->Fdb;
	
		if(v->Err >= (AS5048_TIME>>1))
			v->Err -=(AS5048_TIME);
		if(v->Err <= -(AS5048_TIME>>1))
			v->Err +=(AS5048_TIME);
					
    // Compute the proportional output
    v->Up = _IQ10mpy(v->Kp,v->Err);

    // Compute the integral output
    v->Ui = v->Ui + _IQmpy(v->Ki,v->Up) + _IQmpy(v->Kc,v->SatErr);

    // Compute the derivative output
//    v->Ud = _IQmpy(v->Kd,(v->Err - v->Up1));//kd = 0;
		v->Ud = _IQ10mpy(v->Kd,(v->Err - v->Up1));//kd = 0;
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Err; 

}

/*! \fn					void function(UNSIGNED32 u32Param1)
 *  \brief 			Description of this function
 *  \param 			param1: Description of parameter
 *  \param 			param2: Description of parameter
 *  \exception  (None non-reentrant code)
 *  \return 		TRUE: success FALSE: unsuccess
 */
void LPF_calc(FLITER *v)
{
	v->Out = _IQmpy(v->In, v->Kp_Gain) + _IQmpy(v->Out,(_IQ(1.0)- v->Kp_Gain));//_IQmpy乘法
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */

