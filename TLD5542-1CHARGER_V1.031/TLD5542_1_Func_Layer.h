/*
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************************************/
/**
 * \file     TLD5542_1_Func_Layer.h
 *
 * \brief    TLD5542_1 Functional Layer Header
 *
 * \version  V0.0.1
 * \date     18 Mar 2020
 *
 * \note 
 */

/*******************************************************************************
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** AT           Alberto Trentin                                               **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/*

 */

#ifndef TLD5542_1_FUNCLAYER_H
#define TLD5542_1_FUNCLAYER_H /**< @brief prevent the contents of TLD5542_1_Func_Layer.h being included more than once*/ 

#include "TLD5542_1_Reg_Layer.h"
//#include "PowerFlex_HAL_Layer.h"

#define TLD5542_1_VFHB_VFBL_REF_uV  150000UL /**< @brief TLD5542-1QV control loop reference voltage at 100% analog dimming (ADIMVAL 240) [uV]*/
#define TLD5542_1_ADIM_FS 						 240UL	 /**< @brief TLD5542-1QV analog dimming full scale value (100%)*/
#define TLD5542_1_MAX_MFSDLY 					0xFFUL  /**< @brief TLD5542-1QV MFSDLY maximum value*/
#define TLD5542_1_IEA_NEG 							56UL  /**< @brief TLD5542-1QV error amplifiere neagtive saturation current A6 sense Amp -sat current [uA] */
#define TLD5542_1_TSAMPLEFILT 					11UL  /**< @brief TLD5542-1QV Required time interval between the mfs Denominator and Numerator SPI commands */
#define TLD5542_1_TCALIBEXE 					 250UL  /**< @brief TLD5542-1QV time to internally perform the calibration routine*/
#define TLD5542_1_LEDCURRCAL_TIMEOUT 	 100UL  /**< @brief Maximum number of consecutive LEDCURRCAL SPI readout before stopping CURRMON.EOCAL polling*/
#define TLD5542_1_CURRMON_TIMEOUT 			 8UL  /**< @brief Maximum number of consecutive CURRMON SPI readout before stopping CURRMON.EOMON polling*/

typedef enum{
	OFF = 0,	
	FM12KHZ_FEDEV8 = 1,
	FM12KHZ_FEDEV16 = 2,
	FM18KHZ_FEDEV8 = 3,
	FM18KHZ_FEDEV16 = 4
}TLD5542_1_FLATSPECTRUMMODE; /**< @brief Possible Flat Spectrum feature configuration */ 


typedef struct{
	uint32 voutinitial_mV; 	/**< @brief Cout (output capacitor) voltage before starting FD [mV]*/
	uint32 vcompinitial_mV; /**< @brief Ccomp (compensation capacitor) voltage before starting Fast Discahrge procedure [mV]*/
	uint32 voutfinal_mV;	  /**< @brief Target Cout (output capacitor) voltage after the end of Fast Discharge procedure [mV]*/ 
	uint32 vcompfinal_mV; 	/**< @brief Target Ccomp (compensation capacitor) voltage after the end of Fast Discharge procedure [mV]*/ 
	uint32 ccomp_nF;				/**< @brief TLD5542-1QV Ccomp (compensation capacitor) value [nF] */
	uint32 fsw_kHz;					/**< @brief TLD5542-1QV Switching frequency [kHz]*/
}MFS_type; /**< @brief Contains information to calculate mfs core control used Fast Discharge procedure */

TLD5542_1_STATUS TLD5542_1_analogDimmingCurrentRegulator(uint32 iout_mA, uint32 rfb_mOhm);
/*TLD5542_1_STATUS TLD5542_1_analogDimmingVoltageRegulator(uint32 vout_mV, uint32 rfb1_Ohm, uint32 rfb2_Ohm);*/

TLD5542_1_STATUS TLD5542_1_flatSpectrum(TLD5542_1_FLATSPECTRUMMODE fsmode);

TLD5542_1_STATUS TLD5542_1_currentMonitorRoutine(void);

TLD5542_1_STATUS TLD5542_1_currentCalibrationRoutine(void);

uint8 TLD5542_1_mfsdlyCalc(uint32 vcompinitial_mV, uint32 vcompfinal_mV, uint32 ccomp_nF, uint32 fsw_kHz);
void TLD5542_1_mfsNumDenCalc(uint8 *num, uint8 *den, uint32 voutinital_mV, uint32 voutfinal_mV);
TLD5542_1_STATUS TLD5542_1_fastDischargeMFS(uint8 numerator, uint8 denominator, uint8 mfsdly, const bool *waitmfstrigger);
TLD5542_1_STATUS TLD5542_1_advFastDischargeMFS(const MFS_type* mfs, const bool *waitmfstrigger);

#endif /* TLD5542_1_FUNCLAYER_H */
