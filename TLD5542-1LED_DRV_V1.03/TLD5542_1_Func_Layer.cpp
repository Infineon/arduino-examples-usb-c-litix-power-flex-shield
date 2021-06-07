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
 * \file     TLD5542_1_Func_Layer.c
 *
 * \brief    TLD5542_1 Functional Layer Source Code
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
  
#include "TLD5542_1_Func_Layer.h"
#include <Arduino.h>



/** @brief Controls the regulated output current when TLD5542-1QV is used as current regulator
 * \param[in] iout_mA target output current
 * \param[in] rfb_mOhm output current shunt sense resistor (see Figure 44 TLD5542-1QV datasheet)
 * \return TLD5542-1QV status
 * \note  TLD5542-1QV Chapter 8, Analog Dimming and Limp Home, Chapter 13 Application information
*/
TLD5542_1_STATUS TLD5542_1_analogDimmingCurrentRegulator(uint32 iout_mA, uint32 rfb_mOhm){
	
	/*Using Register Layer APIs and Defines*/
	uint32 adim_reg = (uint32)((iout_mA * rfb_mOhm * TLD5542_1_ADIM_FS)/TLD5542_1_VFHB_VFBL_REF_uV)+1;

               
	if(adim_reg > TLD5542_1_ADIM_FS){
		adim_reg = TLD5542_1_ADIM_FS;
	}
	
	/*Register APIs*/
	tld5542_1.LEDCURRADIM.ADIMVAL = (uint8) adim_reg;
	return (TLD5542_1_Sync_Write_Register(TLD5542_1_LEDCURRADIM_ADDR));
	
}

/** @brief Controls the regulated output voltage when TLD5542-1QV is used as voltage regulator
 * \param[in] vout_mV target output voltage
 * \param[in] rfb1_Ohm output voltage sense resistor 1 connected to FBH (see Figure 45 TLD5542-1QV datasheet)
 * \param[in] rfb2_Ohm output voltage sense resistor 2 Connected to FBL (see Figure 45 TLD5542-1QV datasheet)
 * \return TLD5542-1QV status
 * \note  TLD5542-1QV Chapter 8, Analog Dimming and Limp Home, Chapter 13 Application information
*/
TLD5542_1_STATUS TLD5542_1_analogDimmingVoltageRegulator(uint32 vout_mV, uint32 rfb2_Ohm, uint32 rfb1_Ohm){	

  //uint32 vfbhvfbl_uV = (((vout_mV * 1000) +  (TLD5542_1_IFBL * rfb1_Ohm)) / (rfb1_Ohm + rfb2_Ohm)) * rfb1_Ohm;
  uint32 vfbhvfbl_uV = ((vout_mV * 1000*rfb1_Ohm)/(rfb1_Ohm + rfb2_Ohm)) + rfb1_Ohm*TLD5542_1_IFBL;
  uint32 adim_reg = ( (vfbhvfbl_uV * TLD5542_1_ADIM_FS)/TLD5542_1_VFHB_VFBL_REF_uV) +1;

  
	if(adim_reg > TLD5542_1_ADIM_FS){
		adim_reg = TLD5542_1_ADIM_FS;
	}
	
	tld5542_1.LEDCURRADIM.ADIMVAL = (uint8) adim_reg;
 
  Serial.print(adim_reg, HEX);// DEBUG
   Serial.println(" ADIMREG");// DEBUG
 

	return (TLD5542_1_Sync_Write_Register(TLD5542_1_LEDCURRADIM_ADDR));
}


/** @brief Configure Flat spectrum according with options proposed by TLD5542_1_FLATSPECTRUMMODE enum
 * \param[in] fsmode flat spectrum configuration
 * \return TLD5542-1QV status
 * \note  TLD5542-1QV Chapter 11, Infineon FLAT SPECTRUM Feature set
*/

TLD5542_1_STATUS TLD5542_1_flatSpectrum(TLD5542_1_FLATSPECTRUMMODE fsmode){
	
	switch(fsmode){
		case OFF:
			tld5542_1.SWTMOD.ENSPREAD = TLD5542_1_SWTMOD_ENSPREAD_DISABLE_SSM;
		break;
		case FM12KHZ_FEDEV8:
			tld5542_1.SWTMOD.ENSPREAD   = TLD5542_1_SWTMOD_ENSPREAD_ENABLE_SSM;
			tld5542_1.SWTMOD.FMSPREAD   = TLD5542_1_SWTMOD_FMSPREAD_12kHz;
			tld5542_1.SWTMOD.FDEVSPREAD = TLD5542_1_SWTMOD_FDEVSPREAD_8_PERC;
		break;
		case FM12KHZ_FEDEV16:
			tld5542_1.SWTMOD.ENSPREAD   = TLD5542_1_SWTMOD_ENSPREAD_ENABLE_SSM;
			tld5542_1.SWTMOD.FMSPREAD   = TLD5542_1_SWTMOD_FMSPREAD_12kHz;
			tld5542_1.SWTMOD.FDEVSPREAD = TLD5542_1_SWTMOD_FDEVSPREAD_16_PERC;
		break;
		case FM18KHZ_FEDEV8:
			tld5542_1.SWTMOD.ENSPREAD   = TLD5542_1_SWTMOD_ENSPREAD_ENABLE_SSM;
			tld5542_1.SWTMOD.FMSPREAD   = TLD5542_1_SWTMOD_FMSPREAD_18kHz;
			tld5542_1.SWTMOD.FDEVSPREAD = TLD5542_1_SWTMOD_FDEVSPREAD_8_PERC;
		break;
		case FM18KHZ_FEDEV16:
			tld5542_1.SWTMOD.ENSPREAD   = TLD5542_1_SWTMOD_ENSPREAD_ENABLE_SSM;
			tld5542_1.SWTMOD.FMSPREAD   = TLD5542_1_SWTMOD_FMSPREAD_18kHz;
			tld5542_1.SWTMOD.FDEVSPREAD = TLD5542_1_SWTMOD_FDEVSPREAD_16_PERC;
		break;
		default:
			/*no action*/
		break;
	}
	
	return (TLD5542_1_Sync_Write_Register(TLD5542_1_SWTMOD_ADDR));
}


/** @brief Executes current monitor routine
 *
 * Starts the current monitor routine with appropriate SPI commands. Polling EOMON flag until the rotuine is finished
 * \return TLD5542-1QV status
 * \note  TLD5542-1QV Chapter 10.4 and 10.5, Input current Monitoring , Output current Monitoring 
*/

TLD5542_1_STATUS TLD5542_1_currentMonitorRoutine(void){
	TLD5542_1_STATUS status;
	uint8 count = 0;
	
	/*To execute the current monitor routine the CURRMON.SOMON bit has to be set HIGH and the result is ready when CURRMON.EOMON is read HIGH*/
	tld5542_1.CURRMON.EOMON = TLD5542_1_CURRMON_SOMON_START;
	status = TLD5542_1_Sync_Write_Register(TLD5542_1_CURRMON_ADDR);
	
	if(status == TLD5542_1_ACTIVE){
		/*When CURRMON.SOMON bit is set to HIGH both input and output current monitor routines are executed in parallel.*/
		do{
			count ++;
			status = TLD5542_1_Sync_Write_Register(TLD5542_1_CURRMON_ADDR);
		}while((count < TLD5542_1_CURRMON_TIMEOUT) && (status == TLD5542_1_ACTIVE) && (tld5542_1.CURRMON.EOMON == TLD5542_1_CURRMON_EOMON_NOT_PERFORMED)); 
	}
	
	return status;
}


/** @brief Executes current calibration routine
 * Starts the calibration routine with appropriate SPI commands. Waits the time needed to internally execute the rotuine (250u).
 * Polling EOCAL flag until the rotuine is finished
 * Before calling this service the user shall prepare TLD5542-1QV as described by the datasheet:
 * power the Load with a low analog dimming value (for example 10%)
 * Set PWMI = LOW and disconnect the Load at the same time (to avoid Vout drifts from operating conditions and bring the output current to 0)d
 * \return TLD5542-1QV status
 * \note  TLD5542-1QV Chapter 8.2, LED current calibration procedure 
*/

TLD5542_1_STATUS TLD5542_1_currentCalibrationRoutine(void){
	TLD5542_1_STATUS status;
	uint8 count = 0;
	
	/*uC enables the calibration routine: DVCCTRL.ENCAL = HIGH. When ENCAL = HIGH: the calibration results coming from the routine is used by internal
		circuitery and can be read back from LEDCURRCAL.CALIBVAL
	*/
	tld5542_1.DVCCTRL.ENCAL = TLD5542_1_DVCCTRL_ENCAL_CALIB_AUTO_PROC;
	status = TLD5542_1_Sync_Write_Register(TLD5542_1_DVCCTRL_ADDR);
		
	/*uC starts the calibration routine: LEDCURRCAL.SOCAL = HIGH */
	if(status == TLD5542_1_ACTIVE){
		tld5542_1.LEDCURRCAL.SOCAL = TLD5542_1_LEDCURRCAL_SOCAL_START_CALIB;
		status = TLD5542_1_Sync_Write_Register(TLD5542_1_LEDCURRCAL_ADDR);
	}
	
	//waiting time (needed to internally perform the calibration rotuine) --> approx 200us
	delay(TLD5542_1_TCALIBEXE);
	
	if(status == TLD5542_1_ACTIVE){
		/*polling LEDCURRCAL.EOCAL. TLD5542-1QV will set the flag: LEDUCRRCAL.EOCAL = HIGH, when calibration routine has finished*/
		do{
			count ++;
			status = TLD5542_1_Sync_Write_Register(TLD5542_1_LEDCURRCAL_ADDR );
		}while((count < TLD5542_1_LEDCURRCAL_TIMEOUT) && (status == TLD5542_1_ACTIVE) && (tld5542_1.LEDCURRCAL.EOCAL == TLD5542_1_LEDCURRCAL_EOCAL_CALIB_NOT_PERFORMED)); 
	}
	
	return status;
}

/** @brief Calculates the best MFS num,den to fit the desired voutF/VoutI ratio.
 *
 * All the possibile num/den combiantions and the correspondent ratio are stored in a static array (i.e num=9, den=14 --> ratio=643), sorted by ratio 
 * descending order. The function implements a formula to approximate the position of the ratio (voutF/voutI) in the array. After that an iterative loop 
 * is used to find out the ratio which most fits the requested one (voutF/voutI).
 * \param[in,out] num MFS numerator
 * \param[in,out] den MFS denominator
 * \param[in] voutinital_mV, \[mV\] measured output voltage before jump
 * \param[in] voutfinal_mV   \[mV\] target output voltage after jump 
* \note TLD5542-1QV Chapter 6.5, Fast Output Discharge Operation Mode - Multi Floating Switches Topology: Set the target Cout Discharge voltage
*/
void TLD5542_1_mfsNumDenCalc(uint8 *num, uint8 *den, uint32 voutinital_mV, uint32 voutfinal_mV){
	static uint16 ratioLUT [64][3] =
	{{12,13,923}  ,{10,11,909}  ,{9,10,900}  ,{8,9,889}  ,{7,8,875}   ,{6,7,857}  ,{11,13,846}  ,{5,6,833}  ,{9,11,818}  ,{4,5,800},
	 {11,14,786}  ,{7,9,778}    ,{10,13,769} ,{3,4,750}  ,{11,15,733} ,{8,11,727} ,{5,7,714}    ,{7,10,700} ,{9,13,692}  ,{2,3,667},
	 {9,14,643}   ,{7,11,636}   ,{5,8,625}   ,{8,13,615} ,{3,5,600}   ,{4,7,571}  ,{5,9,556}    ,{6,11,545} ,{7,13,538}  ,{8,15,533},
	 {1,2,500}    ,{7,15,467}   ,{6,13,462}  ,{5,11,455} ,{4,9,444}   ,{3,7,429}  ,{2,5,400}    ,{5,13,385} ,{3,8,375}   ,{4,11,364}, 
	 {5,14,357}   ,{1,3,333}    ,{4,13,308}  ,{3,10,300} ,{2,7,286}   ,{3,11,273} ,{4,15,267}   ,{1,4,250}  ,{3,13,231}  ,{2,9,222} ,
	 {3,14,214}   ,{1,5,200}    ,{2,11,182}  ,{1,6,167}  ,{2,13,154}  ,{1,7,143}  ,{2,15,133}  	,{1,8,125} 	,{1,9,111}   ,{1,10,100},
	 {1,11,91}   	,{1,13,77}    ,{1,14,71}   ,{1,15,67}     
	};
	uint32 pos = 0;
	bool eos = false;
	uint32 ratioTarget = (voutfinal_mV * 1000) / voutinital_mV;
	
	if (ratioTarget >= 1000){
		*num = 15u; 
		*den = 1u;
	}else if(ratioTarget >= 923){
		*num = 12u; 
		*den = 13u;
	}else if (ratioTarget <= 67){
		*num = 1u; 
		*den = 15u;
	}else{
		pos = ((930 - ratioTarget) << 4) / 229; //formula to calculate the approximated position of the requested ratio Target in the LUT
		do{
			if(ratioLUT[pos][2] > ratioTarget){
				pos++;
			}else{
				if((ratioLUT[pos-1][2] > ratioTarget)){
					eos = true;
				}else{
					pos--;
				}
			}		
		}while(eos == false);
		*num = (uint8) ratioLUT[pos][0]; 
		*den = (uint8) ratioLUT[pos][1]; 
	}
}



/** @brief Calculates the mfsdly value needed to discharghe Cout to a desired final vcomp.
 *
 *  dVcomp = (vcompinitial_mV - vcompfinal_mV ) is translated into TLD5542.MFSSETUP2.MSFDLY value according with system param (CCOMP, FSW, IEA_NEG)
 * \param[in] vcompinitial_mV [mV] measured vcomp before initia MFS
 * \param[in] vcompfinal_mV [mV] desired vcomp after the end of MFS
 * \param[in] ccomp_nF [nF] TLD5542 compensation capacitor value
 * \param[in] fsw_kHz [kHz] TLD5542 switching frequency
 * \return mfsdly TLD5542 register value to set Ccomp discharge time
 * \note TLD5542-1QV Chapter 6.5, Fast Output Discharge Operation Mode - Multi Floating Switches Topology: Preparation Time tprep
 * \note mfsdly calculation involves TLD5542-1QV device parameters: compensation capacitor CCOMP[nF], switching frequency fsw[kHz] and  
 *	internal error amplifier current IEA_neg [uA] which may depend on temperature and nominal value accuracy. 
 *	Underestimating mfsdly is source of overshoot while overestimating mfsdly may produces undershoot.
 *	These variations shall be compensated, for example by reading (periodically) vcomp at the end of mfs and compare with the 
 *	target mfs->vcompfinal_mV. The differnce beetween obtaind values and target can be used to adjust mfsdly calcualtion.
*/
uint8 TLD5542_1_mfsdlyCalc(uint32 vcompinitial_mV, uint32 vcompfinal_mV, uint32 ccomp_nF, uint32 fsw_kHz){
	uint32 mfsdly = 0;
	uint32 tprep_us = 0;

	
	if(vcompfinal_mV < vcompinitial_mV){ /*discharge ccomp only when dvcomp > 0 --> vcompF < vcompI*/
		
		tprep_us = ((vcompinitial_mV - vcompfinal_mV) * ccomp_nF) / TLD5542_1_IEA_NEG; /*(dvcomp[mV] * ccomp[nF]) / iea_neg[uA] --> u[s]*/
		
		mfsdly = ((tprep_us * fsw_kHz) / 1000); /* Euqation 6.7 --> 2+mfsdly = tprep*fsw [us] * [kHz*/
		
		if(mfsdly > TLD5542_1_MAX_MFSDLY){ /*be sure mfsdly is not exceeding the maximum allowed value*/
			mfsdly = TLD5542_1_MAX_MFSDLY;
		}else if(mfsdly >= 2){
			mfsdly = mfsdly - 2;
		}else{}
	}
	return (uint8) mfsdly;
}


/** @brief Sends SPI commands to execute Fast Discharge (denominator, mfsdly, numerator + SOMFS).
 *
 * The first SPI command to set MFS denominator (LEDCHAIN) is sent out.
 * The second SPI command to set MFS mfsdly is sent out
 * The third SPI command to set MFS numerator (LEDCHAIN) and start MFS routine (SOMFS) is sent out only when mfsstarttrigger flag becomes false.
 * Delay to ensure tsample of FILT capacitor is applied beetween the second and the third SPI command 
 * \param[in] numerator
 * \param[in] denumerator
 * \param[in] mfsdly
 * \param[in] mfsstarttrigger flag to synchronize SOMFS SPI command 
 * \return TLD5542-1QV status
 * \note TLD5542-1QV Chapter 6.5, Fast Output Discharge Operation Mode - Multi Floating Switches Topology 
*/
TLD5542_1_STATUS TLD5542_1_fastDischargeMFS(uint8 numerator, uint8 denominator, uint8 mfsdly, const bool *waitmfstrigger){
	TLD5542_1_STATUS status;
	
	tld5542_1.MFSSETUP1.SOMFS = TLD5542_1_MFSSETUP1_SOMFS_NOT_ACTIVE;
	
	if(denominator > 15){
		tld5542_1.MFSSETUP1.LEDCHAIN = 0x0u; 
	}else{
		tld5542_1.MFSSETUP1.LEDCHAIN = denominator; 
	}
	
	status = TLD5542_1_Sync_Write_Register(TLD5542_1_MFSSETUP1_ADDR);

	if(status == TLD5542_1_ACTIVE){
		tld5542_1.MFSSETUP2.MFSDLY = mfsdly;
		status = TLD5542_1_Sync_Write_Register(TLD5542_1_MFSSETUP2_ADDR);
	}

	/*The time interval between the Ratio Denominator and Numerator SPI commands (tsample), needs to be long enough to sample
	the Initial Voltage of the F.D. in the FILT capacitor. (i.e. with CFILT 220pF, tsmaple > 11 us)*/
	delay(TLD5542_1_TSAMPLEFILT);
	
	if(status == TLD5542_1_ACTIVE){
		if(numerator > 15){
			tld5542_1.MFSSETUP1.LEDCHAIN = 0x0u; 
		}else{
			tld5542_1.MFSSETUP1.LEDCHAIN = numerator; 
		}
		tld5542_1.MFSSETUP1.SOMFS = TLD5542_1_MFSSETUP1_SOMFS_ACTIVE;

		while(*waitmfstrigger == true){}; /*wait until boolean trigger to synchronize mfs becomes true*/

		status = TLD5542_1_Sync_Write_Register(TLD5542_1_MFSSETUP1_ADDR);
	}
	
	return status;
}


/** @brief Calculates mfs core controles (mfsdly, num, den) and sends SPI command to execute Fast Discharge
 *
 * Calculates numerator denominator and mfsdly using data stored into mfs struct. The 
 * \param[in] mfs collect necessary information to execute mfs
 * \param[in] mfsstarttrigger flag to syncronize SOMFS SPI command 
 * \return TLD5542-1QV status
 * \note TLD5542-1QV Chapter 6.5, Fast Output Discharge Operation Mode - Multi Floating Switches Topology 	
*/
TLD5542_1_STATUS TLD5542_1_advFastDischargeMFS(const MFS_type* mfs, const bool *waitmfstrigger){
	uint8 numerator, denominator, mfsdly;
	
	TLD5542_1_mfsNumDenCalc(&numerator, &denominator, mfs->voutinitial_mV, mfs->voutfinal_mV);
	mfsdly =  TLD5542_1_mfsdlyCalc(mfs->vcompinitial_mV, mfs->vcompfinal_mV, mfs->ccomp_nF, mfs->fsw_kHz);
	
	return (TLD5542_1_fastDischargeMFS(numerator, denominator, mfsdly, waitmfstrigger));
}
