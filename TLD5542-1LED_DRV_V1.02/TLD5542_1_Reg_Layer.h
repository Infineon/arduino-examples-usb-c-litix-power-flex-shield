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
 * \file     TLD5542_1_Reg_Layer.h
 *
 * \brief    TLD5542_1 Register Layer Functions
 *
 * \version  V0.0.1
 * \date     18. Mar 2020
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

#ifndef TLD5542_1_REGLAYER_H
#define TLD5542_1_REGLAYER_H /**< @brief prevent the contents of TLD5542_1_Reg_Layer.h being included more than once*/ 

#include <stdbool.h>

typedef unsigned char uint8; /**< \brief 8 bit unsigned value */
typedef unsigned int uint16; /**< \brief 16 bit unsigned value */
typedef unsigned long uint32; /**< \brief 32 bit unsigned value */

typedef signed char sint8; /**< \brief 8 bit signed value */
typedef signed int sint16; /**< \brief 16 bit signed value */
typedef signed long sint32; /**< \brief 32 bit signed value */

typedef float float32; /**< \brief 32 bit float value */
typedef double float64; /**< \brief 64 bit float value */

typedef short FixPoint16; /**< \brief 16 bit fix point value */
typedef long FixPoint32; /**< \brief 32 bit fix point value */


/*****************************************************************************
										Global Typedef declaration   			                  
*****************************************************************************/
typedef enum{
	TLD5542_1_STANDARD_DIAGNOSIS_ADDR = 1,	/**< @brief Standard Diagnosis Register*/ 
	TLD5542_1_LEDCURRADIM_ADDR 				= 0,	/**< @brief LED Current Configuration Register*/ 
	TLD5542_1_LEDCURRCAL_ADDR 				= 3,	/**< @brief LED Current Accuracy Trimming Configuration Register*/
	TLD5542_1_SWTMOD_ADDR 						= 5,	/**< @brief Switching Mode Configuration Register*/
	TLD5542_1_DVCCTRL_ADDR 						= 6,	/**< @brief Device Control Register */
	TLD5542_1_MFSSETUP1_ADDR 					= 9,	/**< @brief Multifloat Switch configuration Register */
	TLD5542_1_MFSSETUP2_ADDR 					= 10,	/**< @brief Multifloatswitch configuration register 2 (delay time programming)*/
	TLD5542_1_CURRMON_ADDR 						= 12,	/**< @brief Current Monitor Register */
	TLD5542_1_REGUSETMON_ADDR 				= 15,	/**< @brief Regulation Setup And Monitor Register*/
	TLD5542_1_MUXCTRL_ADDR 						= 16,	/**< @brief Internal Muxes Configuration Register*/
}TLD5542_1_REG_ADDRESS; /**< @brief Univocally identify each TLD5542-1QV register*/	

typedef enum{
	TLD5542_1_READ_CMD = 0, 	/**< @brief read SPI frame. Both MSB [b15] and LSB [b0] are set low*/ 
	TLD5542_1_WRITE_CMD = 1,	/**< @brief write SPI frame. MSB [b15] is set high*/ 
}TLD5542_1_CMD_TYPE;	/**< @brief  Univocally identify the type of SPI command*/ 

typedef enum{
	TLD5542_1_RESERVED = 0, 	/**< @brief TLD5542-1QV STD.STATE (operation status) is reserved and no fault (except for OUTOV abnd SHRTELD) has been reported*/
	TLD5542_1_LIMPHOME = 1, 	/**< @brief TLD5542-1QV STD.STATE (operation status) is limphome and no fault (except for OUTOV abnd SHRTELD) has been reported*/
	TLD5542_1_ACTIVE = 2,			/**< @brief TLD5542-1QV STD.STATE (operation status) is active and no fault (except for OUTOV abnd SHRTELD) has been reported*/
	TLD5542_1_IDLE = 3,				/**< @brief TLD5542-1QV STD.STATE (operation status) is idle and no fault (except for OUTOV abnd SHRTELD) has been reported*/
	TLD5542_1_ERROR = 4,			/**< @brief At least one TLD5542-1QV fault beetween(SWRST_BSTUV, UVLORST, TER, IVCCUVLO, TSD, TW has been reported). OUTOV and SHRTLED are ignored*/
	TLD5542_1_RESET = 5,			/**< @brief TLD5542-1QV has been resetted since the last SPI readout (SWRST_BSTUV and TER flags are high)*/
	TLD5542_1_SPI_NOT_OK = 6,	  /**< @brief requested SPI activity has not been accepted*/
	TLD5542_1_UNKNOWN = 7,		/**< @brief Received command is not a standard diagnosis*/
}TLD5542_1_STATUS; 					/**< @brief Briefly sumarize the status of TLD5542-1QV according with the recived SPI standard diagnosis*/	


/* ============================================================================================================= */
/* ================                     TLD5542-1QV Specific Peripheral Section                     ================ */
/* ============================================================================================================= */

typedef struct {
	uint8	ADIMVAL ;  /**<@brief [7..0] Analog dimming value*/
}		TLD5542_1_LEDCURRADIM_REG_T  ;/**< @brief LED Current Configuration Register*/ 


typedef struct {
	uint8 CALIBVAL		;  /**< @brief [3..0] LED current calibration value definition*/
  uint8 EOCAL     	;  /**< @brief [4..4] End of calibration routine signalling bit*/
  uint8 SOCAL   		;  /**< @brief [5..5] Start of calibration routine signalling bit*/
  uint8	DAC_OFF   	;  /**< @brief [6..6] Switch OFF internal analog dimming DAC bit*/
}		TLD5542_1_LEDCURRCAL_REG_T ;/**< @brief LED Current Accuracy Trimming Configuration Register*/             
    
 
typedef struct {
	uint8 FDEVSPREAD  ;	 /**< @brief [0..0] Deviation Frequency fDEV definition*/
  uint8 FMSPREAD    ;  /**< @brief [1..1] Frequency Modulation Frequency fFM definition*/
  uint8 ENSPREAD    ;	 /**< @brief [2..2] Enable Spread Spectrum feature*/
	uint8	S2G_OFF  		;  /**< @brief [3..3] Short to GND protection enable Bit*/
  uint8 VFB_VIN_OFF ;  /**< @brief [4..4] Vin Feedback Enable on VFB_VIN pin*/
  uint8	CCM_4EVER   ;  /**< @brief [5..5] Forced Continuous Conduction Mode*/
  uint8	DCM_EN   		;  /**< @brief [6..6] Enable Bit to allow DCM regulation (MOSFET M4 control)*/
} 	TLD5542_1_SWTMOD_REG_T ;/**< @brief Switching Mode Configuration Register*/					

	
typedef struct {
	uint8 IDLE  	;      /**< @brief [0..0] IDLE mode configuration bit*/
	uint8 SWRST   ;      /**< @brief [1..1] Software reset bit*/
  uint8 CLRLAT  ;      /**< @brief [2..2] Clear Latch bit*/
	uint8 ENCAL  	;      /**< @brief [3..3] Enable automatic output current calibration bit*/
  uint8	SLOPE 	;      /**< @brief [5..4] slope compensation strenght*/
  uint8 EA_GM   ;      /**< @brief [6..6] Increase the gain of the error amplifier, active only when output current is below 80% of target (i.e. < VFBH_FBL_EA )*/
  uint8 EA_IOUT	;      /**< @brief [7..7] Bit to decrease the saturation current of the error amplifier in current mode control loop*/
} 	TLD5542_1_DVCCTRL_REG_T ;/**< @brief Device Control Register */ 					
             
   
typedef struct {
	uint8 LEDCHAIN  		;  /**< @brief [3..0] MFS ratio bits: set MFS jump ratio*/
  uint8 EOMFS   			;  /**< @brief [4..4] End of MFS routine bit*/
  uint8 SOMFS  				;  /**< @brief [5..5] Start of MFS routine bit*/
	uint8 ILIM_HALF_MFS ;  /**< @brief [6..6] Bit to decrease the Switch Current Limit (ISwLim) during F.D. operation*/
  uint8 EA_IOUT_MFS		;  /**< @brief [7..7] Bit to decrease the saturation current of the error amplifier (A6) in current mode control loop only during MFS routin*/
} 	TLD5542_1_MFSSETUP1_REG_T 	;	/**< @brief Multifloat Switch configuration Register */			
                 
  
typedef struct {
  uint8 MFSDLY 	;      /**< @brief [7..0] delay time programming*/
} 	TLD5542_1_MFSSETUP2_REG_T ;	/**< @brief Multifloatswitch configuration register 2 (delay time programming)*/  			


typedef struct {
	uint8 LEDCURR			;	 /**< @brief [1..0] Status of the LED Current bits*/
	uint8	INCURR 			;  /**< @brief [3..2] Status of the Input Current bits*/
	uint8 EOMON  			;  /**< @brief [4..4] End of LED/Input Current Monitoring bit*/
	uint8 SOMON  			;  /**< @brief [5..5] Start of LED/Input Current Monitoring bit*/
} 	TLD5542_1_CURRMON_REG_T ;	/**< @brief Current Monitor Register */				
  

typedef struct {
	uint8 BB_BST_CMP	;  /**< @brief [1..0] Buck boost to Boost transition compensation level*/
	uint8	REGUMODFB 	;  /**< @brief [3..2] Feedback of Regulation Mode bits*/
} 	TLD5542_1_REGUSETMON_REG_T ; /**< @brief Regulation Setup And Monitor Register*/					
  
               
typedef struct {
	uint8 SO_MUX_SEL	;  /**< @brief [5..0] Digital Multiplexer selector, output on SO pin*/
	uint8 AMUX_EN 		;  /**< @brief [3..2] Analog Mux enable, output on FILT pin*/
	uint8	MFS_REF 		;  /**< @brief [3..2] MFS reference to FILT pin Path Enable*/
} 	TLD5542_1_MUXCTRL_REG_T; /**< @brief Internal Muxes Configuration Register*/ 					
                
  
typedef struct {
	uint8	TW					;  /**< @brief [0..0] Over Temperature Warning*/
	uint8 TSD					;  /**< @brief [1..1] Over Temperature Shutdown*/
	uint8	SHRTLED			;  /**< @brief [2..2] Shorted LED Diagnosis*/
	uint8 LEDCUR			;  /**< @brief [3..3] LED Current Flag (see LEDCUR pin description Chapter10.5)*/
	uint8 IVCCUVLO		;  /**< @brief [4..4] IVCC or IVCC_EXT Undervoltage Lockout Monitor*/
	uint8 OUTOV				;  /**< @brief [5..5] Output overvoltage Monitor*/
	uint8 EOCAL				;  /**< @brief [7..7] End of Calibration Routine*/
	uint8	EOMFS				;  /**< @brief [8..8] End of MFS Routine Bit*/
	uint8 EOMON				;  /**< @brief [9..9] End of LED/Input Current Monitor Routine Bit*/
	uint8 TER					;  /**< @brief [10..10] Transmission Error*/
	uint8 STATE				;  /**< @brief [12..11] Operative State Monitor*/
	uint8 UVLORST			;  /**< @brief [13..13] VDD OR VEN/INUVLO Undervoltage Monitor*/
	uint8	SWRST_BSTUV	;  /**< @brief [14..14] SWRST OR VBSTx-VSWNx_UVth Monitor*/
} 	TLD5542_1_STANDARD_DIAGNOSIS_REG_T; /**< @brief Standard Diagnosis */ 


typedef struct{                
	uint8	RESERVED;																/**< @brief Not used*/
  TLD5542_1_STANDARD_DIAGNOSIS_REG_T 	STANDARD_DIAGNOSIS;	/**< @brief The Standard Diagnosis reports several diagnostic informations and the status of the device and the utility routine*/
	TLD5542_1_LEDCURRADIM_REG_T 			 	LEDCURRADIM;				/**< @brief LED Current Configuration Register*/ 
	TLD5542_1_LEDCURRCAL_REG_T					LEDCURRCAL;					/**< @brief LED Current Accuracy Trimming Configuration Register*/
	TLD5542_1_SWTMOD_REG_T							SWTMOD;							/**< @brief Switching Mode Configuration Register*/
	TLD5542_1_DVCCTRL_REG_T							DVCCTRL;						/**< @brief Device Control Register */
	TLD5542_1_MFSSETUP1_REG_T						MFSSETUP1;					/**< @brief Multifloat Switch configuration Register */
	TLD5542_1_MFSSETUP2_REG_T						MFSSETUP2;					/**< @brief Multifloatswitch configuration register 2 (delay time programming)*/
	TLD5542_1_CURRMON_REG_T							CURRMON;						/**< @brief Current Monitor Register */
	TLD5542_1_REGUSETMON_REG_T					REGUSETMON;					/**< @brief Regulation Setup And Monitor Register*/
	TLD5542_1_MUXCTRL_REG_T							MUXCTRL; 						/**< @brief Internal Muxes Configuration Register*/
} TLD5542_1_Type; /**< @brief represent a local copy of the entire TLD5542-1QV register bank in the MCU memory*/	


/*****************************************************************************
										Global Variables declaration   			                  
*****************************************************************************/
extern TLD5542_1_Type tld5542_1;


/*****************************************************************************
										Global Functions declaration   			                  
*****************************************************************************/
void TLD5542_1_resetRegMapLocalCopy(void);

/*syncronous SPI transmission*/
TLD5542_1_STATUS TLD5542_1_Sync_Write_Register(TLD5542_1_REG_ADDRESS regadd);
TLD5542_1_STATUS TLD5542_1_Sync_Read_Register(TLD5542_1_REG_ADDRESS regadd);

/*asyncronous SPI transmission*/
TLD5542_1_STATUS TLD5542_1_Async_Transmit_Command(TLD5542_1_REG_ADDRESS regadd, TLD5542_1_CMD_TYPE rw);
TLD5542_1_STATUS TLD5542_1_Async_Get_Response(void);
bool TLD5542_1_Async_EoTx_Notification(void);

void TLD5542_1_testwrite(void);
void TLD5542_1_testread(void);

/*****************************************************************************
										Global Defines declaration   			                  
*****************************************************************************/

/* ============================================================================================================= */
/* ================                     TLD5542_1 BITFIELDS VALUES                              ================ */
/* ============================================================================================================= */

/** ------------------------------------------------------------------------------------------------------------**/
/* LEDCURRCAL: LED Current Accuracy Trimming Configuration Register*/
/** ------------------------------------------------------------------------------------------------------------**/
 
/* bit 6 r/w, Switch OFF internal analog dimming DAC bit*/
#define TLD5542_1_LEDCURRCAL_DAC_OFF_ENABLE_DAC  0U /**< @brief (default) internal DAC active */
#define	TLD5542_1_LEDCURRCAL_DAC_OFF_DISABLE_DAC 1U /**< @brief internal DAC inactive and analog dimming error amplifier mapped to SET pin */

/* bit 5 r/w, Start of calibration routine signalling bit*/
#define	TLD5542_1_LEDCURRCAL_SOCAL_NO_CALIB_STARTED 0U /**< @brief (default) no calibration routine started */
#define	TLD5542_1_LEDCURRCAL_SOCAL_START_CALIB      1U /**< @brief calibration routine start (autoclear) */


/* bit 4 r, End of calibration routine signalling bit*/
#define	TLD5542_1_LEDCURRCAL_EOCAL_CALIB_NOT_PERFORMED 0U /**< @brief (default) calibration routine not completed, not successfully performed or never run */
#define	TLD5542_1_LEDCURRCAL_EOCAL_CALIB_PERFORMED     1U	/**< @brief calibration successfully performed (is reset to 0B when SOCAL is set to 1B) */

/** ------------------------------------------------------------------------------------------------------------**/
/* SWTMOD: Switching Mode Configuration Register */
/** ------------------------------------------------------------------------------------------------------------**/

/* bit 6 r/w, Enable Bit to allow DCM regulation (MOSFET M4 control)*/
#define	TLD5542_1_SWTMOD_DCM_EN_DISABLE_DCM 0U /**< @brief (default) DCM is disabled (M4 alternately switching) */
#define	TLD5542_1_SWTMOD_DCM_EN_ENABLE_DCM  1U /**< @brief DCM is enabled (M4 is permanently OFF) */

/* bit 5 r/w, Forced Continuous Conduction Mode*/
#define	TLD5542_1_SWTMOD_CCM_4EVER_AFTER_SS  0U /**< @brief (default) Forced CCM after soft start finish */
#define	TLD5542_1_SWTMOD_CCM_4EVER_DURING_SS 1U /**< @brief Forced CCM even during soft start ramp up */ 

/* bit 4 r/w, Vin Feedback Enable on VFB_VIN pin*/
#define	TLD5542_1_SWTMOD_VFB_VIN_OFF_ENABLE  0U /**< @brief (default) Enable Vin Feedback */
#define	TLD5542_1_SWTMOD_VFB_VIN_OFF_DISABLE 1U /**< @brief Disable Vin Feedback */

/* bit 3 r/w, Short to GND protection enable Bit*/
#define	TLD5542_1_SWTMOD_S2G_OFF_ENABLE_PROTECTION  0U /**< @brief (default) Short to ground protection enabled */
#define	TLD5542_1_SWTMOD_S2G_OFF_DISABLE_PROTECTION 1U /**< @brief Short to ground protection disabled */

/* bit 2 r/w, Enable Spread Spectrum feature*/
#define	TLD5542_1_SWTMOD_ENSPREAD_DISABLE_SSM 0U /**< @brief (default) Spread Spectrum modulation disabled */
#define	TLD5542_1_SWTMOD_ENSPREAD_ENABLE_SSM  1U /**< @brief Spread Spectrum modulation enabled */

/* bit 1 r/w, Frequency Modulation Frequency fFM definition*/
#define	TLD5542_1_SWTMOD_FMSPREAD_12kHz 0U /**< @brief (default) 12 kHz */
#define	TLD5542_1_SWTMOD_FMSPREAD_18kHz 1U /**< @brief 18 kHz */

/* bit 0 r/w, Switching Mode Configuration Register*/
#define	TLD5542_1_SWTMOD_FDEVSPREAD_16_PERC 0U /**< @brief  (default) +- 16% of fSW */
#define	TLD5542_1_SWTMOD_FDEVSPREAD_8_PERC  1U /**< @brief  +- 8% of fSW */



/** ---------------------------------------------------------------------------------------------------------------------**/
/* DVCCTRL: Device Control Register */
/** ---------------------------------------------------------------------------------------------------------------------**/

/* bit  7 r/w, Bit to decrease the saturation current of the error */
#define	TLD5542_1_DVCCTRL_EA_IOUT_NOT_REDUCED     0U /**< @brief (default) inactive */
#define	TLD5542_1_DVCCTRL_EA_IOUT_REDUCED_TO_10uA 1U /**< @brief  active: error amplifier saturation current reduced to 10uA*/

/* bit  6 r/w, Increase the gain of the error amplifier, active only when output current is below 80% of target (i.e. < VFBH_FBL_EA ) */
#define	TLD5542_1_DVCCTRL_EA_GM_INACTIVE        0U /**< @brief (default) inactive */
#define	TLD5542_1_DVCCTRL_EA_GM_BOOST_TO_1420uS 1U /**< @brief IFBxgm boosted to 1420uS */

/* bit 5:4 r/w, slope compensation strenght */
#define	TLD5542_1_DVCCTRL_SLOPE_NOMINAL       0U /**< @brief (default) Nominal */
#define	TLD5542_1_DVCCTRL_SLOPE_PLUS_25_PERC  1U /**< @brief +25% */
#define	TLD5542_1_DVCCTRL_SLOPE_MINUS_50_PERC 2U /**< @brief -50% */
#define	TLD5542_1_DVCCTRL_SLOPE_MINUS_25_PERC 3U /**< @brief -25% */

/* bit 3 r/w, Enable automatic output current calibration bit */
#define	TLD5542_1_DVCCTRL_ENCAL_CALIB_SPI       0U /**< @brief (default) DAC takes CALIBVAL from SPI registers */
#define	TLD5542_1_DVCCTRL_ENCAL_CALIB_AUTO_PROC 1U /**< @brief DAC takes CALIBVAL from last completed automatic calibration procedure; SOCAL Bit can be set */   

/* bit 2 r/w, Clear Latch bit */
#define	TLD5542_1_DVCCTRL_CLRLAT_NORMAL_OP 0U /**< @brief (default) normal operation */
#define	TLD5542_1_DVCCTRL_CLRLAT_EXECUTE   1U /**< @brief execute CLRLAT command */   

/* bit 1 r/w, Software reset bit */
#define	TLD5542_1_DVCCTRL_SWRST_NORMAL_OP 0U /**< @brief (default) normal operation */
#define	TLD5542_1_DVCCTRL_SWRST_EXECUTE   1U /**< @brief execute reset command */   

/* bit 0 r/w, IDLE mode configuration bit */
#define	TLD5542_1_DVCCTRL_IDLE_ACTIVE 0U /**< @brief (default) ACTIVE mode */
#define	TLD5542_1_DVCCTRL_IDLE_IDLE   1U /**< @brief IDLE mode */   



/** ----------------------------------------------------------------------------------------------------------------**/
/* MFSSETUP1: Multifloat Switch configuration Register */
/** ----------------------------------------------------------------------------------------------------------------*/

/* bit  7 r/w, Bit to decrease the saturation current of the error amplifier (A6) in current mode control loop only during MFS routine */
#define	TLD5542_1_MFSSETUP1_EA_IOUT_MFS_NORMAL     				 0U /**< @brief (default) inactive */
#define	TLD5542_1_MFSSETUP1_EA_IOUT_MFS_REDUCED_TO_20_PERC 1U /**< @brief active: error amplifier current reduced to 20% */

/* bit  6 r/w, Bit to decrease the Switch Current Limit (ISwLim) during F.D. operation*/
#define	TLD5542_1_MFSSETUP1_ILIM_HALF_REDUCED_TO_2_3 0U /**< @brief (default)inactive ISwLim = 2/3of default value */
#define	TLD5542_1_MFSSETUP1_ILIM_HALF_REDUCED_TO_1_2 1U	/**< @brief  ISwLim reduced to 1/2of default value */ 

/* bit  5 r/w, Start of MFS routine biT*/
#define	TLD5542_1_MFSSETUP1_SOMFS_NOT_ACTIVE 0U	/**< @brief (default) MFS routine not activated */
#define	TLD5542_1_MFSSETUP1_SOMFS_ACTIVE     1U /**< @brief MFS routine activated */

/* bit  4 r, End of MFS routine bit*/
#define	TLD5542_1_MFSSETUP1_EOMFS_NOT_PERFORMED 0U /**< @brief (default) MFS routine not completed, not successfully performed or never run */
#define	TLD5542_1_MFSSETUP1_EOMFS_PERFORMED     1U /**< @brief MFS routine successfully performed (is reset to 0B when SOMFS is set to 1B) */ 



/** ----------------------------------------------------------------------------------------------------------------**/
/* CURRMON: Current Monitor Register */
/** ----------------------------------------------------------------------------------------------------------------**/

/* bit  5 r/w, Start of LED/Input Current Monitoring bit */
#define	TLD5542_1_CURRMON_SOMON_NOT_STARTED	 0U /**< @brief (default) Current monitor routine not started */
#define	TLD5542_1_CURRMON_SOMON_START        1U /**< @brief Start of the current monitor routine */

/* bit  4 r, End of LED/Input Current Monitoring bit */
#define	TLD5542_1_CURRMON_EOMON_NOT_PERFORMED	 0U /**< @brief  (default) Current monitoring routine not completed, not successfully performed or never run */
#define	TLD5542_1_CURRMON_EOMON_PERFORMED      1U /**< @brief  Current Monitor routine successfully performed (is reset to 0B when SOMON is set to 1B) */

/* bit 3:2 r Status of the Input Current bits */
#define	TLD5542_1_CURRMON_INCURR_BETWEEN_75_90_PERC	0U  /**< @brief (default) Input current between 75% and 90% of Limit */
#define	TLD5542_1_CURRMON_INCURR_ABOVE_90_PERC   		1U  /**< @brief Input current between 90% and the Limit */
#define	TLD5542_1_CURRMON_INCURR_BETWEEN_60_75_PERC	2U  /**< @brief Input current between 60% and 75% of Limit */
#define	TLD5542_1_CURRMON_INCURR_BELOW_60_PERC      3U  /**< @brief Input current below 60% of Limit */

/* bit 1:0 r, Status of the LED Current bits */
#define	TLD5542_1_CURRMON_LEDCURR_BETWEEN_100_125_PERC	0U /**< @brief (default) LED current between Target and +25% (100% - 125%) */
#define	TLD5542_1_CURRMON_LEDCURR_ABOVE_125_PERC   		  1U /**< @brief LED current above +25% of Target (> 125%) */
#define	TLD5542_1_CURRMON_LEDCURR_BETWEEN_75_100_PERC	  2U /**< @brief LED current between Target and -25% (75% - 100%) */
#define	TLD5542_1_CURRMON_LEDCURR_BELOW_75_PERC         3U /**< @brief LED current below -25% of Target (<75%) */



/** -------------------------------------------------------------------------------------------------------------**/
/* REGUSTEMON: Regulation Setup And Monitor Register */
/** --------------------------------------------------------------------------------------------------------------*/
/* bit 3:2 r, Feedback of Regulation Mode bits */
#define	TLD5542_1_REGUSTEMON_REGUMODFB_BUCK	 			1U /**< @brief (default) buck */
#define	TLD5542_1_REGUSTEMON_REGUMODFB_BOOST      2U /**< @brief Boost */
#define	TLD5542_1_REGUSTEMON_REGUMODFB_BUCK_BOOST 3U /**< @brief Buck-Boost */

/* bit 1:0 r/w, Buck boost to Boost transition compensation level */
#define	TLD5542_1_REGUSTEMON_BB_BST_CMP_MIN 0U /**< @brief (default) Min compensation: useful when low switching frequencies are selected */
#define	TLD5542_1_REGUSTEMON_BB_BST_CMP_LOW 1U /**< @brief Low compensation: I.E. adopted @fSW=250kHz */
#define	TLD5542_1_REGUSTEMON_BB_BST_CMP_MID 2U /**< @brief Mid compensation: I.E. adopted @fSW=400kHz */
#define	TLD5542_1_REGUSTEMON_BB_BST_CMP_MAX 3U /**< @brief Max compensation: useful when high switching frequencies are selected I.E. @fSW=500kHz */ 



/** ------------------------------------------------------------------------------------------------------------**/
/* MUXCTRL: Internal Muxes Configuration Register */
/** ------------------------------------------------------------------------------------------------------------**/

/* bit 7 r/w, MFS reference to FILT pin Path Enable */
#define	TLD5542_1_MUXCTRL_MSF_REF_PATH_DISABLED 0U /**< @brief (default) path disabled */
#define	TLD5542_1_MUXCTRL_MSF_REF_PATH_ENABLED  1U /**< @brief Path Enabled */

/* bit 6 r/w, Analog Mux enable, output on FILT pin */
#define	TLD5542_1_MUXCTRL_AMUX_EN_DISABLED 0U /**< @brief (default) Analog Mux disabled */
#define	TLD5542_1_MUXCTRL_AMUX_EN_ENABLED  1U /**< @brief Analog Mux enabled */

/* bit 6 r/w, Analog Mux enable, output on FILT pin */
#define	TLD5542_1_MUXCTRL_SO_MUX_SEL_SO_AS_SPI 					 0U /**< @brief (default) Analog Mux disabled */
#define	TLD5542_1_MUXCTRL_SO_MUX_SEL_SO_AS_INTERNAL_CLK  4U /**< @brief Analog Mux enabled */ 

/** ------------------------------------------------------------------------------------------------------------**/
/* STANDARD DIAGNOSIS: The Standard Diagnosis reports several diagnostic informations and the status of the device and the utility routines */
/** ------------------------------------------------------------------------------------------------------------**/

/* bit 14 r, SWRST OR VBSTx-VSWNx_UVth Monitor */
#define	TLD5542_1_STD_SWRST_BSTUV_NOT_OCCURED	0U /**< @brief no SWRST or undervoltage on the Gate Drivers occured */
#define	TLD5542_1_STD_SWRST_BSTUV_OCCURED 		1U /**< @brief there was at least one SWRST since last readout OR an undervoltage condition at the gate drivers is occurred */

/* bit 13 r, VDD OR VEN/INUVLO Undervoltage Monitor*/
#define	TLD5542_1_STD_UVLORST_NOT_OCCURED	0U /**< @brief there was no VDD OR VEN/INUVLO undervoltage since last readout */
#define	TLD5542_1_STD_UVLORST_OCCURED 		1U /**< @brief there was at least one VDD undervoltage OR VEN/INUVLO undervoltage condition since last readout */ 

/* bit 12:11 r, Operative State Monitor*/
#define	TLD5542_1_STD_STATE_RESERVED	0U 	/**< @brief (reserved) */
#define	TLD5542_1_STD_STATE_LIMPHOME 	1U	/**< @brief Limp Home Mode */
#define	TLD5542_1_STD_STATE_ACTIVE 		2U	/**< @brief Active Mode*/
#define	TLD5542_1_STD_STATE_IDLE 			3U	/**< @brief Idle Mode*/

/* bit 10 r, Operative State Monitor*/
#define	TLD5542_1_STD_TER_TX_SUCCESS	0U 	/**< @brief Previous transmission was successful (modulo 16 + n*8 clocks received, where n = 0, 1, 2...) */
#define	TLD5542_1_STD_TER_TX_FAILED 	1U	/**< @brief Previous transmission failed or first transmission after reset */

/* bit 9 r, Operative State Monitor*/
#define	TLD5542_1_STD_EOMON_NOT_PERFORMED 0U 	/**< @brief Current monitoring routine not completed, not successfully performed or never run */
#define	TLD5542_1_STD_EOMON_PERFORMED 		1U	/**< @brief Current Monitor routine successfully performed (is reset to 0B when SOMON is set to 1B) */ 

/* bit 8 r, End of MFS routine bit*/
#define	TLD5542_1_STD_EOMFS_NOT_PERFORMED 0U /**< @brief (default) MFS routine not completed, not successfully performed or never run */
#define	TLD5542_1_STD_EOMFS_PERFORMED     1U /**< @brief MFS routine successfully performed (is reset to 0B when SOMFS is set to 1B)*/

/* bit 7 r, End of calibration routine signalling bit*/
#define	TLD5542_1_STD_EOCAL_CALIB_NOT_PERFORMED 0U /**< @brief (default) calibration routine not completed, not successfully performed or never run */
#define	TLD5542_1_STD_EOCAL_CALIB_PERFORMED     1U /**< @brief calibration successfully performed (is reset to 0B when SOCAL is set to 1B) */

/* bit 5 r, Output overvoltage Monitor*/
#define	TLD5542_1_STD_OUTOV_OUTPUT_OVERVOLTAGE_NOT_OCCURED 0U	/**< @brief Output overvoltage not detected since last readout */
#define	TLD5542_1_STD_OUTOV_OUTPUT_OVERVOLTAGE_OCCURED     1U	/**< @brief Output overvoltage was detected since last readout*/ 

/* bit 4 r, IVCC or IVCC_EXT Undervoltage Lockout Monitor*/
#define	TLD5542_1_STD_IVCCUVLO_UNDERVOLTAGE_NOT_OCCURED 0U	/**< @brief IVCC and IVCC_EXT above VIVCC_RTH,d or VIVCC_EXT_RTH,d threshold since last readout */
#define	TLD5542_1_STD_IVCCUVLO_UNDERVOLTAGE_OCCURED     1U	/**< @brief Undervoltage on IVCC or IVCC_EXT occurred since last readout*/

/* bit 3 r, LED Current Flag (see LEDCUR pin description Chapter10.5) */
#define	TLD5542_1_STD_LEDCUR_NOT_DETECTED 0U	/**< @brief LED current not detected*/
#define	TLD5542_1_STD_LEDCUR_DETECTED     1U	/**< @brief LED current detected*/

/* bit 2 r, Shorted LED Diagnosis */
#define	TLD5542_1_STD_SHRTLED_OUTPUT_SHORT_CIRCUIT_NOT_OCCURED 0U	/**< @brief Short circuit condition not detected since last readout*/
#define	TLD5542_1_STD_SHRTLED_OUTPUT_SHORT_CIRCUIT_OCCURED     1U	/**< @brief Short circuit condition detected since last readout*/ 

/* bit 1 r, Over Temperature Shutdown */
#define	TLD5542_1_STD_TSD_OVERTEMPERATURE_NOT_OCCURED  0U	/**< @brief Tj below temperature shutdown threshold */
#define	TLD5542_1_STD_TSD_OVERTEMPERATURE_OCCURED    	 1U	/**< @brief Overtemperature condition detected since last readout*/

/* bit 1 r, Over Temperature Shutdown */
#define	TLD5542_1_STD_TW_TEMP_WARNING_NOT_OCCURED  0U	/**< @brief Tj below temperature shutdown threshold */
#define	TLD5542_1_STD_TW_TEMP_WARNING_OCCURED    	 1U	/**< @brief Overtemperature condition detected since last readout*/

#endif /* TLD5542_1_REGLAYER_H */
