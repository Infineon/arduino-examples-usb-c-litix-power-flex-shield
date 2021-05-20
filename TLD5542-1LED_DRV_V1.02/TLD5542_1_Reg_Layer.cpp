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
 * \file     TLD5542_1_Reg_Layer.c
 *
 * \brief    TLD5542_1 Register Layer Source Code
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

#include "TLD5542_1_Reg_Layer.h"
#include <Arduino.h>
#include <SPI.h>

// set pin 10 as the slave select for the digital pot:
#define CSN_TLD_PIN 10

/*****************************************************************************
                    Global Defines declaration                          
*****************************************************************************/

#define POWERFLEX_SPI_STANDARD_DIAGNOSIS_MSG        0x0001U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame to request STD readout*/ 
#define POWERFLEX_SPI_STANDARD_DIAGNOSIS_B15_MASK   0x8000U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame mask to identify SPI STD response (MSB = HIGH)*/

#define POWERFLEX_SPI_ACTION_WRITE                  0x8000U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame mask to set SPI WRITE ACTION (MSB = HIGH)*/
#define POWERFLEX_SPI_ACTION_READ                   0x0000U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame mask to set SPI WRITE ACTION (MSB = LOW)*/

#define POWERFLEX_SPI_ADDRESS_OFFSET_SHIFT               8U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame ADDRESS [b15-b8] postion offset */
#define POWERFLEX_SPI_ADDRESS_MASK                  0x7F00U /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame mask to identify register address [b14-b8]*/
#define POWERFLEX_SPI_DATA_MASK                     0x00FFU /**< @brief TLD5542-1QV/TLD5501-2QV SPI frame mask to identify register data [b7-b0]*/



/*****************************************************************************
										Global Variable declaration   			                  
*****************************************************************************/

TLD5542_1_Type tld5542_1 = {
0,/*RESERVED*/
{0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u},/*STANDARD_DIAGNOSIS_REG_T: TW, TSD, SHRTLED, LEDCUR, IVCCUVLO, OUTOVM, EOCAL, EOMFS, EOMON, TER, STATE, UVLORST, SWRST_BSTUV, fault, reset*/
{0xF0},																/*LEDCURRADIM_REG_T: ADIMVAL*/
{0x00,0x00,0x00,0x00},								/*LEDCURRCAL_REG_T: CALIBVAL, EOCAL, SOCAL, DAC_OFF*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00},	/*SWTMOD_REG_T: FDEVSPREAD, FMSPREAD, ENSPREAD, S2G_OFF, VFB_VIN_OFF, CCM4EVER, DCM_EN*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00},	/*DVCCTRL_REG_T: IDLE, SWRST, CLRLAT, ENCAL, SLOPE, EA_GM, EA_IOUT*/
{0x08,0x00,0x00,0x00,0x00},						/*MFSSETUP1_REG_T: LEDCHAIN, EOMFS, SOMFS, ILIM_HALF_MFS, EA_IOUT_MFS*/
{0x80},																/*MFSSETUP2_REG_T: MFSDLY*/
{0x00,0x00,0x00,0x00},								/*CURRMON_REG_T: */
{0x00,0x01},													/*REGUSETMON_REG_T*/
{0x00,0x00,0x00}											/*MUXCTRL_REG_T*/
}; /**< @brief represent a local copy of the entire TLD5542-1QV register bank in the MCU memory*/

static const  TLD5542_1_Type tld5542_1_default = {
0,/*RESERVED*/
{0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u,0x00u},/*STANDARD_DIAGNOSIS_REG_T: TW, TSD, SHRTLED, LEDCUR, IVCCUVLO, OUTOVM, EOCAL, EOMFS, EOMON, TER, STATE, UVLORST, SWRST_BSTUV, fault, reset*/
{0xF0},																/*LEDCURRADIM_REG_T: ADIMVAL*/
{0x00,0x00,0x00,0x00},								/*LEDCURRCAL_REG_T: CALIBVAL, EOCAL, SOCAL, DAC_OFF*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00},	/*SWTMOD_REG_T: FDEVSPREAD, FMSPREAD, ENSPREAD, S2G_OFF, VFB_VIN_OFF, CCM4EVER, DCM_EN*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00},	/*DVCCTRL_REG_T: IDLE, SWRST, CLRLAT, ENCAL, SLOPE, EA_GM, EA_IOUT*/
{0x08,0x00,0x00,0x00,0x00},						/*MFSSETUP1_REG_T: LEDCHAIN, EOMFS, SOMFS, ILIM_HALF_MFS, EA_IOUT_MFS*/
{0x80},																/*MFSSETUP2_REG_T: MFSDLY*/
{0x00,0x00,0x00,0x00},								/*CURRMON_REG_T: */
{0x00,0x01},													/*REGUSETMON_REG_T*/
{0x00,0x00,0x00}											/*MUXCTRL_REG_T*/
}; /**< @brief Rappresent TLD5542-1QV default values. It is used to initialize tld5542_1*/


/*****************************************************************************
										Local Functions declaration   			                  
*****************************************************************************/

static uint16 TLD5542_1_set_SPI_command(TLD5542_1_REG_ADDRESS regadd, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_SPI_response(uint16 response, TLD5542_1_STATUS *status);

static void TLD5542_1_get_StandardDiagnosisCmd(TLD5542_1_STANDARD_DIAGNOSIS_REG_T *stdreg, uint16 std, TLD5542_1_STATUS *status);

static void TLD5542_1_set_LedcurradimCmd(const TLD5542_1_LEDCURRADIM_REG_T* ledcurradimreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_LedcurradimCmd(TLD5542_1_LEDCURRADIM_REG_T* ledcurradimreg, uint8 data);

static void TLD5542_1_set_LedcurrcalCmd(const TLD5542_1_LEDCURRCAL_REG_T* ledcurrcalreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_LedcurrcalCmd(TLD5542_1_LEDCURRCAL_REG_T* ledcurrcalreg, uint8 data);

static void TLD5542_1_set_SwtmodCmd(const TLD5542_1_SWTMOD_REG_T* swtmodreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_SwtmodCmd(TLD5542_1_SWTMOD_REG_T* swtmodreg, uint8 data);

static void TLD5542_1_set_DvcctrlCmd(const TLD5542_1_DVCCTRL_REG_T* dvcctrlreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_DvcctrlCmd(TLD5542_1_DVCCTRL_REG_T* dvcctrlreg, uint8 data);

static void TLD5542_1_set_Mfssetup1Cmd(const TLD5542_1_MFSSETUP1_REG_T* mfssetup1reg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_Mfssetup1Cmd(TLD5542_1_MFSSETUP1_REG_T* mfssetup1reg, uint8 data);

static void TLD5542_1_set_Mfssetup2Cmd(const TLD5542_1_MFSSETUP2_REG_T* mfssetup2reg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_Mfssetup2Cmd(TLD5542_1_MFSSETUP2_REG_T* mfssetup2reg, uint8 data);

static void TLD5542_1_set_CurrmonCmd(const TLD5542_1_CURRMON_REG_T* currmonreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_CurrmonCmd(TLD5542_1_CURRMON_REG_T* currmonreg, uint8 data);

static void TLD5542_1_set_RegusetmonCmd(const TLD5542_1_REGUSETMON_REG_T* regusetmonreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_RegusetmonCmd(TLD5542_1_REGUSETMON_REG_T* regusetmonreg, uint8 data);

static void TLD5542_1_set_MuxctrlCmd(const TLD5542_1_MUXCTRL_REG_T* muxctrlreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw);
static void TLD5542_1_get_MuxctrlCmd(TLD5542_1_MUXCTRL_REG_T* muxctrlreg, uint8 data);


/*****************************************************************************
										Local Defines declaration   			                  
*****************************************************************************/


/* ============================================================================================================= */
/* ================                      TLD5542_1 REG ADDRESS                                  ================ */
/* ============================================================================================================= */

#define TLD5542_1_STANDARDSIAGNOSIS_ADDR_MSB	(0x00U)
#define TLD5542_1_LEDCURRADIM_ADDR_MSB				(0x00U)
#define TLD5542_1_LEDCURRCAL_ADDR_MSB 				(0x03U)
#define TLD5542_1_SWTMOD_ADDR_MSB 						(0x05U)
#define TLD5542_1_DVCCTRL_ADDR_MSB 						(0x06U)
#define TLD5542_1_MFSSETUP1_ADDR_MSB 					(0x09U)
#define TLD5542_1_MFSSETUP2_ADDR_MSB 					(0x0AU)
#define TLD5542_1_CURRMON_ADDR_MSB 						(0x0CU)
#define TLD5542_1_REGUSETMON_ADDR_MSB 				(0x0FU)
#define TLD5542_1_MUXCTRL_ADDR_MSB 						(0x7EU)

#define TLD5542_1_STANDARDSIAGNOSIS_ADDR_LSB	(0x01U)
#define TLD5542_1_LEDCURRADIM_ADDR_LSB				(0x00U)
#define TLD5542_1_LEDCURRCAL_ADDR_LSB 				(0x00U)
#define TLD5542_1_SWTMOD_ADDR_LSB 						(0x00U)
#define TLD5542_1_DVCCTRL_ADDR_LSB						(0x00U)
#define TLD5542_1_MFSSETUP1_ADDR_LSB 					(0x00U)
#define TLD5542_1_MFSSETUP2_ADDR_LSB 					(0x00U)
#define TLD5542_1_CURRMON_ADDR_LSB						(0x00U)
#define TLD5542_1_REGUSETMON_ADDR_LSB					(0x00U)
#define TLD5542_1_MUXCTRL_ADDR_LSB						(0x00U)

/* ============================================================================================================= */
/* ================                     TLD5542_1 REG POS and MASK                              ================ */
/* ============================================================================================================= */

/* ===================================== STANDARD DIAGNOSIS ==================================================== */
#define TLD5542_1_STD_SWRST_BSTUV_Pos (14UL)      			/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SWRST_BSTUV (Bit 14-14*/
#define TLD5542_1_STD_SWRST_BSTUV_Msk		(0x4000UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SWRST_BSTUV (Bitfield-Mask: 0x4000)*/
#define TLD5542_1_STD_UVLORST_Pos 		(13U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: UVLRST (Bit 13-13*/
#define TLD5542_1_STD_UVLORST_Msk 			(0x2000UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: UVLRST (Bitfield-Mask: 0x2000)*/
#define TLD5542_1_STD_STATE_Pos 			(11U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: STATE (Bit 12-11*/
#define TLD5542_1_STD_STATE_Msk 				(0x1800UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: STATE (Bitfield-Mask: 0x1800)*/
#define TLD5542_1_STD_TER_Pos 				(10U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TER (Bit 10-10*/
#define TLD5542_1_STD_TER_Msk 					(0x0400UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TER (Bitfield-Mask: 0x0400)*/
#define TLD5542_1_STD_EOMON_Pos 			(9U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOMON (Bit 9-9*/
#define TLD5542_1_STD_EOMON_Msk 				(0x0200UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOMON (Bitfield-Mask: 0x0200)*/
#define TLD5542_1_STD_EOMFS_Pos 			(8U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOMFS (Bit 8-8*/
#define TLD5542_1_STD_EOMFS_Msk 				(0x0100UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOMFS (Bitfield-Mask: 0x0100)*/
#define TLD5542_1_STD_EOCAL_Pos 			(7U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOCAL (Bit 7-7*/
#define TLD5542_1_STD_EOCAL_Msk 				(0x0080UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: EOCAL (Bitfield-Mask: 0x0080)*/
#define TLD5542_1_STD_OUTOV_Pos 			(5U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: OUTOV (Bit 5-5*/
#define TLD5542_1_STD_OUTOV_Msk 				(0x0020UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: OUTOV (Bitfield-Mask: 0x0020)*/
#define TLD5542_1_STD_IVCCUVLO_Pos		(4U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: IVCCUVLO (Bit 4-4*/
#define TLD5542_1_STD_IVCCUVLO_Msk 			(0x0010UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: IVCCUVLO (Bitfield-Mask: 0x0010)*/
#define TLD5542_1_STD_LEDCUR_Pos 		  (3U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: LEDCURR (Bit 3-3*/
#define TLD5542_1_STD_LEDCUR_Msk 			  (0x0008UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: LEDCURR (Bitfield-Mask: 0x0008)*/
#define TLD5542_1_STD_SHRTLED_Pos 		(2U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SHRTLED (Bit 2-2*/
#define TLD5542_1_STD_SHRTLED_Msk 			(0x0004UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SHRTLED (Bitfield-Mask: 0x0004*/
#define TLD5542_1_STD_TSD_Pos 				(1U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TSD (Bit 1-1*/
#define TLD5542_1_STD_TSD_Msk 					(0x0002UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TSD (Bitfield-Mask: 0x0002)*/
#define TLD5542_1_STD_TW_Pos 					(0U)      				/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TW (Bit 0-0*/
#define TLD5542_1_STD_TW_Msk 						(0x0001UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: TW (Bitfield-Mask: 0x0001)*/
#define TLD5542_1_STD_ERROR_Msk 				(0x6437UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SWRST_BSTUV & UVLORST & TER & IVCCUVLO & TSD & TW  & SHRTLED (Bitfield-Mask: 0x6413)*/
//#define TLD5542_1_STD_ERROR_Msk         (0x6413UL)      /**< @brief TLD5542_1 STANDARD DIAGNOSIS: SWRST_BSTUV & UVLORST & TER & IVCCUVLO & TSD & TW (Bitfield-Mask: 0x6413)*/
#define TLD5542_1_STD_RESET_Msk 				(0x4400UL)  		/**< @brief TLD5542_1 STANDARD DIAGNOSIS: SWRST_BSTUV & TER  (Bitfield-Mask: 0x4400)*/
/* ========================================= LEDCURRADIM ======================================================= */
#define TLD5542_1_LEDCURRADIM_ADIMVAL_Pos (0U)      		/**< @brief TLD5542_1 LEDCURRADIM: ADIMVAL (Bit 7-0*/
#define TLD5542_1_LEDCURRADIM_ADIMVAL_Msk 	(0xFFU)     /**< @brief TLD5542_1 LEDCURRADIM: ADIMVAL (Bitfield-Mask: 0xFF)*/
/* ========================================== LEDCURRCAL ======================================================= */
#define TLD5542_1_LEDCURRCAL_DAC_OFF_Pos  (6U)      		/**< @brief TLD5542_1 LEDCURRCAL: DAC_OFF (Bit 6)*/
#define TLD5542_1_LEDCURRCAL_DAC_OFF_Msk  	(0x40U)   	/**< @brief TLD5542_1 LEDCURRCAL: DAC_OFF (Bitfield-Mask: 0x40)*/
#define TLD5542_1_LEDCURRCAL_SOCAL_Pos    (5U)      		/**< @brief TLD5542_1 LEDCURRCAL: SOCAL (Bit 5)*/
#define TLD5542_1_LEDCURRCAL_SOCAL_Msk    	(0x20U)   	/**< @brief TLD5542_1 LEDCURRCAL: SOCAL (Bitfield-Mask: 0x20)*/
#define TLD5542_1_LEDCURRCAL_EOCAL_Pos    (4U)      		/**< @brief TLD5542_1 LEDCURRCAL: EOCAL (Bit 4)*/
#define TLD5542_1_LEDCURRCAL_EOCAL_Msk    	(0x10U)   	/**< @brief TLD5542_1 LEDCURRCAL: EOCAL (Bitfield-Mask: 0x10)*/
#define TLD5542_1_LEDCURRCAL_CALIBVAL_Pos	(0U)      		/**< @brief TLD5542_1 LEDCURRCAL: CALIBVAL (Bit 3-0)*/
#define TLD5542_1_LEDCURRCAL_CALIBVAL_Msk 	(0x0FU)   	/**< @brief TLD5542_1 LEDCURRCAL: CALIBVAL (Bitfield-Mask: 0x0F)*/
/* ============================================ SWTMOD ========================================================= */
#define TLD5542_1_SWTMOD_DCM_EN_Pos       (6U)      		/**< @brief TLD5542_1 SWTMOD: DCM_EN (Bit 6)*/
#define TLD5542_1_SWTMOD_DCM_EN_Msk       	(0x40U)   	/**< @brief TLD5542_1 SWTMOD: DCM_EN (Bitfield-Mask: 0x40)*/
#define TLD5542_1_SWTMOD_CCM4EVER_Pos     (5U)      		/**< @brief TLD5542_1 SWTMOD: CCM4EVER (Bit 5)*/
#define TLD5542_1_SWTMOD_CCM4EVER_Msk     	(0x20U)   	/**< @brief TLD5542_1 SWTMOD: CCM4EVER (Bitfield-Mask: 0x20)*/
#define TLD5542_1_SWTMOD_VFB_VIN_OFF_Pos  (4U)      		/**< @brief TLD5542_1 SWTMOD: VFB_VIN_OFF (Bit 4)*/
#define TLD5542_1_SWTMOD_VFB_VIN_OFF_Msk		(0x10U)   	/**< @brief TLD5542_1 SWTMOD: VFB_VIN_OFF (Bitfield-Mask: 0x10)*/
#define TLD5542_1_SWTMOD_S2G_OFF_Pos      (3U)      		/**< @brief TLD5542_1 SWTMOD: S2G_OFF (Bit 3)*/
#define TLD5542_1_SWTMOD_S2G_OFF_Msk      	(0x08U)   	/**< @brief TLD5542_1 SWTMOD: S2G_OFF (Bitfield-Mask: 0x08)*/
#define TLD5542_1_SWTMOD_ENSPREAD_Pos     (2U)      		/**< @brief TLD5542_1 SWTMOD: ENSPREAD (Bit 2)*/
#define TLD5542_1_SWTMOD_ENSPREAD_Msk     	(0x04U)   	/**< @brief TLD5542_1 SWTMOD: ENSPREAD (Bitfield-Mask: 0x04)*/
#define TLD5542_1_SWTMOD_FMSPREAD_Pos     (1U)      		/**< @brief TLD5542_1 SWTMOD: FMSPREAD (Bit 1)*/
#define TLD5542_1_SWTMOD_FMSPREAD_Msk     	(0x02U)   	/**< @brief TLD5542_1 SWTMOD: FMSPREAD (Bitfield-Mask: 0x02)*/
#define TLD5542_1_SWTMOD_FDEVSPREAD_Pos   (0U)      		/**< @brief TLD5542_1 SWTMOD: FDEVSPREAD (Bit 0)*/
#define TLD5542_1_SWTMOD_FDEVSPREAD_Msk   	(0x01U)   	/**< @brief TLD5542_1 SWTMOD: FDEVSPREAD (Bitfield-Mask: 0x01)*/
/* =========================================== DVCCTRL ========================================================= */
#define TLD5542_1_DVCCTRL_EA_IOUT_Pos     (7U)      		/**< @brief TLD5542_1 DVCCTRL: EA_IOUT (Bit 7)*/
#define TLD5542_1_DVCCTRL_EA_IOUT_Msk     	(0x80U)   	/**< @brief TLD5542_1 DVCCTRL: EA_IOUT (Bitfield-Mask: 0x80)*/
#define TLD5542_1_DVCCTRL_EA_GM_Pos       (6U)      		/**< @brief TLD5542_1 DVCCTRL: EA_GM (Bit 6)*/
#define TLD5542_1_DVCCTRL_EA_GM_Msk       	(0x40U)   	/**< @brief TLD5542_1 DVCCTRL: EA_GM (Bitfield-Mask: 0x40)*/
#define TLD5542_1_DVCCTRL_SLOPE_Pos       (4U)      		/**< @brief TLD5542_1 DVCCTRL: SLOPE (Bit 5-4)*/
#define TLD5542_1_DVCCTRL_SLOPE_Msk       	(0x30U)   	/**< @brief TLD5542_1 DVCCTRL: SLOPE (Bitfield-Mask: 0x30)    */
#define TLD5542_1_DVCCTRL_ENCAL_Pos       (3U)      		/**< @brief TLD5542_1 DVCCTRL: ENCAL (Bit 3)*/
#define TLD5542_1_DVCCTRL_ENCAL_Msk       	(0x08U)   	/**< @brief TLD5542_1 DVCCTRL: ENCAL (Bitfield-Mask: 0x08)*/
#define TLD5542_1_DVCCTRL_CLRLAT_Pos      (2U)      		/**< @brief TLD5542_1 DVCCTRL: CLRLAT (Bit 2)*/
#define TLD5542_1_DVCCTRL_CLRLAT_Msk				(0x04U)   	/**< @brief TLD5542_1 DVCCTRL: CLRLAT (Bitfield-Mask: 0x04)*/
#define TLD5542_1_DVCCTRL_SWRST_Pos       (1U)      		/**< @brief TLD5542_1 DVCCTRL: SWRST (Bit 1)*/
#define TLD5542_1_DVCCTRL_SWRST_Msk       	(0x02U)   	/**< @brief TLD5542_1 DVCCTRL: SWRST (Bitfield-Mask: 0x02)*/
#define TLD5542_1_DVCCTRL_IDLE_Pos       	(0U)      		/**< @brief TLD5542_1 DVCCTRL: IDLE (Bit 0)*/
#define TLD5542_1_DVCCTRL_IDLE_Msk       		(0x01U)  		/**< @brief TLD5542_1 DVCCTRL: IDLE (Bitfield-Mask: 0x01)*/
/* ========================================== MFSSETUP1 ======================================================== */
#define TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Pos   (7U)    	/**< @brief TLD5542_1 MFSSETUP1: EA_IOUT_MFS (Bit 7)*/
#define TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Msk   	(0x80U) /**< @brief TLD5542_1 MFSSETUP1: EA_IOUT_MFS (Bitfield-Mask: 0x80)*/
#define TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Pos (6U)     	/**< @brief TLD5542_1 MFSSETUP1: ILIM_HALF_MFS (Bit 6)*/
#define TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Msk		(0x40U) /**< @brief TLD5542_1 MFSSETUP1: ILIM_HALF_MFS (Bitfield-Mask: 0x40)*/
#define TLD5542_1_MFSSETUP1_SOMFS_Pos       	(5U)     	/**< @brief TLD5542_1 MFSSETUP1: SOMFS_MFS (Bit 5)*/
#define TLD5542_1_MFSSETUP1_SOMFS_Msk       		(0x20U) /**< @brief TLD5542_1 MFSSETUP1: SOMFS_MFS (Bitfield-Mask: 0x20)*/
#define TLD5542_1_MFSSETUP1_EOMFS_Pos       	(4U)     	/**< @brief TLD5542_1 MFSSETUP1: EOMFS_MFS (Bit 4)*/
#define TLD5542_1_MFSSETUP1_EOMFS_Msk       		(0x10U) /**< @brief TLD5542_1 MFSSETUP1: EOMFS_MFS (Bitfield-Mask: 0x10)*/
#define TLD5542_1_MFSSETUP1_LEDCHAIN_Pos      (0U)     	/**< @brief TLD5542_1 MFSSETUP1: LEDCHAIN (Bit 3-0)*/
#define TLD5542_1_MFSSETUP1_LEDCHAIN_Msk      	(0x0FU) /**< @brief TLD5542_1 MFSSETUP1: LEDCHAIN (Bitfield-Mask: 0x0F)*/
/* ========================================= MFSSETUP2 ========================================================= */
#define TLD5542_1_MFSSETUP2_MFSDLY_Pos	(0U)           	/**< @brief TLD5542_1 MFSSETUP2: MFSDLY (Bit 7-0)*/
#define TLD5542_1_MFSSETUP2_MFSDLY_Msk  	(0xFFU)       /**< @brief TLD5542_1 MFSSETUP2: MFSDLY (Bitfield-Mask: 0xFF)*/
/* ========================================== CURRMON ========================================================== */
#define TLD5542_1_CURRMON_SOMON_Pos   (5U)             	/**< @brief TLD5542_1 CURRMON: SOMON (Bit 5)*/
#define TLD5542_1_CURRMON_SOMON_Msk   	(0x20U)         /**< @brief TLD5542_1 CURRMON: SOMON (Bitfield-Mask: 0x20)*/
#define TLD5542_1_CURRMON_EOMON_Pos   (4U)             	/**< @brief TLD5542_1 CURRMON: EOMON (Bit 4)*/
#define TLD5542_1_CURRMON_EOMON_Msk   	(0x10U)         /**< @brief TLD5542_1 CURRMON: EOMON (Bitfield-Mask: 0x10)*/
#define TLD5542_1_CURRMON_INCURR_Pos  (2U)             	/**< @brief TLD5542_1 CURRMON: INCURR (Bit 3-2)*/
#define TLD5542_1_CURRMON_INCURR_Msk  	(0x0CU)         /**< @brief TLD5542_1 CURRMON: INCURR (Bitfield-Mask: 0x0C)*/
#define TLD5542_1_CURRMON_LEDCURR_Pos	(0U)             	/**< @brief TLD5542_1 CURRMON: LEDCURR (Bit 1-0)*/
#define TLD5542_1_CURRMON_LEDCURR_Msk 	(0x03U)         /**< @brief TLD5542_1 CURRMON: LEDCURR (Bitfield-Mask: 0x03)*/
/* ========================================= REGUSETMON  ======================================================= */
#define TLD5542_1_REGUSETMON_REGUMODFB_Pos  (2U)       	/**< @brief TLD5542_1 REGUSETMON: REGUMODFB (Bit 3-2)*/
#define TLD5542_1_REGUSETMON_REGUMODFB_Msk  	(0x0CU)   /**< @brief TLD5542_1 REGUSETMON: REGUMODFB (Bitfield-Mask: 0x0C)*/
#define TLD5542_1_REGUSETMON_BB_BST_CMP_Pos	(0U)       	/**< @brief TLD5542_1 REGUSETMON: BB_BST_CMP (Bit 1-0)*/
#define TLD5542_1_REGUSETMON_BB_BST_CMP_Msk 	(0x03U)   /**< @brief TLD5542_1 REGUSETMON: BB_BST_CMP (Bitfield-Mask: 0x03)*/
/* ========================================== MUXCTRL  ========================================================= */
#define TLD5542_1_MUXCTRL_MFS_REF_Pos     (7U)         	/**< @brief TLD5542_1 MUXCTRL: MFS_REF (Bit 7)*/
#define TLD5542_1_MUXCTRL_MFS_REF_Msk     	(0x80U)     /**< @brief TLD5542_1 MUXCTRL: MFS_REF (Bitfield-Mask: 0x80)*/
#define TLD5542_1_MUXCTRL_AMUX_EN_Pos     (6U)         	/**< @brief TLD5542_1 MUXCTRL: AMUX_EN (Bit 6)*/
#define TLD5542_1_MUXCTRL_AMUX_EN_Msk     	(0x40U)     /**< @brief TLD5542_1 MUXCTRL: AMUX_EN (Bitfield-Mask: 0x40)*/
#define TLD5542_1_MUXCTRL_SO_MUX_SEL_Pos	(0U)         	/**< @brief TLD5542_1 MUXCTRL: SO_MUX_SEL (Bit 5-0)*/
#define TLD5542_1_MUXCTRL_SO_MUX_SEL_Msk  	(0x3FU)     /**< @brief TLD5542_1 MUXCTRL: SO_MUX_SEL (Bitfield-Mask: 0x3F)*/

/*****************************************************************************
										Global Functions definition   			                  
*****************************************************************************/

/** @brief Resets the global variable tld5542_1 with TLD5542-1QV SPI registers default values (see datasheet 12.6.2- Table14 of TLD5542-1QV datasheet).*/
void TLD5542_1_resetRegMapLocalCopy(void){
	tld5542_1 = tld5542_1_default;
}


void printByte(byte printByte)          // This function writes to the serial monitor the value of the byte printByte in binary form 
{
  for (int i = 7; i >= 0; i--)
  {
    Serial.print(bitRead(printByte, i));
  }
  Serial.print("\t");
}


bool PowerFlex_Sync_Write_Register(uint16 cmd, uint16 *response){
  bool statusSpi = true;
  //Arduino : set SPI speed and mode for powerFlex, 
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  digitalWrite(CSN_TLD_PIN, LOW);               // take the CSN pin low to select the chip:
  byte tmp= cmd>>8; //extract first byte
  byte byte0 = SPI.transfer(tmp);   // send address byte to the TLE94112 and store the received byte to byte 0
  tmp= cmd&0X00FF;  // extract second byte
  byte byte1 = SPI.transfer(tmp);  // send data byte to the TLE94112 and store the received byte to byte 1
  digitalWrite(CSN_TLD_PIN, HIGH);              // take the CSN pin high to terminate the SPI frame
  //Arduino
  SPI.endTransaction();// stop using SPI bus, allow other libraries to use the SPI bus

  *response= (uint16)byte0<<8 |(uint16)byte1;
   return statusSpi;
}


/** @brief Transmits a synchronous SPI frame to write an SPI register (user code is blocked during SPI transmission). Received response is used to updated tld5542_1 global variable.
 *
 *Generates the SPI frame to updated the register using data stored into  tld5542_1. Communicate the SPI frame using synchronous APIs offered by 
 *communication layer. When transmission is over updates tld5542_1 with the received SPI response
 *\param[in] regAdd	Unique identifier of the TLD5542-1QV SPI register address
 *\return TLD5542-1QV status, Quick evaluation of SPI communication
*/
TLD5542_1_STATUS TLD5542_1_Sync_Write_Register(TLD5542_1_REG_ADDRESS regadd){
	TLD5542_1_STATUS status = TLD5542_1_UNKNOWN;
	uint16 response;
	uint16 cmd = TLD5542_1_set_SPI_command(regadd, TLD5542_1_WRITE_CMD);
  
	if(PowerFlex_Sync_Write_Register(cmd, &response) == true){
		TLD5542_1_get_SPI_response(response, &status);
	}else{
		status = TLD5542_1_SPI_NOT_OK;
	}
	
	return status;	
}


/** @brief Transmits one or two synchronous SPI frames to read a SPI register and returns the received SPI responses (user code is blocked during SPI transmissions).
 *
 *Puts cmd in the SPI transmission buffer. Starts a synchronous SPI transmission. When transmission is over gets the SPI response from the SPI receiver buffer.
 *Puts a read STANDARD DIAGNOSIS command in the SPI transmission buffer. Starts a synchronous SPI transmission. When transmission is over gets the SPI response from the SPI receiver buffer.
 *The second response shall contain register content requested with the first SPI transmission (cmd).
 *If cmd is a STANDARD DIAGNOSIS request and response1 is a STANDARD DIAGNOSIS response, the second SPI frame is not transmitted and response2 is set equal to response1.
 *\note TLD5542-1QV and TLD5501-2QV responds with a register content when it has been requested with the previous SPI transmission. In all other cases responds with STANDARD DIAGNOSIS.
 *\note See SPI Protocol (chapter 12.5 of TLD5542-1QV/TLD5501-2QV datasheets)
 *\param[in] cmd SPI (read register) frame to be transmitted
 *\param[in, out] response1 received SPI response to the first (cmd, read register) SPI request
 *\param[in, out] response2 received SPI response to the second (STANDARD DIAGNSIS, read register) SPI request
 *\return status, true when the activity has been successfully executed, false when the activity has not been executed
*/
bool PowerFlex_Sync_Read_Register(uint16 cmd, uint16 *response1, uint16 *response2){
  bool statusSpi = false;
      if(PowerFlex_Sync_Write_Register(cmd, response1) == true)
      {
        if((cmd == POWERFLEX_SPI_STANDARD_DIAGNOSIS_MSG) && ((*response1 & POWERFLEX_SPI_STANDARD_DIAGNOSIS_B15_MASK) == 0)){ /*requested register is STANDARD DIAG and received response to the first SPI frame is STANDARD DIAG*/
          *response2 = *response1;
          statusSpi = true;
        }else{                                                                      /*requested register is not STANDARD DIAG or received response to the first SPI command is not standard diagnosis*/
          if(PowerFlex_Sync_Write_Register(POWERFLEX_SPI_STANDARD_DIAGNOSIS_MSG, response2) == true){
            statusSpi = true;
          }
        }
      }
  return statusSpi; 
}

/** @brief Transmits one or two consecutive synchronous SPI frame to read an SPI register (user code is blocked during SPI transmissions). Received response is used to updated tld5542_1 global variable
 *
 *Generates and communicate a first SPI frame to read the requested register
 *generates and communicate a second SPI frame to get the register content requested with the first SPI frame 
 *both SPI communications use synchronous APIs offered by communication layer.
 *when transmissions are over updates tld5542_1 with both received SPI responses 
 *if requested register is STANDARD DIAG the second SPI frame may not be necessary since STARNDARD DIAG is the TLD5542-1QV default SPI response. 
 *Therefore it may be already received as response of the first SPI communication.
 *\param[in] regAdd	Unique identifier of the TLD5542-1QV SPI register address
 *\return TLD5542-1QV status, Quick evaluation of SPI communication
 *\note See SPI Protocol (chapter 12.5 of TLD5542-1QV datasheets)
*/
TLD5542_1_STATUS TLD5542_1_Sync_Read_Register(TLD5542_1_REG_ADDRESS regadd){
	TLD5542_1_STATUS status = TLD5542_1_UNKNOWN;
	uint16 response1, response2;
	uint16 cmd = TLD5542_1_set_SPI_command(regadd, TLD5542_1_READ_CMD);

  //ARduino: set SPI speed and mode for powerFlex, 
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
   
	if(PowerFlex_Sync_Read_Register(cmd, &response1, &response2) == true){
		TLD5542_1_get_SPI_response(response1, &status);
		TLD5542_1_get_SPI_response(response2, &status);
	}else{
		status = TLD5542_1_SPI_NOT_OK;
	}

  //Arduino
  SPI.endTransaction();// stop using SPI bus, allow other libraries to use the SPI bus

	return status;
}




/*****************************************************************************
										Local Functions definition   			                  
*****************************************************************************/

/** @brief Generates the appropriate SPI frame to write/read an SPI register
 *when rw = TLD5542_1_WRITE_CMD generates the SPI frame to updated the register using data stored into tld5542_1, when rw = TLD5542_1_READ_CMD generates the SPI frame to read the 
 *register. 
 *\param[in] regAdd	Unique identifier of the TLD5542-1QV SPI register address
 *\param[in] rw	Unique identifier of SPI frame type: TLD5542_1_READ_CMD or TLD5542_1_WRITE_CMD
 *\return cmd generated SPI frame to be transmitted
*/
static uint16 TLD5542_1_set_SPI_command(TLD5542_1_REG_ADDRESS regadd, TLD5542_1_CMD_TYPE rw){
	uint8 msb, lsb;
	uint16 cmd;
	
	switch(regadd){
		case TLD5542_1_STANDARD_DIAGNOSIS_ADDR:
				msb = TLD5542_1_STANDARDSIAGNOSIS_ADDR_MSB;
				lsb = TLD5542_1_STANDARDSIAGNOSIS_ADDR_LSB;
			break;
		case TLD5542_1_LEDCURRADIM_ADDR:
				TLD5542_1_set_LedcurradimCmd(&tld5542_1.LEDCURRADIM, &msb, &lsb, rw);
			break;
		case TLD5542_1_LEDCURRCAL_ADDR:
				TLD5542_1_set_LedcurrcalCmd(&tld5542_1.LEDCURRCAL, &msb, &lsb, rw);
			break;
		case TLD5542_1_SWTMOD_ADDR:
				TLD5542_1_set_SwtmodCmd(&tld5542_1.SWTMOD, &msb, &lsb, rw);
			break;
		case TLD5542_1_DVCCTRL_ADDR:
				TLD5542_1_set_DvcctrlCmd(&tld5542_1.DVCCTRL, &msb, &lsb, rw);
			break;
		case TLD5542_1_MFSSETUP1_ADDR:
				TLD5542_1_set_Mfssetup1Cmd(&tld5542_1.MFSSETUP1, &msb, &lsb, rw);
			break;
		case TLD5542_1_MFSSETUP2_ADDR:
				TLD5542_1_set_Mfssetup2Cmd(&tld5542_1.MFSSETUP2, &msb, &lsb, rw);			
			break;
		case TLD5542_1_CURRMON_ADDR:
				TLD5542_1_set_CurrmonCmd(&tld5542_1.CURRMON, &msb, &lsb, rw);
			break;
		case TLD5542_1_REGUSETMON_ADDR:
				TLD5542_1_set_RegusetmonCmd(&tld5542_1.REGUSETMON, &msb, &lsb, rw);
			break;
		case TLD5542_1_MUXCTRL_ADDR:
				TLD5542_1_set_MuxctrlCmd(&tld5542_1.MUXCTRL, &msb, &lsb, rw);
			break;
		default:
			/*no action*/
			break;
	}
	
	if((rw == TLD5542_1_READ_CMD) || (regadd == TLD5542_1_STANDARD_DIAGNOSIS_ADDR)){ /*read command*/
		cmd = ((((uint16) msb << POWERFLEX_SPI_ADDRESS_OFFSET_SHIFT) & POWERFLEX_SPI_ADDRESS_MASK) | (POWERFLEX_SPI_ACTION_READ)  | ((uint16) lsb));
	}else{												/*write command*/
		cmd = ((((uint16) msb << POWERFLEX_SPI_ADDRESS_OFFSET_SHIFT) & POWERFLEX_SPI_ADDRESS_MASK) | (POWERFLEX_SPI_ACTION_WRITE)  | ((uint16) lsb));
	}
	
	return cmd;
}

/** @brief Updates the gloabal vatiable tld5542_1 according with the received SPI response
 *when recived SPI repononse is STANDARD DIAGNOSIS offers a quick evaluation of TLD5542-1QV status. Reporting device power state, communication errors or
 *Fault conditions.
 *\param[in] regAdd	Unique identifier of the TLD5542-1QV SPI register address
 *\param[in,out] status, Quick evaluation of SPI communication
*/
static void TLD5542_1_get_SPI_response(uint16 response, TLD5542_1_STATUS *status){
	uint8 addr, data;
	
	if((response & POWERFLEX_SPI_STANDARD_DIAGNOSIS_B15_MASK) == 0){/*MSB will be low in case of standard diag response */
		TLD5542_1_get_StandardDiagnosisCmd(&tld5542_1.STANDARD_DIAGNOSIS, response, status);		
	}else{ 
		addr = (uint8) ((response & POWERFLEX_SPI_ADDRESS_MASK) >> POWERFLEX_SPI_ADDRESS_OFFSET_SHIFT);
		data = (uint8) ((response & POWERFLEX_SPI_DATA_MASK));
		switch(addr){
			case TLD5542_1_LEDCURRADIM_ADDR_MSB:
					TLD5542_1_get_LedcurradimCmd(&tld5542_1.LEDCURRADIM, data);
				break;
			case TLD5542_1_LEDCURRCAL_ADDR_MSB:
					TLD5542_1_get_LedcurrcalCmd(&tld5542_1.LEDCURRCAL, data);
				break;
			case TLD5542_1_SWTMOD_ADDR_MSB:
					TLD5542_1_get_SwtmodCmd(&tld5542_1.SWTMOD, data);
				break;
			case TLD5542_1_DVCCTRL_ADDR_MSB:
					TLD5542_1_get_DvcctrlCmd(&tld5542_1.DVCCTRL, data);
				break;
			case TLD5542_1_MFSSETUP1_ADDR_MSB:
					TLD5542_1_get_Mfssetup1Cmd(&tld5542_1.MFSSETUP1, data);
				break;
			case TLD5542_1_MFSSETUP2_ADDR_MSB:
					TLD5542_1_get_Mfssetup2Cmd(&tld5542_1.MFSSETUP2, data);
				break;
			case TLD5542_1_CURRMON_ADDR_MSB:
					TLD5542_1_get_CurrmonCmd(&tld5542_1.CURRMON, data);
				break;
			case TLD5542_1_REGUSETMON_ADDR_MSB:
					TLD5542_1_get_RegusetmonCmd(&tld5542_1.REGUSETMON, data);
				break;
			case TLD5542_1_MUXCTRL_ADDR_MSB:
					TLD5542_1_get_MuxctrlCmd(&tld5542_1.MUXCTRL, data);
				break;
			default:
				*status = TLD5542_1_SPI_NOT_OK;
			break;
		}
	}
}

/** @brief Update tld5542_1 STANDARD DIAGNOSIS field with SPI response (std) and evaluates the status of TLD5542-1QV.     
 *\param[in,out] stdreg STANDARD_DIAGNOSIS register
 *\param[in] std Standard Diagnosis SPI command
 *\param[in,out] status, Quick evaluation of SPI communication
 * \returns TLD5542-1QV status
*/
static void TLD5542_1_get_StandardDiagnosisCmd(TLD5542_1_STANDARD_DIAGNOSIS_REG_T *stdreg, uint16 std, TLD5542_1_STATUS *status){
	
	stdreg->TW 				  = (uint8) ((std & TLD5542_1_STD_TW_Msk) 					>> TLD5542_1_STD_TW_Pos);
	stdreg->TSD 				= (uint8) ((std & TLD5542_1_STD_TSD_Msk) 					>> TLD5542_1_STD_TSD_Pos);
	stdreg->SHRTLED 		= (uint8) ((std & TLD5542_1_STD_SHRTLED_Msk) 			>> TLD5542_1_STD_SHRTLED_Pos);
	stdreg->LEDCUR 		  = (uint8) ((std & TLD5542_1_STD_LEDCUR_Msk) 			>> TLD5542_1_STD_LEDCUR_Pos);
	stdreg->IVCCUVLO 	  = (uint8) ((std & TLD5542_1_STD_IVCCUVLO_Msk) 		>> TLD5542_1_STD_IVCCUVLO_Pos);
	stdreg->OUTOV 			= (uint8) ((std & TLD5542_1_STD_OUTOV_Msk) 				>> TLD5542_1_STD_OUTOV_Pos);
	stdreg->EOCAL 			= (uint8) ((std & TLD5542_1_STD_EOCAL_Msk) 				>> TLD5542_1_STD_EOCAL_Pos);
	stdreg->EOMFS 			= (uint8) ((std & TLD5542_1_STD_EOMFS_Msk) 				>> TLD5542_1_STD_EOMFS_Pos);
	stdreg->EOMON 			= (uint8) ((std & TLD5542_1_STD_EOMON_Msk)	 			>> TLD5542_1_STD_EOMON_Pos);
	stdreg->TER 				= (uint8) ((std & TLD5542_1_STD_TER_Msk) 					>> TLD5542_1_STD_TER_Pos);
	stdreg->STATE 			= (uint8) ((std & TLD5542_1_STD_STATE_Msk) 				>> TLD5542_1_STD_STATE_Pos);
	stdreg->UVLORST 		= (uint8) ((std & TLD5542_1_STD_UVLORST_Msk) 			>> TLD5542_1_STD_UVLORST_Pos);
	stdreg->SWRST_BSTUV = (uint8) ((std & TLD5542_1_STD_SWRST_BSTUV_Msk)  >> TLD5542_1_STD_SWRST_BSTUV_Pos);
	
	if(stdreg->STATE == TLD5542_1_STD_STATE_ACTIVE){
		*status = TLD5542_1_ACTIVE;
	}else if(stdreg->STATE == TLD5542_1_STD_STATE_IDLE){
		*status = TLD5542_1_IDLE;
	}else if(stdreg->STATE == TLD5542_1_STD_STATE_LIMPHOME){	
		*status = TLD5542_1_LIMPHOME;
	}else{
		*status = TLD5542_1_RESERVED;
	}	
	
	if((std & TLD5542_1_STD_RESET_Msk) != 0x0000){
		*status = TLD5542_1_RESET;
	}else if(((std & TLD5542_1_STD_ERROR_Msk) != 0x0000)){ 	
		*status = TLD5542_1_ERROR;
	}else{}
}


static void TLD5542_1_set_LedcurradimCmd(const TLD5542_1_LEDCURRADIM_REG_T* ledcurradimreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_LEDCURRADIM_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_LEDCURRADIM_ADDR_LSB;
	}else{
		*lsb = 0x00;	
		*lsb |= (uint8) ((ledcurradimreg->ADIMVAL << TLD5542_1_LEDCURRADIM_ADIMVAL_Pos) & TLD5542_1_LEDCURRADIM_ADIMVAL_Msk);
	}
}

static void TLD5542_1_get_LedcurradimCmd(TLD5542_1_LEDCURRADIM_REG_T* ledcurradimreg, uint8 data){
	ledcurradimreg->ADIMVAL = (uint8) ((data & TLD5542_1_LEDCURRADIM_ADIMVAL_Msk) >> TLD5542_1_LEDCURRADIM_ADIMVAL_Pos);
}

static void TLD5542_1_set_LedcurrcalCmd(const TLD5542_1_LEDCURRCAL_REG_T* ledcurrcalreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_LEDCURRCAL_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_LEDCURRCAL_ADDR_LSB;
	}else{
		*lsb = 0x00;	
		*lsb |= (uint8) (( ledcurrcalreg->CALIBVAL  << TLD5542_1_LEDCURRCAL_CALIBVAL_Pos)  & TLD5542_1_LEDCURRCAL_CALIBVAL_Msk);
		*lsb |= (uint8) (( ledcurrcalreg->SOCAL 		<< TLD5542_1_LEDCURRCAL_SOCAL_Pos) 		 & TLD5542_1_LEDCURRCAL_SOCAL_Msk);
		*lsb |= (uint8) (( ledcurrcalreg->DAC_OFF 	<< TLD5542_1_LEDCURRCAL_DAC_OFF_Pos)	 & TLD5542_1_LEDCURRCAL_DAC_OFF_Msk);
	}
}

static void TLD5542_1_get_LedcurrcalCmd(TLD5542_1_LEDCURRCAL_REG_T* ledcurrcalreg, uint8 data){
	ledcurrcalreg->CALIBVAL = (uint8) ((data & TLD5542_1_LEDCURRCAL_CALIBVAL_Msk) >> TLD5542_1_LEDCURRCAL_CALIBVAL_Pos);
	ledcurrcalreg->EOCAL 		= (uint8) ((data & TLD5542_1_LEDCURRCAL_EOCAL_Msk) 	  >> TLD5542_1_LEDCURRCAL_EOCAL_Pos);
	ledcurrcalreg->SOCAL 		= (uint8) ((data & TLD5542_1_LEDCURRCAL_SOCAL_Msk) 	  >> TLD5542_1_LEDCURRCAL_SOCAL_Pos);
	ledcurrcalreg->DAC_OFF	= (uint8) ((data & TLD5542_1_LEDCURRCAL_DAC_OFF_Msk)  >> TLD5542_1_LEDCURRCAL_DAC_OFF_Pos);
}

static void TLD5542_1_set_SwtmodCmd(const TLD5542_1_SWTMOD_REG_T* swtmodreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_SWTMOD_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_SWTMOD_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( swtmodreg->FDEVSPREAD 	<< TLD5542_1_SWTMOD_FDEVSPREAD_Pos) 	& TLD5542_1_SWTMOD_FDEVSPREAD_Msk);
		*lsb |= (uint8) (( swtmodreg->FMSPREAD 		<< TLD5542_1_SWTMOD_FMSPREAD_Pos) 		& TLD5542_1_SWTMOD_FMSPREAD_Msk);
		*lsb |= (uint8) (( swtmodreg->ENSPREAD 		<< TLD5542_1_SWTMOD_ENSPREAD_Pos) 		& TLD5542_1_SWTMOD_ENSPREAD_Msk);
		*lsb |= (uint8) (( swtmodreg->S2G_OFF 		<< TLD5542_1_SWTMOD_S2G_OFF_Pos) 			& TLD5542_1_SWTMOD_S2G_OFF_Msk);
		*lsb |= (uint8) (( swtmodreg->VFB_VIN_OFF << TLD5542_1_SWTMOD_VFB_VIN_OFF_Pos) 	& TLD5542_1_SWTMOD_VFB_VIN_OFF_Msk);
		*lsb |= (uint8) (( swtmodreg->CCM_4EVER 	<< TLD5542_1_SWTMOD_CCM4EVER_Pos) 		& TLD5542_1_SWTMOD_CCM4EVER_Msk);
		*lsb |= (uint8) (( swtmodreg->DCM_EN 			<< TLD5542_1_SWTMOD_DCM_EN_Pos) 			& TLD5542_1_SWTMOD_DCM_EN_Msk);		

	}
}

static void TLD5542_1_get_SwtmodCmd(TLD5542_1_SWTMOD_REG_T* swtmodreg, uint8 data){
	swtmodreg->FDEVSPREAD 	= (uint8) ((data & TLD5542_1_SWTMOD_FDEVSPREAD_Msk) 	>> TLD5542_1_SWTMOD_FDEVSPREAD_Pos);
	swtmodreg->FMSPREAD 		= (uint8) ((data & TLD5542_1_SWTMOD_FMSPREAD_Msk) 		>> TLD5542_1_SWTMOD_FMSPREAD_Pos);
	swtmodreg->ENSPREAD 		= (uint8) ((data & TLD5542_1_SWTMOD_ENSPREAD_Msk) 		>> TLD5542_1_SWTMOD_ENSPREAD_Pos);
	swtmodreg->S2G_OFF 			= (uint8) ((data & TLD5542_1_SWTMOD_S2G_OFF_Msk) 		  >> TLD5542_1_SWTMOD_S2G_OFF_Pos);
	swtmodreg->VFB_VIN_OFF 	= (uint8) ((data & TLD5542_1_SWTMOD_VFB_VIN_OFF_Msk)  >> TLD5542_1_SWTMOD_VFB_VIN_OFF_Pos);
	swtmodreg->CCM_4EVER 		= (uint8) ((data & TLD5542_1_SWTMOD_CCM4EVER_Msk) 		>> TLD5542_1_SWTMOD_CCM4EVER_Pos);
	swtmodreg->DCM_EN 			= (uint8) ((data & TLD5542_1_SWTMOD_DCM_EN_Msk) 			>> TLD5542_1_SWTMOD_DCM_EN_Pos);
}


static void TLD5542_1_set_DvcctrlCmd(const TLD5542_1_DVCCTRL_REG_T* dvcctrlreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_DVCCTRL_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_DVCCTRL_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( dvcctrlreg->IDLE 		<< TLD5542_1_DVCCTRL_IDLE_Pos) 		& TLD5542_1_DVCCTRL_IDLE_Msk);
		*lsb |= (uint8) (( dvcctrlreg->SWRST 		<< TLD5542_1_DVCCTRL_SWRST_Pos) 	& TLD5542_1_DVCCTRL_SWRST_Msk);
		*lsb |= (uint8) (( dvcctrlreg->CLRLAT 	<< TLD5542_1_DVCCTRL_CLRLAT_Pos) 	& TLD5542_1_DVCCTRL_CLRLAT_Msk);
		*lsb |= (uint8) (( dvcctrlreg->ENCAL 		<< TLD5542_1_DVCCTRL_ENCAL_Pos) 	& TLD5542_1_DVCCTRL_ENCAL_Msk);
		*lsb |= (uint8) (( dvcctrlreg->SLOPE 		<< TLD5542_1_DVCCTRL_SLOPE_Pos) 	& TLD5542_1_DVCCTRL_SLOPE_Msk);
		*lsb |= (uint8) (( dvcctrlreg->EA_GM 		<< TLD5542_1_DVCCTRL_EA_GM_Pos) 	& TLD5542_1_DVCCTRL_EA_GM_Msk);
		*lsb |= (uint8) (( dvcctrlreg->EA_IOUT	<< TLD5542_1_DVCCTRL_EA_IOUT_Pos)	& TLD5542_1_DVCCTRL_EA_IOUT_Msk);
	}
}

static void TLD5542_1_get_DvcctrlCmd(TLD5542_1_DVCCTRL_REG_T* dvcctrlreg, uint8 data){
	dvcctrlreg->IDLE 		= (uint8) ((data & TLD5542_1_DVCCTRL_IDLE_Msk) 		>> TLD5542_1_DVCCTRL_IDLE_Pos);
	dvcctrlreg->SWRST 	= (uint8) ((data & TLD5542_1_DVCCTRL_SWRST_Msk) 	>> TLD5542_1_DVCCTRL_SWRST_Pos);
	dvcctrlreg->CLRLAT 	= (uint8) ((data & TLD5542_1_DVCCTRL_CLRLAT_Msk) 	>> TLD5542_1_DVCCTRL_CLRLAT_Pos);
	dvcctrlreg->ENCAL 	= (uint8) ((data & TLD5542_1_DVCCTRL_ENCAL_Msk) 	>> TLD5542_1_DVCCTRL_ENCAL_Pos);
	dvcctrlreg->SLOPE 	= (uint8) ((data & TLD5542_1_DVCCTRL_SLOPE_Msk) 	>> TLD5542_1_DVCCTRL_SLOPE_Pos);
	dvcctrlreg->EA_GM 	= (uint8) ((data & TLD5542_1_DVCCTRL_EA_GM_Msk) 	>> TLD5542_1_DVCCTRL_EA_GM_Pos);
	dvcctrlreg->EA_IOUT = (uint8) ((data & TLD5542_1_DVCCTRL_EA_IOUT_Msk)	>> TLD5542_1_DVCCTRL_EA_IOUT_Pos);	
}

static void TLD5542_1_set_Mfssetup1Cmd(const TLD5542_1_MFSSETUP1_REG_T* mfssetup1reg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_MFSSETUP1_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_MFSSETUP1_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( mfssetup1reg->LEDCHAIN 			<< TLD5542_1_MFSSETUP1_LEDCHAIN_Pos) 			& TLD5542_1_MFSSETUP1_LEDCHAIN_Msk);
		*lsb |= (uint8) (( mfssetup1reg->SOMFS 				  << TLD5542_1_MFSSETUP1_SOMFS_Pos) 				& TLD5542_1_MFSSETUP1_SOMFS_Msk);
		*lsb |= (uint8) (( mfssetup1reg->ILIM_HALF_MFS	<< TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Pos)	& TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Msk);
		*lsb |= (uint8) (( mfssetup1reg->EA_IOUT_MFS		<< TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Pos) 	& TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Msk);
	}
}

static void TLD5542_1_get_Mfssetup1Cmd(TLD5542_1_MFSSETUP1_REG_T* mfssetup1reg, uint8 data){
	mfssetup1reg->LEDCHAIN 		 	= (uint8) ((data & TLD5542_1_MFSSETUP1_LEDCHAIN_Msk) 			>> TLD5542_1_MFSSETUP1_LEDCHAIN_Pos);
	mfssetup1reg->EOMFS 				= (uint8) ((data & TLD5542_1_MFSSETUP1_EOMFS_Msk) 				>> TLD5542_1_MFSSETUP1_EOMFS_Pos);
	mfssetup1reg->SOMFS 				= (uint8) ((data & TLD5542_1_MFSSETUP1_SOMFS_Msk) 				>> TLD5542_1_MFSSETUP1_SOMFS_Pos);
	mfssetup1reg->ILIM_HALF_MFS = (uint8) ((data & TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Msk)	>> TLD5542_1_MFSSETUP1_ILIM_HALF_MFS_Pos);
	mfssetup1reg->EA_IOUT_MFS	 	= (uint8) ((data & TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Msk) 	>> TLD5542_1_MFSSETUP1_EA_IOUT_MFS_Pos);
}


static void TLD5542_1_set_Mfssetup2Cmd(const TLD5542_1_MFSSETUP2_REG_T* mfssetup2reg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_MFSSETUP2_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_MFSSETUP2_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( mfssetup2reg->MFSDLY << TLD5542_1_MFSSETUP2_MFSDLY_Pos)	& TLD5542_1_MFSSETUP2_MFSDLY_Msk);
	}
}

static void TLD5542_1_get_Mfssetup2Cmd(TLD5542_1_MFSSETUP2_REG_T* mfssetup2reg, uint8 data){
	mfssetup2reg->MFSDLY = (uint8) ((data & TLD5542_1_MFSSETUP2_MFSDLY_Msk) >> TLD5542_1_MFSSETUP2_MFSDLY_Pos);
}


static void TLD5542_1_set_CurrmonCmd(const TLD5542_1_CURRMON_REG_T* currmonreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_CURRMON_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_CURRMON_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( currmonreg->SOMON << TLD5542_1_CURRMON_SOMON_Pos)	& TLD5542_1_CURRMON_SOMON_Msk);
	}
}

static void TLD5542_1_get_CurrmonCmd(TLD5542_1_CURRMON_REG_T* currmonreg, uint8 data){
	currmonreg->LEDCURR = (uint8) ((data & TLD5542_1_CURRMON_LEDCURR_Msk) >> TLD5542_1_CURRMON_LEDCURR_Pos);
	currmonreg->INCURR 	= (uint8) ((data & TLD5542_1_CURRMON_INCURR_Msk)  >> TLD5542_1_CURRMON_INCURR_Pos);
	currmonreg->EOMON 	= (uint8) ((data & TLD5542_1_CURRMON_EOMON_Msk) 	>> TLD5542_1_CURRMON_EOMON_Pos);
	currmonreg->SOMON 	= (uint8) ((data & TLD5542_1_CURRMON_SOMON_Msk) 	>> TLD5542_1_CURRMON_SOMON_Pos);
}

static void TLD5542_1_set_RegusetmonCmd(const TLD5542_1_REGUSETMON_REG_T* regusetmonreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_REGUSETMON_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_REGUSETMON_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( regusetmonreg->BB_BST_CMP << TLD5542_1_REGUSETMON_BB_BST_CMP_Pos)	& TLD5542_1_REGUSETMON_BB_BST_CMP_Msk);
	}
}

static void TLD5542_1_get_RegusetmonCmd(TLD5542_1_REGUSETMON_REG_T* regusetmonreg, uint8 data){
		regusetmonreg->BB_BST_CMP = (uint8) ((data & TLD5542_1_REGUSETMON_BB_BST_CMP_Msk) >> TLD5542_1_REGUSETMON_BB_BST_CMP_Pos);
		regusetmonreg->REGUMODFB  = (uint8) ((data & TLD5542_1_REGUSETMON_REGUMODFB_Msk)  >> TLD5542_1_REGUSETMON_REGUMODFB_Pos);
}


static void TLD5542_1_set_MuxctrlCmd(const TLD5542_1_MUXCTRL_REG_T* muxctrlreg, uint8 *msb, uint8 *lsb, TLD5542_1_CMD_TYPE rw){
	*msb = TLD5542_1_MUXCTRL_ADDR_MSB;
	
	if(rw == TLD5542_1_READ_CMD){
		*lsb = TLD5542_1_MUXCTRL_ADDR_LSB;
	}else{
		*lsb = 0x00;
		*lsb |= (uint8) (( muxctrlreg->SO_MUX_SEL << TLD5542_1_MUXCTRL_SO_MUX_SEL_Pos) & TLD5542_1_MUXCTRL_SO_MUX_SEL_Msk);
		*lsb |= (uint8) (( muxctrlreg->AMUX_EN 		<< TLD5542_1_MUXCTRL_AMUX_EN_Pos) 	 & TLD5542_1_MUXCTRL_AMUX_EN_Msk);
		*lsb |= (uint8) (( muxctrlreg->MFS_REF 		<< TLD5542_1_MUXCTRL_MFS_REF_Pos) 	 & TLD5542_1_MUXCTRL_MFS_REF_Msk);
	}
}

static void TLD5542_1_get_MuxctrlCmd(TLD5542_1_MUXCTRL_REG_T* muxctrlreg, uint8 data){
	muxctrlreg->SO_MUX_SEL = (uint8) ((data & TLD5542_1_MUXCTRL_SO_MUX_SEL_Msk) >> TLD5542_1_MUXCTRL_SO_MUX_SEL_Pos);
	muxctrlreg->AMUX_EN 	 = (uint8) ((data & TLD5542_1_MUXCTRL_AMUX_EN_Msk) 	 	>> TLD5542_1_MUXCTRL_AMUX_EN_Pos);
	muxctrlreg->MFS_REF 	 = (uint8) ((data & TLD5542_1_MUXCTRL_MFS_REF_Msk) 	 	>> TLD5542_1_MUXCTRL_MFS_REF_Pos);
}
