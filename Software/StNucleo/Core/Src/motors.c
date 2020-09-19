#include "main.h"
#include "motors.h"
#include "powerstep01.h"
#include "stm32f4xx_hal.h"
#include "x_nucleo_ihmxx.h"

#include <stddef.h>

Motor_t motors[MOTOR_COUNT];
static volatile uint16_t gLastError;

/* Initialization parameters for current mode */
union powerstep01_Init_u initDeviceParameters = {
/* common parameters */
.cm.cp.cmVmSelection = POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
		5959, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
		5959, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
		RPM_TO_STEP_P_S(MOTOR_MAX_SPEED_RPM), // Maximum speed in step/s, range 15.25 to 15610 steps/s
		0, // Minimum speed in step/s, range 0 to 976.3 steps/s
		POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
		RPM_TO_STEP_P_S(FULL_STEP_SPEED_RPM), // Full step speed in step/s, range 7.63 to 15625 steps/s
		POWERSTEP01_BOOST_MODE_ON, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
		281.25, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
		STEP_MODE_1_4, // Step mode settings via enum motorStepMode_t
		POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t
		(POWERSTEP01_ALARM_EN_OVERCURRENT
				| POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN
				| POWERSTEP01_ALARM_EN_THERMAL_WARNING
				| POWERSTEP01_ALARM_EN_UVLO
				| POWERSTEP01_ALARM_EN_STALL_DETECTION
				| POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
		POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t
		POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
		POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
		POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t
		POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
		POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t
		/* current mode parameters */
		100, // Hold torque in mV, range from 7.8mV to 1000 mV
		200, // Running torque in mV, range from 7.8mV to 1000 mV
		100, // Acceleration torque in mV, range from 7.8mV to 1000 mV
		100, // Deceleration torque in mV, range from 7.8mV to 1000 mV
		POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t
		POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t
		3.0, // Minimum on-time in us, range 0.5us to 64us
		21.0, // Minimum off-time in us, range 0.5us to 64us
		POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
		POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
		POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
		POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
		POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
		POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
		POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
		POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
		POWERSTEP01_CONFIG_PRED_DISABLE, // Predictive current enabling , enum powerstep01_ConfigPredEn_t
		};

static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);
static void MyErrorHandler(uint16_t error);

void motors_Init() {
	//----- Init of the Powerstep01 library
	/* Set the Powerstep01 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01,
	MOTOR_COUNT);
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the Powerstep01 registers are set with the predefined values from file   */
	/* powerstep01_target_config.h, otherwise the registers are set using the   */
	/* powerstep01_Init_u relevant union of structure values.                   */
	/* The first call to BSP_MotorControl_Init initializes the first device     */
	/* whose Id is 0.                                                           */
	/* The nth call to BSP_MotorControl_Init initializes the nth device         */
	/* whose Id is n-1.                                                         */
	/* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
	/* device with the union declared in the the main.c file and comment the    */
	/* subsequent call having the NULL pointer                                  */
	for (int i = 0; i < MOTOR_COUNT; i++) {
		BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01,
				&initDeviceParameters);
		motors[i].state = Idle;
	}

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	/* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
	BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);

	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		BSP_MotorControl_CmdSoftHiZ(i);
	}
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void) {
	/* Get the value of the status register via the command GET_STATUS */
	for (int i = 0; i < MOTOR_COUNT; i++) {
		uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(i);
		/* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
		if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0
				|| (statusRegister & POWERSTEP01_STATUS_STALL_B) == 0) {
			//overcurrent detection

		}
	}

}

/**
 * @brief  This function is the User handler for the busy interrupt
 * @param  None
 * @retval None
 */
void MyBusyInterruptHandler(void) {

	if (BSP_MotorControl_CheckBusyHw()) {
		/* Busy pin is low, so at list one Powerstep01 chip is busy */
		/* To be customized (for example Switch on a LED) */
	} else {
		/* To be customized (for example Switch off a LED) */
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param[in] error Number of the error
 * @retval None
 */
void MyErrorHandler(uint16_t error) {
	/* Backup error number */
	gLastError = error;

	/* Infinite loop */
	while (1) {
	}
}
