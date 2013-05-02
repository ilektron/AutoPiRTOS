/*
 *
*/

/* TODO:
 * 	Implement PWM and Capture (using GPIO interrupts) in order to capture and modify PWM signals
 * 	Implement some fancy filter to merge magnetometer and gyro
 * 	Implement some gain feature through one of the channels
*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "type.h"

#include "timer32.h"
#include "i2c.h"
#include "gpio.h"
#include "l3g4200d.h"
#include "integration.h"

#include "pid.h"
#include "fixedptc.h"

#include <stdio.h>

#define ABS(X)					((X)>(0)?(X):(-X))
#define CLAMP(X, MIN, MAX)		(((X) < (MIN)) ? (MIN) : (((X) > (MAX)) ? (MAX) : (X)))

/* Priorities at which the tasks are created. */
#define mainTASK_PRIORITY_STATUS_LEDS		( tskIDLE_PRIORITY + 1 )
#define mainTASK_PRIORITY_PWM				( tskIDLE_PRIORITY + 2 )
#define mainTASK_PRIORITY_CAP				( tskIDLE_PRIORITY + 4 )

/* The bit of port 0 that the LPCXpresso LPC13xx LED is connected. */
#define mainMODE_LED_BIT 						( 2 )
#define mainGAIN_LED_BIT 						( 10 )

/* The rate at which data is sent to the queue, specified in milliseconds. */
#define mainSTATUS_LED_DELAY_SHORT		( 150 / portTICK_RATE_MS )
#define mainSTATUS_LED_DELAY_LONG		( 1000 / portTICK_RATE_MS )
#define mainCONTROL_DELAY				( 50 / portTICK_RATE_MS )
#define mainPWM_FREQ					( TIMER_FREQ )
#define mainPWM_DELAY					( 1000 / mainPWM_FREQ / portTICK_RATE_MS )

#define mainPWM_ARM_DELAY				( 50 / portTICK_RATE_MS )
#define mainPWM_MIN_PULSE				800
#define mainPWM_MAX_PULSE				2200
#define mainPWM_SCALE					500.0f
#define mainPWM_CENTER					1500
#define mainPWM_THROTTLE_INIT			1000

#define mainPID_DT						((float)mainPWM_DELAY*(float)portTICK_RATE_MS / 1000.0f)

#define mainCAP_SWITCH_THRESHOLD		400
#define mainCAP_THROTTLE_THRESHOLD		1200

#define mainANGLE_SCALE				40


// Initial PID gains
#define PIDGAIN_ROLL_P		0.0f
#define PIDGAIN_PITCH_P	0.0f
#define PIDGAIN_YAW_P		0.0f

#define PIDGAIN_ROLL_I		0
#define PIDGAIN_PITCH_I	0
#define PIDGAIN_YAW_I		0

#define PIDGAIN_ROLL_D		0.0f
#define PIDGAIN_PITCH_D		0.0f
#define PIDGAIN_YAW_D		0.0f

#define PIDGAIN_ROLL_RATE_P	.200f
#define PIDGAIN_PITCH_RATE_P	.200f
#define PIDGAIN_YAW_RATE_P		.200f

#define PIDGAIN_ROLL_RATE_I	0
#define PIDGAIN_PITCH_RATE_I	0
#define PIDGAIN_YAW_RATE_I		0.000f

#define PIDGAIN_ROLL_RATE_D	0.01f
#define PIDGAIN_PITCH_RATE_D	0.01f
#define PIDGAIN_YAW_RATE_D		0.01f

#define pwmARM_LOW_COUNT	10
#define pwmARM_HIGH_COUNT	8

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

typedef enum
{
	PID_NONE = -1,
	PID_ROLL = 0,
	PID_PITCH,
	PID_YAW,
	PID_ROLL_RATE,
	PID_PITCH_RATE,
	PID_YAW_RATE,
	PID_LAST
}
PID_e;



// The different types of airframes
#define mainUSE_MIXED_MODES		0
#define mainMIX_AIRPLANE		0
#define mainMIX_ELEVON			1
#define mainMIX_TILTROTOR		2
#define mainMIX_QUADROTOR		3
#define mainMIX_TRICOPTER		4
#define mainMIX_AIRPLANE_ELEVON		5
#define mainSET_MIX_MODE		mainMIX_TILTROTOR


/*
 * The tasks
 */
static void prvGainLEDs( void *pvParameters );
static void prvModeLEDs( void *pvParameters );
static void prvPWMTask( void *pvParameters );
static void prvCAPTask( void *pvParameters );

static inline uint32_t pwm_check_if_valid(uint32_t pulse_width);
static inline uint32_t pwm_clip(uint32_t pulse_width);

uint32_t control_to_pwm(float control);
float pwm_to_control(uint32_t pwm_pulse);

#if mainUSE_MIXED_MODES
inline void pwm_calculate(int mix_type);
#else
inline void pwm_calculate(void);
#endif
inline void pwm_update(void);
inline void pwm_center(void);

/* The queue used by both tasks. */
static xQueueHandle xQueue = NULL;

// PID controller structures for Roll, Pitch, and Yaw
static pid_t pids[PID_LAST];

// Array that holds the pwm command output values
volatile uint32_t pwm_command[6];

#if mainUSE_MIXED_MODES
// Variable that keeps track of what type of air vehicle this is
volatile int mainMixType = mainMIX_TILTROTOR;
#endif

// Variable to determine which channel is being modified
volatile int mainGainChannel = PID_NONE;
volatile int mainGain = 0;
volatile int mainRange = 0;

// Variable to tell if the arms system is armed
volatile int mainArmed = 0;

/*-----------------------------------------------------------*/

int main(void)
{
	/* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

//	memset(gpio_pulse_width_sum, 0, sizeof(gpio_pulse_width_sum));

	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the accompanying application
		note. */
		createI2CTasks();
		xTaskCreate( prvModeLEDs, ( signed char * ) "ModeLEDs", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY_STATUS_LEDS, NULL );
		xTaskCreate( prvGainLEDs, ( signed char * ) "GainLEDs", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY_STATUS_LEDS, NULL );
		xTaskCreate( prvPWMTask, ( signed char * ) "PWM", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY_PWM, NULL );
		xTaskCreate( prvCAPTask, ( signed char * ) "CAP", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY_CAP, NULL );

		/* Start the tasks running. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

//static void prvQueueSendTask( void *pvParameters )
//{
//	portTickType xNextWakeTime;
//	const unsigned long ulValueToSend = 100UL;
//
//	/* Initialise xNextWakeTime - this only needs to be done once. */
//	xNextWakeTime = xTaskGetTickCount();
//
//	for( ;; )
//	{
//		/* Place this task in the blocked state until it is time to run again.
//		The block state is specified in ticks, the constant used converts ticks
//		to ms.  While in the blocked state this task will not consume any CPU
//		time. */
//		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );
//
//		/* Send to the queue - causing the queue receive task to flash its LED.
//		0 is used as the block time so the sending operation will not block -
//		it shouldn't need to block as the queue should always be empty at this
//		point in the code. */
//		xQueueSend( xQueue, &ulValueToSend, 0 );
//	}
//}
/*-----------------------------------------------------------*/


// TODO Change this to the PID task to process data from the I2C Task
//static void prvQueueReceiveTask( void *pvParameters )
//{
//unsigned long ulReceivedValue;
//
//	for( ;; )
//	{
//		/* Wait until something arrives in the queue - this task will block
//		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
//		FreeRTOSConfig.h. */
//		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
//
//		/*  To get here something must have been received from the queue, but
//		is it the expected value?  If it is, toggle the LED. */
//		if( ulReceivedValue == 100UL )
//		{
//			prvToggleLED();
//		}
//	}
//}
/*-----------------------------------------------------------*/

static void prvCAPTask( void *pvParameters )
{

	// Init GPIOs
	GPIOInit();

	/* use port0_1 as input event, interrupt test. */
	GPIOSetDir( PORT1, 5, 0 );
	/* port0_1, single trigger, active high. */
	GPIOSetInterrupt( PORT1, 5, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
	GPIOIntEnable( PORT1, 5 );

	/* use port0_1 as input event, interrupt test. */
	GPIOSetDir( PORT1, 6, 0 );
	/* port0_1, single trigger, active high. */
	GPIOSetInterrupt( PORT1, 6, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
	GPIOIntEnable( PORT1, 6 );

	/* use port0_1 as input event, interrupt test. */
//	GPIOSetDir( PORT1, 7, 0 );
	/* port0_1, single trigger, active high. */
//	GPIOSetInterrupt( PORT1, 7, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
//	GPIOIntEnable( PORT1, 7 );

	/* use port0_1 as input event, interrupt test. */
	GPIOSetDir( PORT1, 8, 0 );
	/* port0_1, single trigger, active high. */
	GPIOSetInterrupt( PORT1, 8, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
	GPIOIntEnable( PORT1, 8 );

	/* use port0_1 as input event, interrupt test. */
	GPIOSetDir( PORT1, 9, 0 );
	/* port0_1, single trigger, active high. */
	GPIOSetInterrupt( PORT1, 9, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
	GPIOIntEnable( PORT1, 9);

	// TODO: allow this to be PPM input
	/* use port0_1 as input event, interrupt test. */
	GPIOSetDir( PORT1, 10, 0 );
	/* port0_1, single trigger, active high. */
	GPIOSetInterrupt( PORT1, 10, GPIO_INT_EDGE_TRIGGERED, GPIO_INT_DOUBLE_EDGED, 0 );
	GPIOIntEnable( PORT1, 10);

	// TODO: Check to see if we have new values and whatnot
	for (;;)
	{



		// Should change this to flash out gain channel
		vTaskDelay(mainCONTROL_DELAY);
	}
}

static inline uint32_t pwm_check_if_valid(uint32_t pulse_width)
{
	return (pulse_width < mainPWM_MAX_PULSE && pulse_width > mainPWM_MIN_PULSE) || pulse_width==0;
}

static inline uint32_t pwm_clip(uint32_t pulse_width)
{
	if (pulse_width==0)
	{
		return 0;
	}
	else if (pulse_width < mainPWM_MIN_PULSE)
	{
		return mainPWM_MIN_PULSE;
	}
	else if (pulse_width > mainPWM_MAX_PULSE)
	{
		return mainPWM_MAX_PULSE;
	}
	return pulse_width;
}

// TODO Fix to clip PWM signal to usable ranges
uint32_t control_to_pwm(float control)
{
	return control*mainPWM_SCALE + mainPWM_CENTER;
}

float pwm_to_control(uint32_t pwm_pulse)
{
	if (pwm_pulse)
	{
		return ((int32_t)pwm_pulse-mainPWM_CENTER)/mainPWM_SCALE;
	}
	return 0;
}

inline void pwm_center(void)
{
	// TODO Change for all different mix types
#if	!mainUSE_MIXED_MODES
	pwm_command[0] = mainPWM_CENTER;
	pwm_command[1] = mainPWM_CENTER;
	pwm_command[2] = mainPWM_THROTTLE_INIT;
	pwm_command[3] = mainPWM_THROTTLE_INIT;
	pwm_command[4] = mainPWM_THROTTLE_INIT;
	pwm_command[5] = mainPWM_THROTTLE_INIT;
#endif
}

inline void pwm_update(void)
{
	LPC_TMR32B1->MR0 = TIME_INTERVAL - TIMER_COUNTS_FROM_uS(pwm_command[1]);
	LPC_TMR32B1->MR1 = TIME_INTERVAL - TIMER_COUNTS_FROM_uS(pwm_command[2]);
	LPC_TMR32B1->MR3 = TIME_INTERVAL - TIMER_COUNTS_FROM_uS(pwm_command[3]);
	LPC_TMR32B0->MR3 = TIME_INTERVAL - TIMER_COUNTS_FROM_uS(pwm_command[0]);
	LPC_TMR32B0->MR1 = TIME_INTERVAL - TIMER_COUNTS_FROM_uS(pwm_command[5]);
}

inline void init_pids(void)
{
	pid_init(&pids[PID_ROLL],	PIDGAIN_ROLL_P,		PIDGAIN_ROLL_I,		PIDGAIN_ROLL_D,		mainPID_DT, 0);
	pid_init(&pids[PID_PITCH],	PIDGAIN_PITCH_P,	PIDGAIN_PITCH_I,	PIDGAIN_PITCH_D,	mainPID_DT, 0);
	pid_init(&pids[PID_YAW],	PIDGAIN_YAW_P,		PIDGAIN_YAW_I,		PIDGAIN_YAW_D,		mainPID_DT, 0);

	pid_init(&pids[PID_ROLL_RATE],	PIDGAIN_ROLL_RATE_P,	PIDGAIN_ROLL_RATE_I,	PIDGAIN_ROLL_RATE_D,	mainPID_DT, 0);
	pid_init(&pids[PID_PITCH_RATE],	PIDGAIN_PITCH_RATE_P,	PIDGAIN_PITCH_RATE_I,	PIDGAIN_PITCH_RATE_D,	mainPID_DT, 0);
	pid_init(&pids[PID_YAW_RATE],	PIDGAIN_YAW_RATE_P,		PIDGAIN_YAW_RATE_I,		PIDGAIN_YAW_RATE_D,		mainPID_DT, 0);
}


#if mainUSE_MIXED_MODES
inline void pwm_calculate(int mix_type)
{
	float roll = pids[PID_ROLL].control;
	float pitch = pids[PID_PITCH].control;
	float yaw = pids[PID_YAW].control;
	float throttle = pwm_to_control(gpio_pulse_width[CAP_THROTTLE]);

	switch (mix_type)
	{
		case mainMIX_AIRPLANE:
			pwm_command[0] = control_to_pwm(roll);
			pwm_command[1] = control_to_pwm(pitch);
			pwm_command[2] = gpio_pulse_width[CAP_THROTTLE];
			pwm_command[3] = control_to_pwm(yaw);
			break;
		case mainMIX_ELEVON:
			// Assume reversed elevon servos
			pwm_command[0] = control_to_pwm(pitch + roll);
			pwm_command[1] = control_to_pwm(-pitch + roll);
			pwm_command[2] = gpio_pulse_width[CAP_THROTTLE];
			pwm_command[3] = control_to_pwm(yaw);
			break;
		case mainMIX_TILTROTOR:
			// TODO  Change this based on angle of tiltrotor motors

			pwm_command[2] = control_to_pwm(throttle + roll);
			pwm_command[3] = control_to_pwm(throttle - roll);
			// Assume reversed yaw servos
			pwm_command[0] = control_to_pwm(yaw+pitch);
			pwm_command[1] = control_to_pwm(yaw-pitch);
			break;
		case mainMIX_QUADROTOR:
			pwm_command[0] = control_to_pwm(throttle + yaw + roll);
			pwm_command[1] = control_to_pwm(throttle - yaw + pitch);
			pwm_command[2] = control_to_pwm(throttle + yaw - roll);
			pwm_command[3] = control_to_pwm(throttle - yaw - pitch);
			break;
		case mainMIX_TRICOPTER:
			pwm_command[0] = control_to_pwm(throttle + roll);
			pwm_command[1] = control_to_pwm(throttle - roll);
			pwm_command[2] = control_to_pwm(throttle + pitch);
			pwm_command[3] = control_to_pwm(yaw);
			break;
	}
}
#elif mainSET_MIX_MODE==mainMIX_TILTROTOR
inline void pwm_calculate(void)
{
	// TODO  Change this based on angle of tiltrotor motors
	float roll = pids[PID_ROLL_RATE].control;
	float pitch = pids[PID_PITCH_RATE].control;
	float yaw = pids[PID_YAW_RATE].control;
	float throttle = pwm_to_control(gpio_pulse_width_sum[CAP_THROTTLE]);


	// Don't allow roll if throttle is below a certain point
	// This should force the motors off when the throttle is off
	if (gpio_pulse_width_sum[CAP_THROTTLE] < mainCAP_THROTTLE_THRESHOLD)
	{
		roll = 0;
	}

	if (gpio_pulse_width_sum[CAP_THROTTLE] > mainPWM_THROTTLE_INIT)
	{
		pwm_command[2] = control_to_pwm(throttle + roll);
		pwm_command[3] = control_to_pwm(throttle - roll);
	}
	else
	{
		pwm_command[2] = 0;
		pwm_command[3] = 0;
	}
	// Assume reversed yaw servos
	pwm_command[0] = control_to_pwm(yaw-pitch);
	pwm_command[1] = control_to_pwm(yaw+pitch);


	// Ensure nothing goes above half throttle
//	if (pwm_command[2] > 1500)
//		pwm_command[2] = 1500;
//
//	if (pwm_command[3] > 1500)
//		pwm_command[3] = 1500;
}
#elif mainSET_MIX_MODE==mainMIX_TRICOPTER
inline void pwm_calculate(void)
{
	// TODO  Change this based on angle of tiltrotor motors
	float roll = pids[PID_ROLL_RATE].control;
	float pitch = pids[PID_PITCH_RATE].control;
	float yaw = pids[PID_YAW_RATE].control;
	float throttle = pwm_to_control(gpio_pulse_width[CAP_THROTTLE]);


	// Don't allow roll if throttle is below a certain point
	// This should force the motors off when the throttle is off
	if (gpio_pulse_width[CAP_THROTTLE] < mainCAP_THROTTLE_THRESHOLD)
	{
		roll = 0;
		pitch = 0;
	}

	if (gpio_pulse_width[CAP_THROTTLE] > mainPWM_THROTTLE_INIT)
	{
		pwm_command[0] = control_to_pwm(throttle + roll - pitch/2);
		pwm_command[1] = control_to_pwm(throttle - roll - pitch/2);
		pwm_command[2] = control_to_pwm(throttle + pitch);
	}
	else
	{
		pwm_command[0] = 0;
		pwm_command[1] = 0;
		pwm_command[2] = 0;
	}

	pwm_command[3] = control_to_pwm(yaw);


	// Ensure nothing goes above half throttle
//	if (pwm_command[2] > 1500)
//		pwm_command[2] = 1500;
//
//	if (pwm_command[3] > 1500)
//		pwm_command[3] = 1500;
}
#elif mainSET_MIX_MODE==mainMIX_AIRPLANE
inline void pwm_calculate(void)
{
	float roll = pids[PID_ROLL_RATE].control;
	float pitch = pids[PID_PITCH_RATE].control;
	float yaw = pids[PID_YAW_RATE].control;
	float throttle = pwm_to_control(gpio_pulse_width_sum[CAP_THROTTLE]);

	pwm_command[0] = control_to_pwm(roll);
	pwm_command[1] = control_to_pwm(pitch);
	pwm_command[2] = gpio_pulse_width_sum[CAP_THROTTLE];
	pwm_command[3] = control_to_pwm(yaw);
}
#elif mainSET_MIX_MODE==mainMIX_AIRPLANE_ELEVON
inline void pwm_calculate(void)
{
	float roll = pids[PID_ROLL_RATE].control;
	float pitch = -pids[PID_PITCH_RATE].control;
	float yaw = pids[PID_YAW_RATE].control;
//	float throttle = pwm_to_control(gpio_pulse_width_sum[CAP_THROTTLE]);

	// Looks backwards, but the servos are mirrored
	float right_control = CLAMP(roll - pitch, -1.0f, 1.0f);
	float left_control = CLAMP(roll + pitch, -1.0f, 1.0f);

	pwm_command[0] = control_to_pwm(right_control);
	pwm_command[1] = control_to_pwm(left_control);
	pwm_command[2] = gpio_pulse_width_sum[CAP_THROTTLE];
//	pwm_command[2] = mainPWM_THROTTLE_INIT;
	pwm_command[3] = control_to_pwm(yaw);
}

#endif

void updateGains(void)
{
	// TODO Could have channel 5 be P and channel 6 be D or something
//	int32_t last_switch_pulse = 0;
	float pgain = 0;
	float igain = 0;

	// Check the gain channel and see if we need to update the gains
//	if (mainGainChannel != PID_NONE && pwm_check_if_valid(gpio_pulse_width_sum[CAP_GAIN]))

	// Check if the switch is on
//	if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_SWITCH]) && gpio_pulse_width_sum[CAP_SWITCH] > mainPWM_CENTER)
//	{
	if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_GAIN]) && pwm_check_if_valid(gpio_pulse_width_sum[CAP_SWITCH]))
	{
		pgain = 1.5f*(float)(gpio_pulse_width_sum[CAP_GAIN]-mainPWM_CENTER) / mainPWM_SCALE;
		igain = 1.0f*(float)(gpio_pulse_width_sum[CAP_SWITCH]-mainPWM_CENTER) / mainPWM_SCALE;
		pgain = pgain*pgain;
		igain = igain*igain*igain;

//				pid_set_gain(&(pids[mainGainChannel]), gain, 0, gain*0.3f);
		pid_set_gain(&(pids[PID_ROLL_RATE]), 0.5f*pgain, 0.5f*igain, 0);
		pid_set_gain(&(pids[PID_PITCH_RATE]), pgain, igain, 0);
		pid_set_gain(&(pids[PID_YAW_RATE]), pgain, igain, 0);

		if (igain < 0.0f)
		{
			pid_zero_integrator(&(pids[PID_ROLL_RATE]));
			pid_zero_integrator(&(pids[PID_PITCH_RATE]));
			pid_zero_integrator(&(pids[PID_YAW_RATE]));
		}
//			pid_set_gain(&(pids[PID_ROLL_RATE]), pgain, 0, 0);
//			pid_set_gain(&(pids[PID_PITCH_RATE]), pgain, 0, 0);
//			pid_set_gain(&(pids[PID_YAW_RATE]), pgain, 0, 0);
	}
//	}

//	if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_SWITCH]))
//	{
//		// Check to see if control channel tells us to switch which PID to change the gain
//		if (ABS(gpio_pulse_width_sum[CAP_SWITCH] - last_switch_pulse) > mainCAP_SWITCH_THRESHOLD)
//		{
//			mainGainChannel = (mainGainChannel + 1) % PID_LAST;
//			// Wiggle the servo
//			pwm_command[mainGainChannel] += 100;
//		}
//		last_switch_pulse = gpio_pulse_width_sum[CAP_SWITCH];
//	}
}

void wait_for_arm()
{
	int count = 0;
	int armed_low = 0;
	int armed_high = 0;

	int throttle_low_calibration = 0;
	int throttle_high_calibration = 0;

	mainArmed = 1;
	while (gpio_pulse_width_sum[CAP_THROTTLE] > 1200)
	{
		vTaskDelay(mainPWM_ARM_DELAY);
	}

	mainArmed = 2;
	count = 0;

	while (!armed_high)
	{
		if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_THROTTLE]) && gpio_pulse_width_sum[CAP_THROTTLE] > 1700)
		{
			count++;
			if (count > (pwmARM_HIGH_COUNT/2))
			{
				throttle_high_calibration += gpio_pulse_width_sum[CAP_THROTTLE];
			}
		}
		else
		{
			count = 0;
			throttle_high_calibration = 0;
		}

		// Wait for valid arm
		vTaskDelay(mainPWM_ARM_DELAY);

		armed_high = count >= pwmARM_HIGH_COUNT;
	}

	mainArmed = 3;

	while (!armed_low)
	{
		if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_THROTTLE]) && gpio_pulse_width_sum[CAP_THROTTLE] < 1200)
		{
			count++;
			if (count > (pwmARM_LOW_COUNT/2))
			{
				throttle_low_calibration += gpio_pulse_width_sum[CAP_THROTTLE];
			}
		}
		else
		{
			count = 0;
			throttle_low_calibration = 0;
		}

		// Wait for valid arm
		vTaskDelay(mainPWM_ARM_DELAY);

		armed_low = count >= pwmARM_LOW_COUNT;
	}

	mainArmed = 4;

}

static void prvPWMTask( void *pvParameters )
{
	portTickType xNextWakeTime;

//	int i;

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();


	// Center PWM Signals
	pwm_center();
	pwm_update();

	// Set up the timers for PWM.  The timers will also be used by the
	// PWM capture module
	init_timer32(1, TIME_INTERVAL);
	enable_timer32(1);

	init_timer32(0, TIME_INTERVAL);
	enable_timer32(0);

	// Initialize the roll pitch and yaw PIDs
	init_pids();

	// Wait awhile to zero the gyros
	vTaskDelay(2000/portTICK_RATE_MS);

	// Delay until armed
	wait_for_arm();

	// Zero out the gyros
	gyros_zero();

	for (;;)
	{
		// This is set up to work stock on a Turnigy 9x.  Just reverse channels to match on different systems
		// Uncomment for rate autopilot
//		pids[PID_ROLL].desired_val = -GYRO_SCALE*pwm_to_control(gpio_pulse_width_sum[CAP_ROLL]);
//		pids[PID_PITCH].desired_val = GYRO_SCALE*pwm_to_control(gpio_pulse_width_sum[CAP_PITCH]);
//		pid_input(&pids[PID_ROLL], GYRO_SCALE*(float)(gyro_y)/(float)(0x7FFF));
//		pid_input(&pids[PID_PITCH], GYRO_SCALE*(float)(gyro_x)/(float)(0x7FFF));
//		pids[PID_YAW].desired_val = -GYRO_SCALE*pwm_to_control(gpio_pulse_width_sum[CAP_YAW]);

		if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_YAW]))
		{
//			pids[PID_YAW].desired_val = pwm_to_control(gpio_pulse_width_sum[CAP_YAW]);
			pids[PID_YAW_RATE].desired_val = pwm_to_control(gpio_pulse_width_sum[CAP_YAW])*mainANGLE_SCALE/80.0f;
		}

		if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_ROLL]))
		{
			pids[PID_ROLL_RATE].desired_val = -pwm_to_control(gpio_pulse_width_sum[CAP_ROLL])*mainANGLE_SCALE/80.0f;
		}

		if (pwm_check_if_valid(gpio_pulse_width_sum[CAP_PITCH]))
		{
			pids[PID_PITCH_RATE].desired_val = pwm_to_control(gpio_pulse_width_sum[CAP_PITCH])*mainANGLE_SCALE/80.0f;
		}

		// Roll rate PIDs
//		pid_input(&pids[PID_ROLL], gyro_x_pos0/180.0f);
//		pid_input(&pids[PID_PITCH], gyro_y_pos0/180.0f);
//		pid_input(&pids[PID_YAW], gyro_z_pos0/180.0f);
		//		pid_input(&pids[PID_YAW], (float)(gyro_z0)/(float)(0x7FFF));

		// A positive angle needs a negative angular rate to correct
//		pids[PID_ROLL_RATE].desired_val = pids[PID_ROLL].control;
//		pids[PID_PITCH_RATE].desired_val = pids[PID_PITCH].control;
//		pids[PID_YAW_RATE].desired_val = pids[PID_YAW].control;

		pid_input(&pids[PID_ROLL_RATE], (float)(-gyro_y0)/(float)(0x7FFF));
//		pid_input(&pids[PID_ROLL_RATE], (float)(0)/(float)(0x7FFF));
		pid_input(&pids[PID_PITCH_RATE], (float)(-gyro_x0)/(float)(0x7FFF));
//		pid_input(&pids[PID_PITCH_RATE], (float)(0)/(float)(0x7FFF));
		pid_input(&pids[PID_YAW_RATE], (float)(-gyro_z0)/(float)(0x7FFF));
//		pid_input(&pids[PID_YAW_RATE], (float)(0)/(float)(0x7FFF));

		// Roll position PIDs
#if mainUSE_MIXED_MODES
		// Calculate the PWM controls based on mixing profile
		pwm_calculate(mainMixType);
#else
		pwm_calculate();
#endif
		// Calculate PID values and determine appropriate control signals
		pwm_update();

		updateGains();

		vTaskDelayUntil(&xNextWakeTime, mainPWM_DELAY);
	}

}

/**
 * This task blinks the Mode LED.  It should blink out the plane type
 */
static void prvModeLEDs( void *pvParameters )
{
	/* This function is only called from one task so does not have to be reentrant,
	so the static qualifier is acceptable. */
	static unsigned long ulLEDState = 1;
	portTickType xNextWakeTime;
	int i = 0;

	/* Set the Autopilot LED ports to output. */
	LPC_GPIO2->DIR |= ( 0x1 << mainMODE_LED_BIT );

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;)
	{
#if mainUSE_MIXED_MODES
		/* Turn the LED off if it was on, and on if it was off. */
		for (i=0; i<mainMixType + 1; i++)
#else
//		for (i=0; i<20; i++)
#endif
//		{
//			if (mainArmed)
//			{
//				ulLEDState = !ulLEDState;
//				LPC_GPIO2->MASKED_ACCESS[ ( 1 << mainMODE_LED_BIT) ] = ( ulLEDState << mainMODE_LED_BIT );
//
//				vTaskDelayUntil(&xNextWakeTime, mainSTATUS_LED_DELAY_SHORT);
//			}
//		}
		vTaskDelayUntil(&xNextWakeTime, mainSTATUS_LED_DELAY_LONG);
	}
}
/*-----------------------------------------------------------*/


/**
 * This task blinks the Mode LED.  It should blink out the plane type
 */
static void prvGainLEDs( void *pvParameters )
{
	/* This function is only called from one task so does not have to be reentrant,
	so the static qualifier is acceptable. */
	static unsigned long ulLEDState = 1;
	portTickType xNextWakeTime;
	int i = 0;

	/* Set the Autopilot LED ports to output. */
	LPC_GPIO2->DIR |= ( 0x1 << mainGAIN_LED_BIT );

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;)
	{
		/* Turn the LED off if it was on, and on if it was off. */
		for (i=0; i<(mainArmed)*2; i++)
		{
			ulLEDState = !ulLEDState;
			LPC_GPIO2->MASKED_ACCESS[ ( 1 << mainGAIN_LED_BIT) ] = ( ulLEDState << mainGAIN_LED_BIT );

			vTaskDelayUntil(&xNextWakeTime, mainSTATUS_LED_DELAY_SHORT);
		}
		vTaskDelayUntil(&xNextWakeTime, mainSTATUS_LED_DELAY_LONG);
	}
}
/*-----------------------------------------------------------*/

