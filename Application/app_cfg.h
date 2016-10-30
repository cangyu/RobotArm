#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

/* Start */
#define APP_CFG_TASK_START_STK_SIZE 128
#define APP_CFG_TASK_START_PRIO 60

/* ServoUpdate */
#define APP_CFG_TASK_SERVO_UPDATE_STK_SIZE 128
#define APP_CFG_TASK_SERVO_UPDATE_PRIO 59

/* ServoMove */
#define APP_CFG_TASK_SERVO_MOVE_STK_SIZE 128
#define APP_CFG_TASK_SERVO_MOVE_PRIO 58

/* Timer */
#define  OS_TASK_TMR_PRIO                   (OS_LOWEST_PRIO - 2)


/*
*********************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#if 0
#define  TRACE_LEVEL_OFF                       		0
#define  TRACE_LEVEL_INFO                      		1
#define  TRACE_LEVEL_DEBUG                     		2
#endif

#define  APP_TRACE_LEVEL                			TRACE_LEVEL_INFO
#define  APP_TRACE                      			BSP_Ser_Printf

#define  APP_TRACE_INFO(x)            ((APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_TRACE x) : (void)0)
#define  APP_TRACE_DEBUG(x)           ((APP_TRACE_LEVEL >= TRACE_LEVEL_DEBUG) ? (void)(APP_TRACE x) : (void)0)



#endif
