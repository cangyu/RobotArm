#include <includes.h>

OS_FLAG_GRP  *ServoModify;
extern struct Servo s[ROBOTIC_ARM_NUM];

/* Task Stacks */
static OS_STK App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK App_Task_ServoUpdate_Stk[APP_CFG_TASK_SERVO_UPDATE_STK_SIZE];
static OS_STK App_Task_ServoMove_Stk[ROBOTIC_ARM_NUM][APP_CFG_TASK_SERVO_MOVE_STK_SIZE];

/* Local Function Prototypes */
static void App_TaskStart(void *p_arg);
static void App_EventCreate(void);
static void App_TaskCreate(void);
static void App_Task_ServoUpdate(void *p_arg);
static void App_Task_ServoMove(void *p_arg);

/**
 * @brief  Entrance of the program.
 * @param  None.
 * @ret    Never return.
 */
int main(void)
{
  	CPU_INT08U  err;

  	/* Disable all interrupts */
  	CPU_IntDis();
  
	/* Initialize uCOS */
	OSInit();

	/* Create the start task */
	err=OSTaskCreateExt((void (*)(void *)) App_TaskStart,
						(void           *) 0,
						(OS_STK         *)&App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
						(INT8U           ) APP_CFG_TASK_START_PRIO,
						(INT16U          ) APP_CFG_TASK_START_PRIO,
						(OS_STK         *)&App_TaskStartStk[0],
						(INT32U          ) APP_CFG_TASK_START_STK_SIZE,
						(void           *) 0,
						(INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	assert(err==OS_ERR_NONE);

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_CFG_TASK_START_PRIO, "Start", &err);
	assert(err==OS_ERR_NONE);
#endif
	
    /* Start uCOS */
	OSStart();
	
	return 0;
}

/**
 * @brief  Initialize the ticker and do some statistic work.
 * @param  p_arg  Pointer to argument, not used here.
 * @ret    None.
 */
static void App_TaskStart(void *p_arg)
{	
	(void)p_arg;
	
	/* Initialize the board and Servos*/	 
	BSP_Init();
	Servo_InitMove();
	
	/* Initialize the SysTick */
	OS_CPU_SysTickInit(72000000);
	
#if (OS_TASK_STAT_EN > 0)
    OSStatInit();
#endif
	
#if (APP_CFG_PROBE_COM_EN == DEF_ENABLED) || (APP_CFG_PROBE_OS_PLUGIN_EN == DEF_ENABLED)
    App_ProbeInit(); //uC/Probe  utility. 
#endif
	
	/* Create application events */
	App_EventCreate(); 
	 
	/* Create application tasks */
    App_TaskCreate();            
	
	while(DEF_TRUE)
	{
	  	LED0_TOGGLE;
	  	OSTimeDlyHMSM(0, 0, 0, 500);
	}
}

/**
 * @brief  Create events like Semaphore, mutex, mailbox and so on.
 * @param  None.
 * @ret    None.
 */
static void App_EventCreate (void)
{
  	CPU_INT08U err;
  	
	ServoModify=OSFlagCreate(SERVOS_USED,&err);
	assert(ServoModify && err==OS_ERR_NONE);
#if (OS_EVENT_NAME_EN > 0)
  	OSFlagNameSet(ServoModify, "ModifyServos", &err);
	assert(err==OS_ERR_NONE);
#endif
	
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
	  	if(!((1<<i)&SERVOS_USED))
			continue;
	  
		SemApply[i]=OSSemCreate(0);
		assert(SemApply[i]);
	#if (OS_EVENT_NAME_EN > 0)
		OSEventNameSet(SemApply[i], (INT8U*)sem_name[i], &err);
		assert(err==OS_ERR_NONE);
	#endif
	}
}

/**
 * @brief  Create tasks for the application.
 * @param  None.
 * @ret    None.
 */
static void App_TaskCreate(void)
{
    CPU_INT08U  err;
	
	/*
	err=OSTaskCreateExt((void (*)(void *)) App_Task_ServoUpdate,
						(void           *) 0,
						(OS_STK         *)&App_Task_ServoUpdate_Stk[APP_CFG_TASK_SERVO_UPDATE_STK_SIZE - 1],
						(INT8U           ) APP_CFG_TASK_SERVO_UPDATE_PRIO,
						(INT16U          ) APP_CFG_TASK_SERVO_UPDATE_PRIO,
						(OS_STK         *)&App_Task_ServoUpdate_Stk[0],
						(INT32U          ) APP_CFG_TASK_SERVO_UPDATE_STK_SIZE,
						(void           *) 0,
						(INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	assert(err==OS_ERR_NONE);

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_CFG_TASK_SERVO_UPDATE_PRIO, "ServoUpdate", &err);
	assert(err==OS_ERR_NONE);
#endif
	*/
	
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
	  	if(!((1<<i)&SERVOS_USED))
		  continue;
	  
		err=OSTaskCreateExt((void (*)(void *)) App_Task_ServoMove,
							(void           *) i,
							(OS_STK         *)&App_Task_ServoMove_Stk[i][APP_CFG_TASK_SERVO_MOVE_STK_SIZE - 1],
							(INT8U           ) APP_CFG_TASK_SERVO_MOVE_PRIO-i,
							(INT16U          ) APP_CFG_TASK_SERVO_MOVE_PRIO-i,
							(OS_STK         *)&App_Task_ServoMove_Stk[i][0],
							(INT32U          ) APP_CFG_TASK_SERVO_MOVE_STK_SIZE,
							(void           *) 0,
							(INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
		assert(err==OS_ERR_NONE);

	#if (OS_TASK_NAME_EN > 0)
		OSTaskNameSet(APP_CFG_TASK_SERVO_MOVE_PRIO-i, (INT8U*)servo_name[i], &err);
		assert(err==OS_ERR_NONE);
	#endif
	}
}

/**
 * @brief  Receive cmds and move the servos gradually.
 * @param  p_arg  Pointer to argument, not used here.
 * @ret    None.
 */
static void App_Task_ServoUpdate(void *p_arg)
{
    (void)p_arg;
	uint8_t err=0x00;

    while (DEF_TRUE) 
    {
	  	OSFlagPend(ServoModify, (OS_FLAGS)SERVOS_USED,OS_FLAG_WAIT_SET_ALL, 0,&err);
		assert(err==OS_ERR_NONE);
	  	Servo_Run();
    }
}

/**
 * @brief  Move specified servo to target angle.
 * @param  p_arg  Pointer to index.
 * @ret    None.
 */
static void App_Task_ServoMove(void *p_arg)
{
	while (DEF_TRUE)
	  	move((int)p_arg);
}
