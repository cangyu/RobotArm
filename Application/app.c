#include <includes.h>

/* Task Stacks */
static OS_STK App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK App_Task_Report_Stk[APP_CFG_TASK_REPORT_STK_SIZE];
static OS_STK App_Task_Servo_Stk[APP_CFG_TASK_SERVO_STK_SIZE];

/* Local Function Prototypes */
static void App_TaskStart(void *p_arg);
static void App_EventCreate(void);
static void App_TaskCreate(void);
static void App_Task_Report(void *p_arg);
static void App_Task_Servo(void *p_arg);

/**
 * @brief  Entrance of the program.
 * @param  None.
 * @ret    Never return.
 */
int main(void)
{
#if (OS_TASK_NAME_EN > 0)    
  CPU_INT08U  err;
#endif

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
	
	/* Initialize the board */	 
	BSP_Init();
	
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
	  	OSTimeDlyHMSM(0, 0, 2, 0);
	}
}

/**
 * @brief  Create events like Semaphore, mutex, mailbox and so on.
 * @param  None.
 * @ret    None.
 */
static void App_EventCreate (void)
{
}

/**
 * @brief  Create tasks for the application.
 * @param  None.
 * @ret    None.
 */
static void App_TaskCreate(void)
{
#if (OS_TASK_NAME_EN > 0)        
    CPU_INT08U      err;
#endif
    
    err=OSTaskCreateExt((void (*)(void *)) App_Task_Report,
						(void           *) 0,
						(OS_STK         *)&App_Task_Report_Stk[APP_CFG_TASK_REPORT_STK_SIZE - 1],
						(INT8U           ) APP_CFG_TASK_REPORT_PRIO,
						(INT16U          ) APP_CFG_TASK_REPORT_PRIO,
						(OS_STK         *)&App_Task_Report_Stk[0],
						(INT32U          ) APP_CFG_TASK_REPORT_STK_SIZE,
						(void           *) 0,
						(INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	assert(err==OS_ERR_NONE);

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_CFG_TASK_REPORT_PRIO, "Report", &err);
	assert(err==OS_ERR_NONE);
#endif
	
	err=OSTaskCreateExt((void (*)(void *)) App_Task_Servo,
						(void           *) 0,
						(OS_STK         *)&App_Task_Servo_Stk[APP_CFG_TASK_SERVO_STK_SIZE - 1],
						(INT8U           ) APP_CFG_TASK_SERVO_PRIO,
						(INT16U          ) APP_CFG_TASK_SERVO_PRIO,
						(OS_STK         *)&App_Task_Servo_Stk[0],
						(INT32U          ) APP_CFG_TASK_SERVO_STK_SIZE,
						(void           *) 0,
						(INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
	assert(err==OS_ERR_NONE);

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_CFG_TASK_SERVO_PRIO, "Servo", &err);
	assert(err==OS_ERR_NONE);
#endif
}

/**
 * @brief  Report the status, just for test.
 * @param  p_arg  Pointer to argument, not used here.
 * @ret    None.
 */
static void App_Task_Report(void *p_arg)
{
    (void)p_arg;

    while (DEF_TRUE) 
    {
        OSTimeDlyHMSM(0, 5, 0, 0);
    }
}

/**
 * @brief  Receive cmds and move the servos gradually.
 * @param  p_arg  Pointer to argument, not used here.
 * @ret    None.
 */
static void App_Task_Servo(void *p_arg)
{
    (void)p_arg;

    while (DEF_TRUE) 
    {
	  	Servo_Run();
    }
}


