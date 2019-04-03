/*
 * Copyright 2018 The MathWorks, Inc.
 *
 * File: MW_PX4_TaskControl.c     
 *
 * Abstract:
 *  This file contains the main application for Simulink that is launched 
 *  by PX4 stack at boot up. The main function spawns a new task and assigns 
 *  the main function in ert_main.c as callback.
 *
 */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"

#define DEBUG 0

extern "C" __EXPORT int px4_simulink_app_main(int argc, char *argv[]);

extern struct hrt_call BaseRate_HRT;      /* Base-rate HRT-callback is used to post base-rate semaphore */
extern int terminatingmodel;
extern int baserate_sem_copy; 			  /* used for checking HRT semaphore water-mark*/
static bool g_baseRateLife = false;		  /* global storage to contain the status of the px4_simulink_app */
static int px4_simulink_app_task;		  /* Handle of daemon thread */

/* Print the correct usage. */
void px4_app_usage(const char *reason)
{
  if (reason)
  PX4_INFO("px4_simulink_app : %s\n", reason);
  errx(1,"usage: px4_simulink_app {start|stop|status} [-p <additional params>]\n\n");
}

int px4_simulink_app_main(int argc, char *argv[])
{
	if (argc < 2) 
	{
		px4_app_usage("missing command");
	}

	if (!strcmp(argv[1], "start"))
	{
		if (g_baseRateLife == false)
		{
			/* Start the Simulink Tasks here */
#if DEBUG
			printf("px4_simulink_app : Starting the Simulink model\n");
			fflush(stdout);
#endif

			/* Reset semaphore */
			g_baseRateLife = true;
			terminatingmodel = 0; // reset global variable. For more info on the rational behind this, please see:
 						  		  // http://nuttx.org/doku.php?id=wiki:nxinternal:tasks-vs-processes#nuttx_flat-build_behavior

			px4_simulink_app_task = px4_task_spawn_cmd("px4_simulink_app_task",			/* Definition of px4_task_spawn_cmd : C:\px4\Firmware\src\platforms\nuttx\px4_layer\px4_nuttx_tasks.c */
														SCHED_DEFAULT,					/* For STM32 F4, NuttX has SCHED_DEFAULT as SCHED_RR */
														SCHED_PRIORITY_MAX - 15,		/* SCHED_PRIORITY_MAX: 255 */
														2048,
														px4_simulink_app_task_main,
														(char *const *)NULL);

		}
		else
		{
			warnx("px4_simulink_app : Simulink model is already running\n");
			fflush(stdout);
		}
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		if (g_baseRateLife == true)
		{
			rtmSetErrorStatus(MW_StringifyDefineFunction(MODEL, _M), "Module stopped by user");
			g_baseRateLife = false;
			sem_post(&baserateTaskSem);
#if DEBUG			
			PX4_INFO("px4_simulink_app : Exiting the Simulink model\n");
#endif
		}
		else
		{
			warnx("px4_simulink_app : No Simulink model is running\n");
		}

        fflush(stdout);
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
        if (g_baseRateLife)
        {
            PX4_INFO("px4_simulink_app : model is running\n");
        }
        else
        {
            PX4_INFO("px4_simulink_app : model is not started\n");
        }
		fflush(stdout);
        
        exit(0);
	}

	px4_app_usage("unrecognized command");
	exit(1);
}

void MW_PX4_Terminate()
{
    hrt_cancel(&BaseRate_HRT);
	g_baseRateLife = false; // set status of PX4 Simulink App to stop
    
#if defined(MW_HRT_CONSTRAINT) && (1 == MW_HRT_CONSTRAINT)
    if (baserate_sem_copy > MW_SEM_WATERMARK)
    {
        PX4_INFO("Hard real-time constraint violated, shutting down. Updating log file. \n");
        FILE* fp_taskover_run = NULL;
        char msg_watermark[100] = { 0 };
        char msg_timestamp[50] = { 0 };
        struct tm *sTm;
        time_t now = time(NULL);
        sTm = gmtime (&now);
        sprintf(msg_watermark,"Base-rate semaphore exceeded water-mark value %d. Model base-rate = %.3f sec.",MW_SEM_WATERMARK,(float)MW_BASERATE_PERIOD);
        strftime(msg_timestamp, sizeof(msg_timestamp),"%Y-%m-%d %H:%M:%S", sTm);
        fp_taskover_run = fopen("/fs/microsd/log/task_overrun_log.txt","a+");        
        fprintf(fp_taskover_run,"%s Timestamp=%s \n",msg_watermark,msg_timestamp);
        fclose(fp_taskover_run);  
        
    }  
    #endif
    
#if DEBUG    
    PX4_INFO("px4_simulink_app : Received command to end the Simulink task \n");
	fflush(stdout);
#endif	
}
