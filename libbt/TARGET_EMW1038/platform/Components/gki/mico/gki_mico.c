/*
 * $ Copyright Broadcom Corporation $
 */
/******************************************************************************
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include "mico.h"

#include "gki_target.h"
#include "gki_int.h"

#if 0
#define gki_log(fmt, ...) custom_log("GKI", fmt, ##__VA_ARGS__)
#else
#define gki_log(fmt, ...)
#endif

/* Define the structure that holds the GKI variables */
#if GKI_DYNAMIC_MEMORY == FALSE
tGKI_CB gki_cb;
#endif

/* Wiced base priority for GKI_TASK_ID 0  */
#define GKI_MICO_BASE_PRIORITY     4u

static UINT8                disable_task_id = 0xFF;

/* Worker thread to manage timed event for MiCOKit led control */
// static mico_worker_thread_t gki_update_timer_worker_thread;
// static mico_timed_event_t   gki_update_timer_event;

static mico_timer_t         gki_update_timer;

/*******************************************************************************
 **
 ** Function         GKI_init
 **
 ** Description      This function is called once at startup to initialize
 **                  all the timer structures.
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_init(void)
{
    memset(&gki_cb, 0, sizeof(gki_cb));

    /* Initialize the GKI mutex used for GKI_enable and GKI_disable */
    mico_rtos_init_mutex(&gki_cb.os.GKI_mutex);

    gki_buffer_init();
    gki_timers_init();

    gki_cb.com.OSTicks = 0;
}

/*******************************************************************************
 **
 ** Function         GKI_get_os_tick_count
 **
 ** Description      This function is called to retrieve the native OS system tick.
 **
 ** Returns          Tick count of native OS.
 **
 *******************************************************************************/
UINT32 GKI_get_os_tick_count(void)
{
    return (gki_cb.com.OSTicks);
}

/*******************************************************************************
 **
 ** Function         GKI_create_task
 **
 ** Description      This function is called to create a new OSS task.
 **
 ** Parameters:      task_entry  - (input) pointer to the entry function of the task
 **                  task_id     - (input) Task id is mapped to priority
 **                  taskname    - (input) name given to the task
 **                  stack       - (input) pointer to the top of the stack (highest memory location)
 **                  stacksize   - (input) size of the stack allocated for the task
 **                  priority    - (input) thread priority (0-9)
 **
 ** Returns          GKI_SUCCESS if all OK, GKI_FAILURE if any problem
 **
 ** NOTE             This function take some parameters that may not be needed
 **                  by your particular OS. They are here for compatability
 **                  of the function prototype.
 **
 *******************************************************************************/
UINT8 GKI_create_task(TASKPTR task_entry, UINT8 task_id, INT8 *taskname, UINT16 *stack, UINT16 stacksize)
{
    UINT8 priority;
    OSStatus ret = 0;

    UNUSED_PARAMETER(stack);

    gki_log ("GKI_create_task func=0x%lx  id=%d  name=%s  stack=0x%lx  stackSize=%d",
             (UINT32)task_entry, task_id, taskname, (UINT32)stack, stacksize);

    if (task_id >= GKI_MAX_TASKS) {
        gki_log("Error! task ID > max task allowed");
        return (GKI_FAILURE);
    }

    /* Adjust priority based on task_id */
    priority = (UINT8)(task_id + GKI_MICO_BASE_PRIORITY);

    gki_cb.com.OSRdyTbl[task_id]  = TASK_READY;
    gki_cb.com.OSTName[task_id]   = taskname;
    gki_cb.com.OSWaitTmr[task_id] = 0;
    gki_cb.com.OSWaitEvt[task_id] = 0;

    ret = mico_rtos_init_mutex(&gki_cb.os.thread_evt_mutex[task_id]);
    if (ret != kNoErr) {
        gki_log("GKI_create_task thread_evt_mutex failed(%d), %s!", ret, taskname);
        return GKI_FAILURE;
    }

    ret = mico_rtos_init_queue(&gki_cb.os.thread_evt_queue[task_id],
                               NULL, THREAD_EVT_QUEUE_MSG_SIZE,
                               THREAD_EVT_QUEUE_NUM_MSG);
    if (ret != kNoErr) {
        gki_log("GKI_create_task thread_evt_queue failed(%d), %s!", ret, taskname);
        return GKI_FAILURE;
    }

    ret = mico_rtos_create_thread(&gki_cb.os.thread_id[task_id], priority, (const char *) taskname,
                                  (mico_thread_function_t) task_entry, stacksize, /*NULL*/0);
    if (ret != kNoErr) {
        gki_log("GKI_create_task thread_id failed(%d), %s!", ret, taskname);
        return GKI_FAILURE;
    }

    return (GKI_SUCCESS);
}

/*******************************************************************************
 **
 ** Function         GKI_shutdown
 **
 ** Description      shutdowns the GKI tasks/threads in from max task id to 0 and frees
 **                  pthread resources!
 **                  IMPORTANT: in case of join method, GKI_shutdown must be called outside
 **                  a GKI thread context!
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_shutdown(void)
{
    UINT8 task_id;

//    mico_rtos_deregister_timed_event(&gki_update_timer_event);
//    mico_rtos_delete_worker_thread(&gki_update_timer_worker_thread);
    mico_rtos_stop_timer(&gki_update_timer);
    mico_rtos_deinit_timer(&gki_update_timer);

    for (task_id = GKI_MAX_TASKS; task_id > 0; task_id--) {
        if (gki_cb.com.OSRdyTbl[task_id - 1] != TASK_DEAD) {

            /* paranoi settings, make sure that we do not execute any mailbox events */
            gki_cb.com.OSWaitEvt[task_id - 1] &= ~(TASK_MBOX_0_EVT_MASK
                                                   | TASK_MBOX_1_EVT_MASK
                                                   | TASK_MBOX_2_EVT_MASK
                                                   | TASK_MBOX_3_EVT_MASK);

            GKI_send_event(task_id - (UINT8)1, EVENT_MASK(GKI_SHUTDOWN_EVT));
        }
    }

    mico_thread_msleep(100);

    for (task_id = GKI_MAX_TASKS; task_id > 0; task_id--) {
        if (gki_cb.com.OSRdyTbl[task_id - 1] != TASK_DEAD) {
            GKI_exit_task(task_id - (UINT8)1);
        }
    }

    /* Destroy mutex and condition variable objects */
    mico_rtos_deinit_mutex(&gki_cb.os.GKI_mutex);
}

/*******************************************************************************
 **
 ** Function         gki_update_timer_cback
 **
 ** Description      Timer thread
 **
 ** Parameters:      id  - (input) timer ID
 **
 ** Returns          void
 **
 *********************************************************************************/
//OSStatus gki_update_timer_cback(void *arg)
//{
//    (void)arg;
//
//    GKI_timer_update(100);
//
//    return kNoErr;
//}

void gki_update_timer_cback(void *context)
{
    (void)context;
    GKI_timer_update(100);
}

/*******************************************************************************
 **
 ** Function         GKI_run
 **
 ** Description      This function runs a task
 **
 ** Parameters:      p_task_id  - (input) pointer to task id
 **
 ** Returns          void
 **
 ** NOTE             This function is only needed for operating systems where
 **                  starting a task is a 2-step process. Most OS's do it in
 **                  one step, If your OS does it in one step, this function
 **                  should be empty.
 *********************************************************************************/
void GKI_run(void *p_task_id)
{
    (void)p_task_id;

    mico_rtos_init_timer(&gki_update_timer, 100, gki_update_timer_cback, NULL);
    mico_rtos_start_timer(&gki_update_timer);

//    if (kNoErr != mico_rtos_create_worker_thread(&gki_update_timer_worker_thread, 4, 1024, 1)) {
//        gki_log("GKI_run: Create GKI Timer Worker thread failed\r\n");
//        return;
//    }
//
//    if (kNoErr != mico_rtos_register_timed_event(&gki_update_timer_event,
//                                                 &gki_update_timer_worker_thread,
//                                                  gki_update_timer_cback,
//                                                  100,
//                                                  NULL)) {
//
//        mico_rtos_delete_worker_thread(&gki_update_timer_worker_thread);
//        gki_log("GKI_run: Register Time Event failed\r\n");
//    }
}

/*******************************************************************************
 **
 ** Function         GKI_stop
 **
 ** Description      This function is called to stop
 **                  the tasks and timers when the system is being stopped
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to use it in your own implementation,
 **                  put specific code here.
 **
 *******************************************************************************/
void GKI_stop(void)
{
    UINT8 task_id;

    for (task_id = 0; task_id < GKI_MAX_TASKS; task_id++) {
        if (gki_cb.com.OSRdyTbl[task_id] != TASK_DEAD) {
            GKI_exit_task(task_id);
        }
    }
}

/*******************************************************************************
 **
 ** Function         GKI_wait
 **
 ** Description      This function is called by tasks to wait for a specific
 **                  event or set of events. The task may specify the duration
 **                  that it wants to wait for, or 0 if infinite.
 **
 ** Parameters:      flag -    (input) the event or set of events to wait for
 **                  timeout - (input) the duration that the task wants to wait
 **                                    for the specific events (in system ticks)
 **
 **
 ** Returns          the event mask of received events or zero if timeout
 **
 *******************************************************************************/
UINT16 GKI_wait(UINT16 flag, UINT32 timeout)
{
    OSStatus    err;
    UINT8       rtask;
    bool        check;
    UINT16      evt;
    UINT32      queueData = 0;

    rtask = GKI_get_taskid();
    if (rtask >= GKI_MAX_TASKS) {
        return 0;
    }

    /* Always lock around OSWaitEvt
     * Check the events that have to be processes by this task
     */
    mico_rtos_lock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);
    gki_cb.com.OSWaitForEvt[rtask] = flag;
    check = !(gki_cb.com.OSWaitEvt[rtask] & flag);
    mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);

    if (check) {
        /* The event has not yet occurred */
        timeout = (timeout ? timeout : MICO_WAIT_FOREVER);

        err = mico_rtos_pop_from_queue(&gki_cb.os.thread_evt_queue[rtask], (void *) &queueData, timeout);
        if (kNoErr != err) {
            gki_log("%s: pop queue failed: %d", __FUNCTION__, err);
            return 0;
        }
        mico_rtos_lock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);
        if (gki_cb.com.OSRdyTbl[rtask] == TASK_DEAD) {
            gki_cb.com.OSWaitEvt[rtask] = 0;
            mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);
            gki_log("GKI TASK_DEAD received. exit thread %d...", rtask);
            GKI_exit_task(rtask);
            return (EVENT_MASK(GKI_SHUTDOWN_EVT));
        }
    } else {
        /* The event has already occurred */
        err = mico_rtos_pop_from_queue(&gki_cb.os.thread_evt_queue[rtask], (void *) &queueData, 0);
        if (err != kNoErr) {
            gki_log("%s: pop queue failed: %d", __FUNCTION__, err);
            return 0;
        }
        mico_rtos_lock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);
    }

    /* Clear the wait for event mask */
    gki_cb.com.OSWaitForEvt[rtask] = 0;

    /* Return only those bits which user wants... */
    evt = gki_cb.com.OSWaitEvt[rtask] & flag;

    /* Clear only those bits which user wants... */
    gki_cb.com.OSWaitEvt[rtask] &= ~flag;

    /* Clear the flag the user wants and push the remaining flags back */
    queueData &= ~flag;
    queueData = queueData & 0x0000FFFF;
    if (queueData) {
        err = mico_rtos_push_to_queue(&gki_cb.os.thread_evt_queue[rtask], (void *) &queueData, MICO_NEVER_TIMEOUT);
        if (err != kNoErr) {
            gki_log("%s: push to queue failed: %d", __FUNCTION__, err);
        }
    }

    /* unlock thread_evt_mutex as mico_cond_wait() does auto lock mutex when cond is met */
    mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[rtask]);

    return (evt);
}

/*******************************************************************************
 **
 ** Function         GKI_delay
 **
 ** Description      This function is called by tasks to sleep unconditionally
 **                  for a specified amount of time. The duration is in milliseconds
 **
 ** Parameters:      timeout -    (input) the duration in milliseconds
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_delay(UINT32 timeout)
{
    UINT8 rtask = GKI_get_taskid();

    mico_rtos_delay_milliseconds(timeout);

    /* Check for a GKI task */
    if (rtask < GKI_MAX_TASKS && gki_cb.com.OSRdyTbl[rtask] == TASK_DEAD) {
        gki_log("GKI_delay call GKI_exit_task");
        GKI_exit_task(rtask);
    }
}

/*******************************************************************************
 **
 ** Function         GKI_send_event
 **
 ** Description      This function is called by tasks to send events to other
 **                  tasks. Tasks can also send events to themselves.
 **
 ** Parameters:      task_id -  (input) The id of the task to which the event has to
 **                  be sent
 **                  event   -  (input) The event that has to be sent
 **
 **
 ** Returns          GKI_SUCCESS if all OK, else GKI_FAILURE
 **
 *******************************************************************************/
UINT8 GKI_send_event(UINT8 task_id, UINT16 event)
{
    OSStatus ret;
    UINT32 queueData = 0;

    /* use efficient coding to avoid pipeline stalls */
    if (task_id < GKI_MAX_TASKS) {
        /* protect OSWaitEvt[task_id] from manipulation in GKI_wait() */
        mico_rtos_lock_mutex(&gki_cb.os.thread_evt_mutex[task_id]);

        if (gki_cb.com.OSRdyTbl[task_id] == TASK_DEAD) {
            mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[task_id]);
            gki_log("GKI_send_event task %i inactive", task_id);
            return (GKI_FAILURE);
        }

        /* Set the event bit */
        gki_cb.com.OSWaitEvt[task_id] |= event;

        if (mico_rtos_is_queue_empty(&gki_cb.os.thread_evt_queue[task_id])) {
            ret = mico_rtos_push_to_queue(&gki_cb.os.thread_evt_queue[task_id], (void *) &event, MICO_NEVER_TIMEOUT);
        } else {
            ret = mico_rtos_pop_from_queue(&gki_cb.os.thread_evt_queue[task_id], (void *) &queueData, 0);
            if (ret != kNoErr) {
                mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[task_id]);
                gki_log("mico_rtos_pop_from_queue failed task_id=0x%x ret=%d queueData=0x%x",
                        task_id,
                        ret,
                        event);
                return (GKI_FAILURE);
            }

            gki_cb.com.OSWaitEvt[task_id] |= (queueData & 0x0000FFFF);
            ret = mico_rtos_push_to_queue(&gki_cb.os.thread_evt_queue[task_id],
                                          (void *) &gki_cb.com.OSWaitEvt[task_id],
                                          MICO_NEVER_TIMEOUT);
        }
        mico_rtos_unlock_mutex(&gki_cb.os.thread_evt_mutex[task_id]);

        if (ret != kNoErr) {
            gki_log("mico_rtos_push_to_queue failed task_id=0x%x ret=%d queueData=0x%x",
                    task_id,
                    ret,
                    event);
        }

        return (GKI_SUCCESS);
    }
    return (GKI_FAILURE);
}

/*******************************************************************************
 **
 ** Function         GKI_isend_event
 **
 ** Description      This function is called from ISRs to send events to other
 **                  tasks. The only difference between this function and GKI_send_event
 **                  is that this function assumes interrupts are already disabled.
 **
 ** Parameters:      task_id -  (input) The destination task Id for the event.
 **                  event   -  (input) The event flag
 **
 ** Returns          GKI_SUCCESS if all OK, else GKI_FAILURE
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to use it in your own implementation,
 **                  put your code here, otherwise you can delete the entire
 **                  body of the function.
 **
 *******************************************************************************/
UINT8 GKI_isend_event(UINT8 task_id, UINT16 event)
{
    OSStatus ret;
    UINT32 queueData = 0;

    /* use efficient coding to avoid pipeline stalls */
    if (task_id < GKI_MAX_TASKS) {

        /* Set the event bit */
        gki_cb.com.OSWaitEvt[task_id] |= event;

        if (mico_rtos_is_queue_empty(&gki_cb.os.thread_evt_queue[task_id])) {
            ret = mico_rtos_push_to_queue(&gki_cb.os.thread_evt_queue[task_id], (void *) &event, MICO_NEVER_TIMEOUT);
        } else {
            if (mico_rtos_pop_from_queue(&gki_cb.os.thread_evt_queue[task_id], (void *) &queueData, 0) != kNoErr) {
                return (GKI_FAILURE);
            }

            gki_cb.com.OSWaitEvt[task_id] |= (queueData & 0x0000FFFF);
            ret = mico_rtos_push_to_queue(&gki_cb.os.thread_evt_queue[task_id],
                                          (void *) &gki_cb.com.OSWaitEvt[task_id],
                                          MICO_NEVER_TIMEOUT);
        }

        gki_log("GKI_send_event task_id=0x%x ret=0x%x queueData=0x%x",
                task_id, ret, event);

        return (GKI_SUCCESS);
    }

    return (GKI_FAILURE);
}

/*******************************************************************************
 **
 ** Function         GKI_get_taskid
 **
 ** Description      This function gets the currently running task ID.
 **
 ** Returns          task ID
 **
 ** NOTE             The Widcomm upper stack and profiles may run as a single task.
 **                  If you only have one GKI task, then you can hard-code this
 **                  function to return a '1'. Otherwise, you should have some
 **                  OS-specific method to determine the current task.
 **
 *******************************************************************************/
UINT8 GKI_get_taskid(void)
{
    UINT8 i;

    for (i = 0; i < GKI_MAX_TASKS; i++) {
        if (mico_rtos_is_current_thread(&gki_cb.os.thread_id[i])) {
            return i;
        }
    }
    return (UINT8)(-1);
}

/*******************************************************************************
 **
 ** Function         GKI_map_taskname
 **
 ** Description      This function gets the task name of the taskid passed as arg.
 **                  If GKI_MAX_TASKS is passed as arg the currently running task
 **                  name is returned
 **
 ** Parameters:      task_id -  (input) The id of the task whose name is being
 **                  sought. GKI_MAX_TASKS is passed to get the name of the
 **                  currently running task.
 **
 ** Returns          pointer to task name
 **
 ** NOTE             this function needs no customization
 **
 *******************************************************************************/
INT8 *GKI_map_taskname(UINT8 task_id)
{
    static const char *pszBad = "BAD";

    if (task_id < GKI_MAX_TASKS) {
        return (gki_cb.com.OSTName[task_id]);
    } else if (task_id == GKI_MAX_TASKS) {
        return (gki_cb.com.OSTName[GKI_get_taskid()]);
    } else {
        return (INT8 *) pszBad;
    }
}

/*******************************************************************************
 **
 ** Function         GKI_enable
 **
 ** Description      This function enables interrupts.
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_enable(void)
{
    OSStatus ret = mico_rtos_unlock_mutex(&gki_cb.os.GKI_mutex);
    if (ret != kNoErr) {
        gki_log("GKI_enable failed");
    }
}

/*******************************************************************************
 **
 ** Function         GKI_disable
 **
 ** Description      This function disables interrupts.
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_disable(void)
{
    OSStatus ret = kGeneralErr;
    UINT8    task_id;

    ret = mico_rtos_lock_mutex(&gki_cb.os.GKI_mutex);
    if (ret != kNoErr) {
        task_id = GKI_get_taskid();
        if (disable_task_id != task_id) {
            disable_task_id = task_id;
            gki_log("GKI_disable failed");
        }
    }
}

/*******************************************************************************
 **
 ** Function         GKI_exception
 **
 ** Description      This function throws an exception.
 **                  This is normally only called for a nonrecoverable error.
 **
 ** Parameters:      code    -  (input) The code for the error
 **                  msg     -  (input) The message that has to be logged
 **
 ** Returns          void
 **
 *******************************************************************************/
void GKI_exception(UINT16 code, char *msg)
{
    gki_log("GKI_exception(): Task State Table");

    for (UINT8 task_id = 0; task_id < GKI_MAX_TASKS; task_id++) {
        gki_log("TASK ID [%d] task name [%s] state [%d]",
                task_id,
                gki_cb.com.OSTName[task_id],
                gki_cb.com.OSRdyTbl[task_id]);
    }

    gki_log("GKI_exception %d %s", code, msg);
#if 0
    GKI_TRACE_ERROR_0( "********************************************************************");
    GKI_TRACE_ERROR_2( "* GKI_exception(): %d %s", code, msg);
    GKI_TRACE_ERROR_0( "********************************************************************");

#if (GKI_DEBUG == TRUE)
    GKI_disable();

    if (gki_cb.com.ExceptionCnt < GKI_MAX_EXCEPTION)
    {
        EXCEPTION_T *pExp;

        pExp = &gki_cb.com.Exception[gki_cb.com.ExceptionCnt++];
        pExp->type = code;
        pExp->taskid = GKI_get_taskid();
        strncpy((char *)pExp->msg, msg, GKI_MAX_EXCEPTION_MSGLEN - 1);
    }

    GKI_enable();
#endif

    GKI_TRACE_ERROR_2("GKI_exception %d %s done", code, msg);
#endif
}

/*******************************************************************************
 **
 ** Function         GKI_get_time_stamp
 **
 ** Description      This function formats the time into a user area
 **
 ** Parameters:      tbuf -  (output) the address to the memory containing the
 **                  formatted time
 **
 ** Returns          the address of the user area containing the formatted time
 **                  The format of the time is ????
 **
 ** NOTE             This function is only called by OBEX.
 **
 *******************************************************************************/
INT8 *GKI_get_time_stamp(INT8 *tbuf)
{
    UINT32 ms_time;
    UINT32 s_time;
    UINT32 m_time;
    UINT32 h_time;
    INT8 *p_out = tbuf;

    uint32_t time;
    time = mico_rtos_get_time();
    gki_cb.com.OSTicks = time;

    ms_time = GKI_TICKS_TO_MS(gki_cb.com.OSTicks);
    s_time = ms_time / 100; /* 100 Ticks per second */
    m_time = s_time / 60;
    h_time = m_time / 60;

    ms_time -= s_time * 100;
    s_time -= m_time * 60;
    m_time -= h_time * 60;

    *p_out++ = (INT8) ((h_time / 10) + '0');
    *p_out++ = (INT8) ((h_time % 10) + '0');
    *p_out++ = ':';
    *p_out++ = (INT8) ((m_time / 10) + '0');
    *p_out++ = (INT8) ((m_time % 10) + '0');
    *p_out++ = ':';
    *p_out++ = (INT8) ((s_time / 10) + '0');
    *p_out++ = (INT8) ((s_time % 10) + '0');
    *p_out++ = ':';
    *p_out++ = (INT8) ((ms_time / 10) + '0');
    *p_out++ = (INT8) ((ms_time % 10) + '0');
    *p_out++ = ':';
    *p_out = 0;

    return (tbuf);
}

/*******************************************************************************
 **
 ** Function         GKI_register_mempool
 **
 ** Description      This function registers a specific memory pool.
 **
 ** Parameters:      p_mem -  (input) pointer to the memory pool
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If your OS has different memory pools, you
 **                  can tell GKI the pool to use by calling this function.
 **
 *******************************************************************************/
void GKI_register_mempool(void *p_mem)
{
    gki_cb.com.p_user_mempool = p_mem;
}

/*******************************************************************************
 **
 ** Function         GKI_os_malloc
 **
 ** Description      This function allocates memory
 **
 ** Parameters:      size -  (input) The size of the memory that has to be
 **                  allocated
 **
 ** Returns          the address of the memory allocated, or NULL if failed
 **
 ** NOTE             This function is called by the Widcomm stack when
 **                  dynamic memory allocation is used. (see dyn_mem.h)
 **
 *******************************************************************************/
void *GKI_os_malloc(UINT32 size)
{
    return (malloc(size));
}

/*******************************************************************************
 **
 ** Function         GKI_os_free
 **
 ** Description      This function frees memory
 **
 ** Parameters:      size -  (input) The address of the memory that has to be
 **                  freed
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. It is only called from within GKI if dynamic
 **
 *******************************************************************************/
void GKI_os_free(void *p_mem)
{
    if (p_mem != NULL) {
        free(p_mem);
    }
}

/*******************************************************************************
 **
 ** Function         GKI_suspend_task()
 **
 ** Description      This function suspends the task specified in the argument.
 **
 ** Parameters:      task_id  - (input) the id of the task that has to suspended
 **
 ** Returns          GKI_SUCCESS if all OK, else GKI_FAILURE
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to implement task suspension capability,
 **                  put specific code here.
 **
 *******************************************************************************/
UINT8 GKI_suspend_task(UINT8 task_id)
{
    return (GKI_FAILURE);
}

/*******************************************************************************
 **
 ** Function         GKI_resume_task()
 **
 ** Description      This function resumes the task specified in the argument.
 **
 ** Parameters:      task_id  - (input) the id of the task that has to resumed
 **
 ** Returns          GKI_SUCCESS if all OK
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to implement task suspension capability,
 **                  put specific code here.
 **
 *******************************************************************************/
UINT8 GKI_resume_task(UINT8 task_id)
{
    return (GKI_FAILURE);
}

/*******************************************************************************
 **
 ** Function         GKI_exit_task
 **
 ** Description      This function is called to stop a GKI task.
 **
 ** Parameters:      task_id  - (input) the id of the task that has to be stopped
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to use it in your own implementation,
 **                  put specific code here to kill a task.
 **
 *******************************************************************************/
void GKI_exit_task(UINT8 task_id)
{
    GKI_disable();
    gki_cb.com.OSRdyTbl[task_id] = TASK_DEAD;
    /* Destroy mutex and condition variable objects */
    mico_rtos_deinit_mutex(&gki_cb.os.thread_evt_mutex[task_id]);
    mico_rtos_deinit_queue(&gki_cb.os.thread_evt_queue[task_id]);
    mico_rtos_delete_thread(&gki_cb.os.thread_id[task_id]);
    GKI_enable();
}

/*******************************************************************************
 **
 ** Function         GKI_sched_lock
 **
 ** Description      This function is called by tasks to disable scheduler
 **                  task context switching.
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to use it in your own implementation,
 **                  put code here to tell the OS to disable context switching.
 **
 *******************************************************************************/
void GKI_sched_lock(void)
{
    GKI_disable();
}

/*******************************************************************************
 **
 ** Function         GKI_sched_unlock
 **
 ** Description      This function is called by tasks to enable scheduler switching.
 **
 ** Returns          void
 **
 ** NOTE             This function is NOT called by the Widcomm stack and
 **                  profiles. If you want to use it in your own implementation,
 **                  put code here to tell the OS to re-enable context switching.
 **
 *******************************************************************************/
void GKI_sched_unlock(void)
{
    GKI_enable();
}

/*******************************************************************************
 **
 ** Function         GKI_interrupt_control
 **
 ** Description      This function enables or disables interrupts as specified
 **                  by the parameter new_posture.
 **
 ** Parameter        new_posture: This parameter specifies whether interrupts
 **                  are disabled or enabled. Legal values include TX_INT_DISABLE
 **                  and TX_INT_ENABLE.
 **
 ** Returns          previous posture : This service returns the previous interrupt
 **                  posture to the caller. This allows users of the service to
 **                  restore the previous posture after interrupts are disabled.
 **
 **
 *******************************************************************************/
//UINT8 GKI_interrupt_control( UINT8 new_posture )
//{
////    return mico_rtos_interrupt_control(new_posture);
//  return 0;
//    //return tx_interrupt_control((UINT)new_posture);
//}

/*******************************************************************************
 **
 ** Function         GKI_shiftdown
 **
 ** Description      shift memory down (to make space to insert a record)
 **
 *******************************************************************************/
void GKI_shiftdown(UINT8 *p_mem, UINT32 len, UINT32 shift_amount)
{
    register UINT8 *ps = p_mem + len - 1;
    register UINT8 *pd = ps + shift_amount;
    register UINT32 xx;

    for (xx = 0; xx < len; xx++) {
        *pd-- = *ps--;
    }
}

/*******************************************************************************
 **
 ** Function         GKI_shiftup
 **
 ** Description      shift memory up (to delete a record)
 **
 *******************************************************************************/
void GKI_shiftup(UINT8 *p_dest, UINT8 *p_src, UINT32 len)
{
    register UINT8 *ps = p_src;
    register UINT8 *pd = p_dest;
    register UINT32 xx;

    for (xx = 0; xx < len; xx++) {
        *pd++ = *ps++;
    }
}

/*******************************************************************************
 **
 ** Function         GKI_mutex_create
 **
 ** Description      Create mutex
 **
 *******************************************************************************/
BOOLEAN GKI_mutex_create(GKI_MUTEX *p_mutex)
{
    p_mutex->p_platform_mutex = malloc(sizeof(mico_mutex_t));
    return (mico_rtos_init_mutex((mico_mutex_t *) p_mutex->p_platform_mutex) == kNoErr);
}

/*******************************************************************************
 **
 ** Function         GKI_mutex_create
 **
 ** Description      Delete mutex
 **
 *******************************************************************************/
void GKI_mutex_delete(GKI_MUTEX *p_mutex)
{
    mico_rtos_deinit_mutex((mico_mutex_t *) p_mutex->p_platform_mutex);
    free(p_mutex->p_platform_mutex);
}

/*******************************************************************************
 **
 ** Function         GKI_mutex_lock
 **
 ** Description      Lock mutex
 **
 *******************************************************************************/
BOOLEAN GKI_mutex_lock(GKI_MUTEX *p_mutex)
{
    return (mico_rtos_lock_mutex((mico_mutex_t *) p_mutex->p_platform_mutex) == kNoErr);
}

/*******************************************************************************
 **
 ** Function         GKI_mutex_unlock
 **
 ** Description      Unlock mutex
 **
 *******************************************************************************/
BOOLEAN GKI_mutex_unlock(GKI_MUTEX *p_mutex)
{
    return (mico_rtos_unlock_mutex((mico_mutex_t *) p_mutex->p_platform_mutex) == kNoErr);
}
