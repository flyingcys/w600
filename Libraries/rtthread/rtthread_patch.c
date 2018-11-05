#include "string.h"
#include <rtthread.h>
#include <stdint.h>
#include "wm_type_def.h"
#include "wm_ram_config.h"

/** TYPE definition of tls_os_task_t */
typedef void * 	tls_os_task_t;
/** TYPE definition of tls_os_timer_t */
typedef void      tls_os_timer_t;
/** TYPE definition of tls_os_sem_t */
typedef void	tls_os_sem_t;
/** TYPE definition of tls_os_queue_t */
typedef void	tls_os_queue_t;
/** TYPE definition of tls_os_mailbox_t */
typedef void	tls_os_mailbox_t;
/** TYPE definition of tls_os_mutex_t */
typedef void    tls_os_mutex_t;
/** TYPE definition of TLS_OS_TIMER_CALLBACK */
typedef  void (*TLS_OS_TIMER_CALLBACK)(void *ptmr, void *parg);

/** MACRO definition of TIMER ONE times */
#define TLS_OS_TIMER_OPT_ONE_SHORT    1u
/** MACRO definition of TIMER PERIOD */
#define TLS_OS_TIMER_OPT_PERIOD       2u

/** ENUMERATION definition of OS STATUS */
typedef enum tls_os_status {
    TLS_OS_SUCCESS = 0,
    TLS_OS_ERROR,
    TLS_OS_ERR_TIMEOUT,
} tls_os_status_t;

/*
*********************************************************************************************************
*                                     CREATE A TASK (Extended Version)
*
* Description: This function is used to have uC/OS-II manage the execution of a task.  Tasks can either
*              be created prior to the start of multitasking or by a running task.  A task cannot be
*              created by an ISR.
*
* Arguments  : task      is a pointer to the task'
*
*			name 	is the task's name
*
*			entry	is the task's entry function
*
*              param     is a pointer to an optional data area which can be used to pass parameters to
*                        the task when the task first executes.  Where the task is concerned it thinks
*                        it was invoked and passed the argument 'param' as follows:
*
*                            void Task (void *param)
*                            {
*                                for (;;) {
*                                    Task code;
*                                }
*                            }
*
*              stk_start      is a pointer to the task's bottom of stack.
*
*              stk_size  is the size of the stack in number of elements.  If OS_STK is set to u8,
*                        'stk_size' corresponds to the number of bytes available.  If OS_STK is set to
*                        INT16U, 'stk_size' contains the number of 16-bit entries available.  Finally, if
*                        OS_STK is set to INT32U, 'stk_size' contains the number of 32-bit entries
*                        available on the stack.
*
*              prio      is the task's priority.  A unique priority MUST be assigned to each task and the
*                        lower the number, the higher the priority.
*
*              flag       contains additional information about the behavior of the task.
*
* Returns    : TLS_OS_SUCCESS             if the function was successful.
*              TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_task_create(tls_os_task_t *task,
      const char* name,
      void (*entry)(void* param),
      void* param,
      u8 *stk_start,
      u32 stk_size,
      u32 prio,
      u32 flag)
{
    rt_err_t err;
    char task_name[RT_NAME_MAX];
    static uint16_t thread_cnt = 0;
    
    if (((u32)stk_start >= TASK_STACK_USING_MEM_UPPER_RANGE) 
		||(((u32)stk_start + stk_size) >= TASK_STACK_USING_MEM_UPPER_RANGE))
    {
    	rt_kprintf("\nCurrent Stack [0x%8x, 0x%8x) is NOT in VALID STACK range [0x20000000,0x20028000)\n", stk_start, stk_start + stk_size);
    	rt_kprintf("Please refer to APIs' manul and modify task stack position!!!\n");
    	return TLS_OS_ERROR;
    }

    struct rt_thread *thread = (struct rt_thread *)rt_malloc(sizeof(struct rt_thread));
    if(thread == RT_NULL)
        return TLS_OS_ERROR;

    memset(thread, 0, sizeof(struct rt_thread));

    memset(task_name, 0, sizeof(task_name));
    if(name != RT_NULL)
    {
        memcpy(task_name, name, strlen(name) > RT_NAME_MAX ? RT_NAME_MAX :  strlen(name));
    }
    else
    {
        snprintf(task_name, RT_NAME_MAX, "task-%02d", thread_cnt ++);
    }
    
    err = rt_thread_init(thread, task_name, entry, param, stk_start, stk_size, prio, 20);
    if(err == RT_EOK) {
        rt_thread_startup(thread);
        if(task != NULL)
            *task = thread;
        return TLS_OS_SUCCESS;
    }
    else
        return TLS_OS_ERROR;
}

/*
*********************************************************************************************************
*                                         GET CURRENT SYSTEM TIME
*
* Description: This function is used by your application to obtain the current value of the 32-bit
*              counter which keeps track of the number of clock ticks.
*
* Arguments  : none
*
* Returns    : The current value of OSTime
*********************************************************************************************************
*/
u32 tls_os_get_time(void)
{
    return rt_tick_get();
}

/**********************************************************************************************************
* Description: Disable interrupts by preserving the state of interrupts.
*
* Arguments  : none
*
* Returns    : cpu_sr
***********************************************************************************************************/
u32 tls_os_set_critical(void)
{
    rt_enter_critical();
    
    return 1;
}

/**********************************************************************************************************
* Description: Enable interrupts by preserving the state of interrupts.
*
* Arguments  : cpu_sr
*
* Returns    : none
***********************************************************************************************************/
void tls_os_release_critical(u32 cpu_sr)
{
    rt_exit_critical();
}

/*
************************************************************************************************************************
*                                                   CREATE A TIMER
*
* Description: This function is called by your application code to create a timer.
*
* Arguments  : timer    A pointer to an OS_TMR data structure.This is the 'handle' that your application
*                       will use to reference the timer created.
*
*               callback      Is a pointer to a callback function that will be called when the timer expires.  The
*                               callback function must be declared as follows:
*
*                               void MyCallback (OS_TMR *ptmr, void *p_arg);
*
*                callback_arg  Is an argument (a pointer) that is passed to the callback function when it is called.
*
*                period        The 'period' being repeated for the timer.
*                               If you specified 'OS_TMR_OPT_PERIODIC' as an option, when the timer expires, it will
*                               automatically restart with the same period.
*
*           repeat  if repeat
*
*               pname         Is a pointer to an ASCII string that is used to name the timer.  Names are useful for
*                               debugging.
*
*Returns    : TLS_OS_SUCCESS
*           TLS_OS_ERROR
************************************************************************************************************************
*/
struct __tls_os_timer
{
    rt_list_t list;
    
    rt_timer_t timer;
    TLS_OS_TIMER_CALLBACK callback;
    void *callback_arg;
};

static rt_list_t __tls_os_timer_head;

static void __tls_os_timer_cb(void *parameter)
{
    rt_list_t *node, *next;
    rt_timer_t timer = (rt_timer_t)parameter;
    struct __tls_os_timer *tls_os_timer = RT_NULL;
    
    if(!rt_list_isempty(&__tls_os_timer_head))
    {
        for(node = __tls_os_timer_head.next; node != &__tls_os_timer_head; node = next)
        {
            next = node->next;
            tls_os_timer = rt_list_entry(node, struct __tls_os_timer, list);
            if((tls_os_timer != RT_NULL) && (tls_os_timer->timer == timer))
            {
                break;
            }
        }

        tls_os_timer->callback(tls_os_timer->timer, tls_os_timer->callback_arg);
    }
}

tls_os_status_t tls_os_timer_create(tls_os_timer_t **timer,
      TLS_OS_TIMER_CALLBACK callback,
      void *callback_arg,
      u32 period,
      bool repeat,
      u8 *name)
{
    static rt_uint16_t __tim;
    char tim_name[RT_NAME_MAX];
    struct rt_timer * __rt_timer;
    rt_uint8_t flag;
    struct __tls_os_timer *tls_os_timer;

    if(name)
        rt_kprintf("name:%s\r\n", name);

    memset(tim_name, 0, sizeof(tim_name));
    if(name == RT_NULL)
        snprintf(tim_name, RT_NAME_MAX, "tim-%03d", __tim ++);
    else
        memcpy(tim_name, name, strlen(name) > RT_NAME_MAX ? RT_NAME_MAX : strlen(name));
    
//    rt_kprintf("%s %d name:%s period:%d\r\n", __FUNCTION__, __LINE__, tim_name, period);

    if(repeat == FALSE)
        flag = RT_TIMER_FLAG_ONE_SHOT;
    else
        flag = RT_TIMER_FLAG_PERIODIC;
    
    __rt_timer = (struct rt_timer *)rt_malloc(sizeof(struct rt_timer));
    if(__rt_timer == RT_NULL)
    {
        rt_kprintf("tls_os_timer malloc __rt_timer error...");
        return TLS_OS_ERROR;
    }
    memset(__rt_timer, 0, sizeof(struct rt_timer));

    tls_os_timer = (struct __tls_os_timer *)rt_malloc(sizeof(struct __tls_os_timer));
    if(tls_os_timer == RT_NULL)
    {
        rt_kprintf("tls_os_timer malloc __tls_os_timer error...");
        rt_free(__rt_timer);
        return TLS_OS_ERROR;
    }
    memset(tls_os_timer, 0, sizeof(struct __tls_os_timer));

    tls_os_timer->callback = callback;
    tls_os_timer->callback_arg = callback_arg;
    tls_os_timer->timer = __rt_timer;
    rt_list_init(&tls_os_timer->list);
    rt_list_insert_after(&__tls_os_timer_head, &tls_os_timer->list);

    rt_timer_init(__rt_timer, tim_name, __tls_os_timer_cb, __rt_timer, period, RT_TIMER_FLAG_SOFT_TIMER | flag);

    *timer = __rt_timer;
    
    return TLS_OS_SUCCESS;
}

/*
************************************************************************************************************************
*                                                   START A TIMER
*
* Description: This function is called by your application code to start a timer.
*
* Arguments  : timer          Is a pointer to an OS_TMR
*
************************************************************************************************************************
*/
void tls_os_timer_start(tls_os_timer_t *timer)
{
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, timer);
    rt_err_t err = rt_timer_start(timer);
    if(err != RT_EOK)
        rt_kprintf("rt_timer_start error...");
}
/*
************************************************************************************************************************
*                                                   CHANGE A TIMER WAIT TIME
*
* Description: This function is called by your application code to change a timer wait time.
*
* Arguments  : timer          Is a pointer to an OS_TMR
*
*           ticks           is the wait time
************************************************************************************************************************
*/
void tls_os_timer_change(tls_os_timer_t *timer, u32 ticks)
{
//    rt_kprintf("%s %d %p tick:%d\r\n", __FUNCTION__, __LINE__, timer, ticks);
    rt_err_t err = rt_timer_control(timer, RT_TIMER_CTRL_SET_TIME, &ticks);
    if(err != RT_EOK)
        rt_kprintf("rt_timer_control error:%d\r\n", err);

    err = rt_timer_start(timer);
    if(err != RT_EOK)
        rt_kprintf("rt_timer_start error:%d\r\n", err);
}
/*
************************************************************************************************************************
*                                                   STOP A TIMER
*
* Description: This function is called by your application code to stop a timer.
*
* Arguments  : timer          Is a pointer to the timer to stop.
*
************************************************************************************************************************
*/
void tls_os_timer_stop(tls_os_timer_t *timer)
{
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, timer);
    rt_err_t err = rt_timer_stop(timer);
//    if(err != RT_EOK)
//        rt_kprintf("rt_timer_stop error:%d...\r\n", err);
}

/*
************************************************************************************************************************
*                                                   Delete A TIMER
*
* Description: This function is called by your application code to delete a timer.
*
* Arguments  : timer          Is a pointer to the timer to delete.
*
************************************************************************************************************************
*/
int tls_os_timer_delete(tls_os_timer_t *timer)
{
    rt_list_t *node, *next;
    struct __tls_os_timer *tls_os_timer;
    
    rt_err_t err = rt_timer_detach(timer);
    if(err != RT_EOK)
        rt_kprintf("rt_timer_detach error...");
    
//    rt_kprintf("%s %d\r\n", __FUNCTION__, __LINE__);
    
    if(!rt_list_isempty(&__tls_os_timer_head))
    {
        for(node = __tls_os_timer_head.next; node != &__tls_os_timer_head; node = next)
        {
            next = node->next;
            rt_list_remove(node);
            tls_os_timer = rt_list_entry(node, struct __tls_os_timer, list);
            if((tls_os_timer != RT_NULL) && (timer == tls_os_timer))
            {
                rt_free(tls_os_timer->timer);
                rt_free(tls_os_timer);
                tls_os_timer = RT_NULL;
                break;
            }
        }
    }

    return 1;
}


/*
*********************************************************************************************************
*                                       DELAY TASK 'n' TICKS
*
* Description: This function is called to delay execution of the currently running task until the
*              specified number of system ticks expires.  This, of course, directly equates to delaying
*              the current task for some time to expire.  No delay will result If the specified delay is
*              0.  If the specified delay is greater than 0 then, a context switch will result.
*
* Arguments  : ticks     is the time delay that the task will be suspended in number of clock 'ticks'.
*                        Note that by specifying 0, the task will not be delayed.
*
* Returns    : none
*********************************************************************************************************
*/
void tls_os_time_delay(u32 ticks)
{
    rt_thread_delay(ticks);
}

/*
*********************************************************************************************************
*                                           CREATE A SEMAPHORE
*
* Description: This function creates a semaphore.
*
* Arguments  :sem		 is a pointer to the event control block (OS_EVENT) associated with the
*                            created semaphore
*			cnt           is the initial value for the semaphore.  If the value is 0, no resource is
*                            available (or no event has occurred).  You initialize the semaphore to a
*                            non-zero value to specify how many resources are available (e.g. if you have
*                            10 resources, you would initialize the semaphore to 10).
*
* Returns    : TLS_OS_SUCCESS	The call was successful
*			TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_sem_create(tls_os_sem_t **sem, u32 cnt)
{
    char name[RT_NAME_MAX];
    static uint16_t __sem;
    rt_sem_t _rt_sem;
    
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, sem);
    memset(name, 0, sizeof(name));
    snprintf(name, RT_NAME_MAX, "sem-%03d", __sem ++);
    
    _rt_sem = (tls_os_sem_t *)rt_sem_create(name, cnt, RT_IPC_FLAG_FIFO);
    if(_rt_sem == RT_NULL)
        return TLS_OS_ERROR;
    
    *sem = _rt_sem;
    return TLS_OS_SUCCESS;
}

/*
*********************************************************************************************************
*                                         DELETE A SEMAPHORE
*
* Description: This function deletes a semaphore and readies all tasks pending on the semaphore.
*
* Arguments  : sem        is a pointer to the event control block associated with the desired
*                            semaphore.
*
* Returns    : TLS_OS_SUCCESS             The call was successful and the semaphore was deleted
*                            TLS_OS_ERROR
*
*********************************************************************************************************
*/
tls_os_status_t tls_os_sem_delete(tls_os_sem_t *sem)
{
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, sem);
    if(rt_sem_delete((rt_sem_t)sem) == RT_EOK)
        return TLS_OS_SUCCESS;
    else
        return TLS_OS_ERROR;
}

/*
*********************************************************************************************************
*                                           PEND ON SEMAPHORE
*
* Description: This function waits for a semaphore.
*
* Arguments  : sem        is a pointer to the event control block associated with the desired
*                            semaphore.
*
*              wait_time       is an optional timeout period (in clock ticks).  If non-zero, your task will
*                            wait for the resource up to the amount of time specified by this argument.
*                            If you specify 0, however, your task will wait forever at the specified
*                            semaphore or, until the resource becomes available (or the event occurs).
*
* Returns    : TLS_OS_SUCCESS
*			TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_sem_acquire(tls_os_sem_t *sem, u32 wait_time)
{
    rt_int32_t time;
    
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, sem);
	if((0 == wait_time) || (wait_time > RT_TICK_MAX / 2))
		time = RT_WAITING_FOREVER;
	else
		time = wait_time;

    if(rt_sem_take((rt_sem_t)sem, time) == RT_EOK)    
        return TLS_OS_SUCCESS;
    else
        return TLS_OS_ERROR;
}

/*
*********************************************************************************************************
*                                         POST TO A SEMAPHORE
*
* Description: This function signals a semaphore
*
* Arguments  : sem        is a pointer to the event control block associated with the desired
*                            semaphore.
*
* Returns    : TLS_OS_SUCCESS
*			TLS_OS_ERROR
*********************************************************************************************************
*/
 tls_os_status_t tls_os_sem_release(tls_os_sem_t *sem)
{
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, sem);
    if(rt_sem_release((rt_sem_t)sem) == RT_EOK)
        return TLS_OS_SUCCESS;
    else
        return TLS_OS_ERROR;
}
      
/*
*********************************************************************************************************
*                                        CREATE A MESSAGE QUEUE
*
* Description: This function creates a message queue if free event control blocks are available.
*
* Arguments  : queue    is a pointer to the event control clock (OS_EVENT) associated with the
*                                created queue
*
*           queue_start         is a pointer to the base address of the message queue storage area.  The
*                            storage area MUST be declared as an array of pointers to 'void' as follows
*
*                            void *MessageStorage[size]
*
*               queue_size          is the number of elements in the storage area
*
*           msg_size
*
* Returns    : TLS_OS_SUCCESS
*           TLS_OS_ERROR
*********************************************************************************************************
*/

tls_os_status_t tls_os_queue_create(tls_os_queue_t **queue, u32 queue_size)
{
    static rt_uint16_t __mq;
    rt_mq_t __rt_mq;
    char name[RT_NAME_MAX];
	rt_uint32_t __queue_size = 10;

    if(queue_size != 0)
        __queue_size = queue_size;
    
    memset(name, 0, sizeof(name));
    snprintf(name, RT_NAME_MAX, "mq-%03d", __mq ++);
    
    __rt_mq = rt_mq_create(name, sizeof(void *), __queue_size, RT_IPC_FLAG_FIFO);
    if(__rt_mq == RT_NULL)
        return TLS_OS_ERROR;

    *queue = __rt_mq;
    
    return TLS_OS_SUCCESS;
}

/*
*********************************************************************************************************
*                                        DELETE A MESSAGE QUEUE
*
* Description: This function deletes a message queue and readies all tasks pending on the queue.
*
* Arguments  : queue        is a pointer to the event control block associated with the desired
*                            queue.
*
*
* Returns    : TLS_OS_SUCCESS
*           TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_queue_delete(tls_os_queue_t *queue)
{
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, queue);

    if(rt_mq_delete((rt_mq_t)queue) != RT_EOK)
    {
        rt_kprintf("rt_mq_delete error...\r\n");
        return TLS_OS_ERROR;
    }
    
    return TLS_OS_SUCCESS;
}

/*
*********************************************************************************************************
*                                        POST MESSAGE TO A QUEUE
*
* Description: This function sends a message to a queue
*
* Arguments  : queue        is a pointer to the event control block associated with the desired queue
*
*               msg          is a pointer to the message to send.
*
*           msg_size
* Returns    : TLS_OS_SUCCESS
*           TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_queue_send(tls_os_queue_t *queue,
     void *msg,
     u32 msg_size)
{
    rt_err_t err;
//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, queue);

    err = rt_mq_send(queue, &msg, sizeof(void *));
    if(err != RT_EOK)
    {
        rt_kprintf("rt_mq_send error %d...\r\n", err);
        return TLS_OS_ERROR;
    }
    
    return TLS_OS_SUCCESS;
}
/*
*********************************************************************************************************
*                                     PEND ON A QUEUE FOR A MESSAGE
*
* Description: This function waits for a message to be sent to a queue
*
* Arguments  : queue        is a pointer to the event control block associated with the desired queue
*
*           msg     is a pointer to the message received
*
*           msg_size
*
*              wait_time       is an optional timeout period (in clock ticks).  If non-zero, your task will
*                            wait for a message to arrive at the queue up to the amount of time
*                            specified by this argument.  If you specify 0, however, your task will wait
*                            forever at the specified queue or, until a message arrives.
*
* Returns    : TLS_OS_SUCCESS
*           TLS_OS_ERROR
*********************************************************************************************************
*/
tls_os_status_t tls_os_queue_receive(tls_os_queue_t *queue,
     void **msg,
     u32 msg_size,
     u32 wait_time)
{
    rt_err_t err;
    rt_int32_t time;

//    rt_kprintf("%s %d %p\r\n", __FUNCTION__, __LINE__, queue);
    if((wait_time == 0) || (wait_time > RT_TICK_MAX / 2))
        time = RT_WAITING_FOREVER;
    else
        time = wait_time;
    
    err = rt_mq_recv(queue, msg, sizeof(void *), time);
    if(err != RT_EOK)
    {
        rt_kprintf("rt_mq_recv error %d...\r\n", err);
        return TLS_OS_ERROR;
    }
    return TLS_OS_SUCCESS;
}

#include "wm_regs.h"
#define	P_ALIGN_BIT	(0x01<<0)	// bit=1 ??? bit=0???
#define  P_FILL_BIT		(0x01<<1)	//bit = 1??'0',????' '
#define  P_BIG_BIT		(0x01<<2)	//bit=1,??,????
     
int sendchar(int ch)
{
    tls_reg_write32(HR_UART0_INT_MASK, 0x3);
    
    if(ch == '\n')  
    {
        while (tls_reg_read32(HR_UART0_FIFO_STATUS) & 0x3F);
        tls_reg_write32(HR_UART0_TX_WIN, '\r');
    }
    
    while(tls_reg_read32(HR_UART0_FIFO_STATUS) & 0x3F);
    tls_reg_write32(HR_UART0_TX_WIN, (char)ch);
    tls_reg_write32(HR_UART0_INT_MASK, 0x0);
    
    return ch;
}

int Int2Str(char *str,int num,char base,char width,int opflag) 
{   
    char temp; 
    int len = 0;
    signed char k = 0;
    char *str_bk;
    signed char k_bk;

    if(num <0) 
    { 
        num = -num;   
        *str='-';
        str++;
        len++;  
    }
    
    if(0 == num)
    {
        *str = '0';
        str ++;
        k ++;
    }
    
    while(num) 
    {   
        temp= num%base; 
        if(temp > 9) // insert hexdecimal--ABCDEF-- 
        {  
            temp-=10;  
            if(opflag & P_BIG_BIT)
                *str = temp + 'A';      
            else
                *str = temp + 'a';  
        } 
        else 
        {  
            *str = temp + '0'; 
        }
        num=num/base; 
        str++;
        k++; 
    }  

    if(opflag&P_ALIGN_BIT)  //???
    {
        str_bk = str;
        k_bk = k;       //????????,??????
        str --;
        k --;
        while(k>0) 
        {
            temp = *str; 
            *str = *(str-k); 
            *(str-k) = temp; 
            str--; 
            k-=2; 
        }  
        k = k_bk;
        str = str_bk;
    }   

    //??????' '??
    while(width>k) 
    {  
        if(opflag&P_FILL_BIT)
        {
            *str++ ='0';
        }
        else
        {
            *str++ =' ';
        }
        k++; 
    }

    len=len+k; 
    *str-- = '\0'; 
    k--; 
    if(0 == (opflag&P_ALIGN_BIT))   //???
    {
        //?? 
        while(k>0) 
        {
            temp = *str; 
            *str = *(str-k); 
            *(str-k) = temp; 
            str--; 
            k-=2; 
        }  
    } 
return len; 
}  

int wm_vprintf(const char *fmt, va_list arg_ptr)
{
    unsigned char width=0;  //????
    unsigned int len;           //????
    char *fp = fmt;  
    //va_list arg_ptr; 
    char *pval;
    int opflag = 0;
    char store[20];
    char c;
    int i;
    char* str;

    //va_start(arg_ptr, fmt); //arg_ptr ???????
    while (*fp !='\0') 
    {
        c = *fp++; 
        if (c != '%') 
        {
            sendchar(c);
        } 
        else 
        { 
            width = 0;  //??????
            opflag = 0;
            if('-' == *fp)
            {
                opflag |= P_ALIGN_BIT;//???
                fp ++;
            }
            
            if('0' == *fp)  //????
            {
                opflag |= P_FILL_BIT;   //??
                fp ++;
            }

            while(*fp>='0'&&*fp<='9') 
            {  
                width = width * 10 + (*fp) - '0'; 
                fp++; 
            } 
            
            if('.' == *fp)  //????????,???
            {
                fp ++;
                while(*fp>='0'&&*fp<='9') 
                {  
                    fp++; 
                }
            }

            while('l' == *fp)
            {
                fp ++;
            }           

            switch (*fp) 
            {  
            case 'c': 
            case 'C': 
                c = (char)va_arg(arg_ptr, int);
                sendchar(c);
                break;
                
            case 'd': 
            case 'i':  
            case 'u':   
                i = va_arg(arg_ptr, int);
                str = store;
                Int2Str(store,i,10,width,opflag); 
                while( *str != '\0') sendchar(*str++);
                break; 
                
            case 'x': 
            case 'X':  
                i = va_arg(arg_ptr, int);
                str = store;
                if('X' == *fp)
                {
                    opflag |= P_BIG_BIT;
                }                    
                Int2Str(store,i,16,width,opflag);         
                while( *str != '\0') sendchar(*str++);
                break; 
            
            case 'o':
                i = va_arg(arg_ptr, int);
                str = store;
                Int2Str(store,i,8,width,opflag);           
                while( *str != '\0') sendchar(*str++);
                break;
            
            case 's': 
            case 'S':
                pval=va_arg(arg_ptr,char*);
                len = strlen(pval);
                if((width > len) && (0 == (opflag&P_ALIGN_BIT)))        //???
                {
                    for(i = 0;i < (width - len);i ++)   //?????
                    {
                    sendchar(' ');
                    }
                }
        
                for(i=0;i < len;i++)
                {
                    sendchar(pval[i]);                  
                }
                
                if((width > len) && (opflag&P_ALIGN_BIT))       //???
                {
                    for(i = 0;i < (width - len);i ++)   //?????
                    {
                        sendchar(' ');
                    }
                }
                break; 
                
            case '%':  
                sendchar('%');
                break; 
            
            default: 
                break; 
            }
            fp++; 
        } 
    }  
    //va_end(arg_ptr); 
    return 0; 

}

int wm_printf(const char *fmt,...) 
{
    va_list ap;

    va_start(ap, fmt);
    wm_vprintf(fmt,ap);
    va_end(ap);
}

