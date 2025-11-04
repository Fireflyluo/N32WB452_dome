/* RT-Thread 配置文件 */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* 线程名称最大长度 */
#define RT_NAME_MAX 8

/* 内存对齐大小 */
#define RT_ALIGN_SIZE 4

/* 最大优先级数 */
#define RT_THREAD_PRIORITY_MAX 32

/* 每秒节拍数 */
#define RT_TICK_PER_SECOND 100

/* SECTION: RT_DEBUG */
/* 线程调试 */
#define RT_DEBUG
#define RT_DEBUG_INIT 1
#define RT_USING_OVERFLOW_CHECK

/* 使用钩子函数 */
/* #define RT_USING_HOOK */

/* 使用软件定时器 */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO       4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_TIMER_TICK_PER_SECOND   10

/* SECTION: IPC */
/* 使用信号量 */
#define RT_USING_SEMAPHORE

/* 使用互斥量 */
#define RT_USING_MUTEX

/* 使用事件 */
/* #define RT_USING_EVENT */

/* 使用邮箱 */
/* #define RT_USING_MAILBOX */

/* 使用消息队列 */
/* #define RT_USING_MESSAGEQUEUE */

/* SECTION: 内存管理 */
/* 使用内存池管理 */
/* #define RT_USING_MEMPOOL */

/* 使用动态堆管理 */
#define RT_USING_HEAP

/* 使用小型内存管理算法 */
#define RT_USING_SMALL_MEM
#define RT_USING_TINY_SIZE

/* SECTION: 设备系统 */
/* 使用设备系统 */
#define RT_USING_DEVICE
// <bool name="RT_USING_DEVICE_IPC" description="使用设备通信" default="true" />
#define RT_USING_DEVICE_IPC
// <bool name="RT_USING_SERIAL" description="使用串口" default="true" />
// #define RT_USING_SERIAL

/* SECTION: 控制台选项 */
#define RT_USING_CONSOLE
/* 控制台缓冲区大小 */
#define RT_CONSOLEBUF_SIZE 128
// <string name="RT_CONSOLE_DEVICE_NAME" description="控制台设备名称" default="uart1" />
#define RT_CONSOLE_DEVICE_NAME "usart3"

#define RT_USING_SERIAL
#define RT_USING_USART3

/* RT-Thread 组件 */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN

/* SECTION: finsh命令行工具 */
/* 开启 FinSH */
#define RT_USING_FINSH
/* 开启 msh 功能 */
#define FINSH_USING_MSH
/* 将线程名称定义为 tshell */
#define FINSH_THREAD_NAME "tshell"
/* 仅使用 msh 命令行 */
#define FINSH_USING_MSH_ONLY
/* 开启历史命令 */
#define FINSH_USING_HISTORY
/* 记录 5 行历史命令 */
#define FINSH_HISTORY_LINES 5

/* 开启使用 Tab 键 */
#define FINSH_USING_SYMTAB
/* 开启描述功能 */
#define FINSH_USING_DESCRIPTION

/* 定义 FinSH 线程优先级为 20 */
#define FINSH_THREAD_PRIORITY 20
/* 定义 FinSH 线程的栈大小为 4KB */
#define FINSH_THREAD_STACK_SIZE 4096
/* 定义命令字符长度为 80 字节 */
#define FINSH_CMD_SIZE 80

/* 最大输入参数数量为 10 个 */
#define FINSH_ARG_MAX 10

#define RCC_GPIOA_CLK_ENABLE
#define RCC_GPIOB_CLK_ENABLE
#define RCC_GPIOC_CLK_ENABLE
#define RCC_GPIOD_CLK_ENABLE
#define RCC_GPIOE_CLK_ENABLE

#define RT_USING_PIN

/* SECTION: libc库管理 */
#define RT_USING_LIBC

/* SECTION: 设备文件系统 */
/* #define RT_USING_DFS */
// #define RT_USING_DFS_ELMFAT
#define RT_DFS_ELM_WORD_ACCESS
/* FatFs模块的可重入性（线程安全） */
#define RT_DFS_ELM_REENTRANT
/* 要使用的卷数（逻辑驱动器） */
#define RT_DFS_ELM_DRIVES 2
/* #define RT_DFS_ELM_USE_LFN           1 */
#define RT_DFS_ELM_MAX_LFN 255
/* 要处理的最大扇区大小 */
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512

#define RT_USING_DFS_ROMFS

/* 最大挂载文件系统数量 */
#define DFS_FILESYSTEMS_MAX 2
/* 最大打开文件数 */
#define DFS_FD_MAX 4

#endif
