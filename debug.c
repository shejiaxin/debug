#include "includes.h"

FISH_st fishcom;
CPU_STK FishTaskStk[FISH_STK_SIZE];
OS_SEM fish_send_sem;
OS_SEM fish_recv_sem;
OS_TCB  FishTackTCB;


sys_mode_t fish_flag = SYS_MODE_PASS;  
uint8_t fish_rev_buf[MAX_BUF_LEN] = {0};
static uint8_t fish_send_buf[200] = {0};

/*
*********************************************************************************************************
*	函 数 名: fish_hw_init
*	功能说明: fish硬件io初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void  fish_hw_init(void)
{
#ifndef USE_VIRTUAL_COM
    bsp_InitUart(FISH_COM, 115200);
    //USART1_Init(115200);
#endif
    
#ifdef FISH_USE_DMA_RX
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    McuDMAChannel_Config(RCC_AHBPeriph_DMA1,DMA2_Channel5,(uint32_t)&USART1->RDR,(uint32_t)fishcom.FishRxBuf,\
                         DMA_DIR_PeripheralSRC,DMA_Mode_Normal,MAX_BUF_LEN);
    DMA_Cmd(DMA1_Channel5, ENABLE);
#endif
    
    fish_flag = SYS_MODE_PASS;
    
    #ifdef DEBUG_OUTPUT0
    DEBUG_UART("\r\n=================================================================");
    DEBUG_UART("\r\nversion:%s",app_Version);
    DEBUG_UART("\r\nbuild date:%s", "(" __DATE__ " - " __TIME__ ")");
    DEBUG_UART("\r\nDEBUG串口初始化......成功!");
    #endif
}

/*
*********************************************************************************************************
*	函 数 名: fish_print
*	功能说明: fish打印输出
*	形    参：串口格式化输出，同printf
*	返 回 值: 无
*********************************************************************************************************
*/
void fish_print(const char *fmt, ...)
{
    OS_ERR err;

    OSSemPend((OS_SEM      *)&fish_send_sem,
              (OS_TICK      )0,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    if(OS_ERR_NONE != err)
        return;
    
    va_list va_args;
    va_start(va_args, fmt);
    vsnprintf((char*)fish_send_buf, sizeof(fish_send_buf), fmt, va_args);
    va_end(va_args);
    
#ifndef USE_VIRTUAL_COM
    USART1_SendStr(fish_send_buf,strlen((char*)fish_send_buf));
#else
    VirtualCom_SendBytes(fish_send_buf,strlen((char*)fish_send_buf));
#endif
    
    OSSemPost(&fish_send_sem,OS_OPT_POST_1,&err);
}

#ifndef  USE_SERIAL_TASK_PRINTF  
void debug_print(const char *fmt, ...)
{
//    uint8_t err;
      char  buf_str[200 + 1];
//    OSSemPend(fish_send_sem,0,&err);
//    if(OS_ERR_NONE != err)
//        return;

//    va_list va_args;
//    va_start(va_args, fmt);
//    vsnprintf((char*)fish_send_buf, sizeof(fish_send_buf), fmt, va_args);
//    va_end(va_args);
//    //VirtualCom_SendBytes(fish_send_buf,strlen((char*)fish_send_buf));
//    USART1_SendStr(fish_send_buf,strlen((char*)fish_send_buf));
//    OSSemPost(fish_send_sem);
    
    
    va_list   v_args;
    OS_ERR err;

    va_start(v_args, fmt);
   (void)vsnprintf((char       *)&buf_str[0],
                   (size_t      ) sizeof(buf_str),
                   (char const *) fmt,
                                  v_args);
    va_end(v_args);
            
    /* 互斥操作 */
    OSSemPend((OS_SEM  *)&fish_send_sem,
              (OS_TICK  )0u,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);

    printf("%s", buf_str);

   (void)OSSemPost((OS_SEM  *)&fish_send_sem,
                   (OS_OPT   )OS_OPT_POST_1,
                   (OS_ERR  *)&err);
}
#endif

void debug_send(char *buf, uint16_t len)
{
#ifdef DEBUG_OUTPUT
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&fish_send_sem,
              (OS_TICK      )0,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    if(OS_ERR_NONE != err)
        return;
    
    memcpy(fish_send_buf, buf, len);
    
#ifndef USE_VIRTUAL_COM
    USART1_SendStr(fish_send_buf,strlen((char*)fish_send_buf));
#else
    VirtualCom_SendBytes(fish_send_buf,strlen((char*)fish_send_buf));
#endif

    OSSemPost(&fish_send_sem,OS_OPT_POST_1,&err);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: fish_menu
*	功能说明: 打印输出fish菜单
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_menu(void)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    fish_print("/****** fish menu ******/\r\n");
    fish_print("r. Reset System.\r\n");
    fish_print("c. check stack.\r\n");
    fish_print("?. List the fish menu.\r\n");
    fish_print("l. List the system variables.\r\n");
    fish_print("u. Configuration initialization.\r\n");
    fish_print("q. quit fish mode\r\n");
}

extern void fault_test_by_unalign(void);
/*
*********************************************************************************************************
*	函 数 名: fish_entry
*	功能说明: fish入口
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_entry(void)
{
    if (fish_flag == SYS_MODE_FISH)
        return;
    
    fish_flag = SYS_MODE_FISH;
    fish_print("fish >>\n");
    #ifdef DEBUG_CM_BACKTRACE
        //fault_test_by_unalign();
        //fault_test_by_div0();
    #endif
}

static void fish_quit(void)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    fish_flag = SYS_MODE_PASS;
    fish_print("quit fish!\n");
}

/*
*********************************************************************************************************
*	函 数 名: config_rs485_output
*	功能说明: 配置rs485的调试信息输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
#ifdef ENABLE_EXT_RS485
static void config_rs485_output(uint8_t state)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    fish_print("\r\ns-->%d",state);

    rs485_output_ctrl(state);
}
#endif

/*
*********************************************************************************************************
*	函 数 名: config_can_output
*	功能说明: 配置can的调试信息输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
#ifdef ENABLE_CAN_COMM
static void config_can_output(uint8_t state)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    fish_print("\r\ns-->%d",state);
    can_debug_ctrl(state);
}
#endif

/*
*********************************************************************************************************
*	函 数 名: control_bat_box_lock
*	功能说明: 控制电池仓锁开关
*	形    参：state -- 0:关锁，1:开锁
*	返 回 值: 无
*********************************************************************************************************
*/
static void control_bat_box_lock(uint8_t state)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    fish_print("\r\np-->%d",state);
    
    BatBoxLockProc(state);
}

/*
*********************************************************************************************************
*	函 数 名: config_can_output
*	功能说明: 配置can的调试信息输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void config_can_test(uint8_t state)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    fish_print("\r\nb-->%d",state);
    
#ifdef ENABLE_CAN_COMM    
    can_test(state);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: fish_reset
*	功能说明: 复位MCU
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_reset(void)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    __set_PRIMASK(1);
    NVIC_SystemReset();
    while (1)
        ;
}

/*
*********************************************************************************************************
*	函 数 名: fish_list_var
*	功能说明: 串口打印指定变量的值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_list_var(void)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    uint8_t i;

	int16_t ctl_temp = -7;
	
    fish_print("\r\n=================CAN INFO=====================");
    fish_print("\r\nLEC(ErrCode) = 0x%X", CAN_GetLastErrorCode(CAN1));
    fish_print("\r\nREC          = %d", CAN_GetReceiveErrorCounter(CAN1));
    fish_print("\r\nTEC          = %d", CAN_GetLSBTransmitErrorCounter(CAN1));
    fish_print("\r\nCAN1->ESR    = 0x%08X", CAN1->ESR);
    fish_print("\r\nCAN1->TSR    = 0x%08X", CAN1->TSR);
    fish_print("\r\nCAN1->RF0R   = 0x%08X", CAN1->RF0R);
    fish_print("\r\nCAN1->RF1R   = 0x%08X", CAN1->RF1R);
    fish_print("\r\n==============================================");
    
    fish_print("\r\n===========Ebike INFO=========================");
    fish_print("\r\nState1       = 0x%X", EbikeData.State[0]);
    fish_print("\r\nState2       = 0x%X", EbikeData.State[1]);
    fish_print("\r\nVoltageGrade = %d", EbikeData.VoltGrade);
    fish_print("\r\nTemp         = %dC", EbikeData.Temperature);
    fish_print("\r\nSpeed        = %dkm/h", EbikeData.Speed);
    fish_print("\r\nMotorRpm     = %dRPM", EbikeData.MotorRpm);
    fish_print("\r\nSoftVer      = %d", EbikeData.SoftVer);
    fish_print("\r\nTrip         = %dkm", EbikeData.Trip);
    fish_print("\r\nTotalMileage = %dkm", EbikeData.TotalMileage);
    fish_print("\r\nFault1       = 0x%X", EbikeData.Fault[0]);
    fish_print("\r\nFault2       = 0x%X", EbikeData.Fault[1]);
    fish_print("\r\n==============================================\r\n");
                            
    fish_print("\r\nNFC id:");
    for(i = 0; i < 6; i++)
       fish_print("%02X ", DeviceInfo.NFC_id[i]);
    fish_print("\r\nSumTime:%08X", AutoTrackData.SumTime);
    fish_print("\r\nDeviceCallFlag:%d", DeviceInfo.DeviceCallFlag);
	fish_print("\r\nctl_temp: 0x%04X, %d", ctl_temp, ctl_temp);
    ctl_temp = ((456 * 0.1)  - 50) * 10; //串口打印实际值=-43,0xFFFFFFD5
    fish_print("\r\ntmp: 0x%04X, %d", ctl_temp, ctl_temp);
    //fish_print("\r\nconnect_flag:%d",gprs_cfg.connect_ok);
    //fish_print("\r\nsend_fail_cnt:%d",gprs_cfg.send_fail_cnt);
    fish_print("\r\nSimCCIDNumber:%.*s", 20, DeviceInfo.SimCCIDNumber);
    fish_print("\r\nRemainTime:%d" , AutoTrackData.RemainTime);
	fish_print("\r\nHandshakeFailCnt:%d", PlatformData_HY.HandshakeFailCnt);
	fish_print("\r\nStatisticSendFailSum:%d", PlatformData.StatisticSendFailSum);
	fish_print("\r\nStatisticSendSuccessSum:%d\r\n", PlatformData.StatisticSendSuccessSum);
}

/*
*********************************************************************************************************
*	函 数 名: fish_default_init
*	功能说明: 初始化系统参数
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_default_init(void)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    SystemParamFactorySettingsProc();
    fish_print("\r\nfactory setting!!!\r\n");
    fish_print("fish >>\r\n");
    while(1);
}

/*
*********************************************************************************************************
*   函 数 名: DispTaskInfo
*   功能说明: 将uCOS-III任务信息通过串口打印出来
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
void DispTaskInfo(void)
{
	OS_ERR err;
	CPU_TS_TMR  ts_int;
	CPU_INT16U  version;
	CPU_INT32U  cpu_clk_freq;
    OS_TCB      *p_tcb;
    float CPU = 0.0f;
    CPU_SR_ALLOC();
	
    if (fish_flag == SYS_MODE_PASS)
        return;
    
    OS_CRITICAL_ENTER();
    p_tcb = OSTaskDbgListPtr;
    OS_CRITICAL_EXIT();

	version = OSVersion(&err);
	cpu_clk_freq = SystemCoreClock;
	ts_int = CPU_IntDisMeasMaxGet ();
	
    fish_print("===============================================================\r\n");
    fish_print(" 优先级 使用栈 剩余栈 百分比 利用率   任务名\r\n");
    fish_print("  Prio   Used  Free   Per    CPU     Taskname\r\n");

    while (p_tcb != (OS_TCB *)0) 
    {
        CPU = (float)p_tcb->CPUUsage / 100;
        fish_print("   %2d  %5d  %5d   %02d%%   %5.2f%%   %s\r\n", 
        p_tcb->Prio, 
        p_tcb->StkUsed, 
        p_tcb->StkFree, 
        (p_tcb->StkUsed * 100) / (p_tcb->StkUsed + p_tcb->StkFree),
        CPU,
        p_tcb->NamePtr);        
        
        OS_CRITICAL_ENTER();
        p_tcb = p_tcb->DbgNextPtr;
        OS_CRITICAL_EXIT();
    }
	fish_print(" uC/OS Ver：V%d.%02d.%02d\r\n",version / 10000, version % 10000 / 100, version % 100 );
	fish_print(" CPU主频：%d MHz\r\n", cpu_clk_freq / 1000000 ); 
	fish_print(" 最大中断时间：%d us\r\n", ts_int / ( cpu_clk_freq / 1000000 ) ); 
	fish_print(" 最大锁调度器时间：%d us\r\n", OSSchedLockTimeMax / ( cpu_clk_freq / 1000000 ) );
	fish_print(" CPU最大使用率：%d.%d%%\r\n", OSStatTaskCPUUsageMax / 100, OSStatTaskCPUUsageMax % 100 );
}

/*
*********************************************************************************************************
*	函 数 名: fish_check_stack
*	功能说明: OS堆栈检查，并通过串口输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_check_stack(void)
{
    DispTaskInfo();
}

#ifdef  USE_MJ_MULTI_MODE 
/*
*********************************************************************************************************
*	函 数 名: dbg_can_id
*	功能说明: 打印指定can id的数据
*	形    参：buffer: 指向缓存区，len: 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void dbg_can_id(uint8_t *buffer,uint16_t len)
{
	/* 命令格式： id=xxx, xxx是can id(标准帧), 比如230、231 */

	uint16_t cmd_data = 0x00, i = 0;
	
	if (fish_flag == SYS_MODE_PASS)
        return;

	cmd_data = CharToValue(buffer[3])<<8 | CharToValue(buffer[4])<<4 | CharToValue(buffer[5]);
	
	fish_print("\r\ncmd: %s", buffer);
    
#if 1 
	if(cmd_data >= 0x090 && cmd_data <= 0x271)
	{
		switch (cmd_data)
		{
//============================打印控制器上报信息========================================             
			case CTRL_1_STATE_MSG:
				fish_print("\r\n============[id 0x230]=================");
				fish_print("\r\ngear_state  : 0x%02X", CtrlData[CTL_NUM_1].state.byte1.gear_state);
				fish_print("\r\nbody_state  : 0x%02X", CtrlData[CTL_NUM_1].state.byte2.body_state);
				fish_print("\r\ndevice_state: 0x%02X", CtrlData[CTL_NUM_1].state.byte3.device_state);
				for(i = 0; i < 5; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",3+i, CtrlData[CTL_NUM_1].state.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_STATE_MSG:
				fish_print("\r\n=============[id 0x231]================");
		        fish_print("\r\ngear_state  : 0x%02X", CtrlData[CTL_NUM_2].state.byte1.gear_state);
		        fish_print("\r\nbody_state  : 0x%02X", CtrlData[CTL_NUM_2].state.byte2.body_state);
		        fish_print("\r\ndevice_state: 0x%02X", CtrlData[CTL_NUM_2].state.byte3.device_state);
		        for(i = 0; i < 5; i++)
		            fish_print("\r\nreserve[%d]: 0x%02X",3+i, CtrlData[CTL_NUM_2].state.reserve[i]);
		        fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_DISP_MSG:
				fish_print("\r\n============[id 0x234]=================");
				fish_print("\r\nvehicle_speed  : %dkm/h", CtrlData[CTL_NUM_1].disp_msg.vehicle_speed);
				fish_print("\r\ntrip           : %dkm", CtrlData[CTL_NUM_1].disp_msg.trip);
				fish_print("\r\nodo            : %dkm", CtrlData[CTL_NUM_1].disp_msg.odo);
				fish_print("\r\nmotor_speed    : %dRPM", CtrlData[CTL_NUM_1].disp_msg.motor_speed);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_DISP_MSG:
				fish_print("\r\n============[id 0x235]=================");
				fish_print("\r\nvehicle_speed  : %dkm/h", CtrlData[CTL_NUM_2].disp_msg.vehicle_speed);
				fish_print("\r\ntrip           : %dkm", CtrlData[CTL_NUM_2].disp_msg.trip);
				fish_print("\r\nodo            : %dkm", CtrlData[CTL_NUM_2].disp_msg.odo);
				fish_print("\r\nmotor_speed    : %dRPM", CtrlData[CTL_NUM_2].disp_msg.motor_speed);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_DISP1_MSG:
				fish_print("\r\n============[id 0x238]=================");
				fish_print("\r\npercent_voltage  : %d%%", CtrlData[CTL_NUM_1].disp_msg1.percent_voltage);
				fish_print("\r\npercent_current  : %d%%", CtrlData[CTL_NUM_1].disp_msg1.percent_current);
				fish_print("\r\nctrl_temp        : %.1f℃", ((CtrlData[CTL_NUM_1].disp_msg1.ctrl_temp/10.0 - 50)));
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_DISP1_MSG:
				fish_print("\r\n============[id 0x239]=================");
				fish_print("\r\npercent_voltage  : %d%%", CtrlData[CTL_NUM_2].disp_msg1.percent_voltage);
				fish_print("\r\npercent_current  : %d%%", CtrlData[CTL_NUM_2].disp_msg1.percent_current);
				fish_print("\r\nctrl_temp        : %.1f℃", ((CtrlData[CTL_NUM_2].disp_msg1.ctrl_temp/10.0 - 50)));
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_VOLT_MSG:
				fish_print("\r\n============[id 0x218]=================");
				fish_print("\r\nbattery_voltage    : %.2fV", CtrlData[CTL_NUM_1].volt_cur_temp.battery_voltage/100.0);       //0.01V/bit
				fish_print("\r\nbat_discharge_cur  : %.2fA", CtrlData[CTL_NUM_1].volt_cur_temp.bat_discharge_cur/100.0);     //0.01A/bit
				fish_print("\r\nmos_temp           : %.1f℃", ((CtrlData[CTL_NUM_1].volt_cur_temp.mos_temp/10.0 - 50)));
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_VOLT_MSG:
				fish_print("\r\n============[id 0x219]=================");
				fish_print("\r\nbattery_voltage    : %.2fV", CtrlData[CTL_NUM_2].volt_cur_temp.battery_voltage/100.0);       //0.01V/bit
				fish_print("\r\nbat_discharge_cur  : %.2fA", CtrlData[CTL_NUM_2].volt_cur_temp.bat_discharge_cur/100.0);     //0.01A/bit
				fish_print("\r\nmos_temp           : %.1f℃", ((CtrlData[CTL_NUM_2].volt_cur_temp.mos_temp/10.0 - 50)));
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_FAULT_MSG:
				fish_print("\r\n============[id 0x265]=================");
				fish_print("\r\nmalf_state1  : 0x%02X", CtrlData[CTL_NUM_1].Fault.byte1.malf_state1);
				fish_print("\r\nmalf_state2  : 0x%02X", CtrlData[CTL_NUM_1].Fault.byte2.malf_state2);
				fish_print("\r\nmalf_state3  : 0x%02X", CtrlData[CTL_NUM_1].Fault.byte3.malf_state3);
				for(i = 0; i < 5; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",3+i, CtrlData[CTL_NUM_1].Fault.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_FAULT_MSG:
				fish_print("\r\n============[id 0x266]=================");
				fish_print("\r\nmalf_state1  : 0x%02X", CtrlData[CTL_NUM_2].Fault.byte1.malf_state1);
				fish_print("\r\nmalf_state2  : 0x%02X", CtrlData[CTL_NUM_2].Fault.byte2.malf_state2);
				fish_print("\r\nmalf_state3  : 0x%02X", CtrlData[CTL_NUM_2].Fault.byte3.malf_state3);
				for(i = 0; i < 5; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",3+i, CtrlData[CTL_NUM_2].Fault.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_RATED_MSG:
				fish_print("\r\n============[id 0x255]=================");
				fish_print("\r\nrated_voltage   : %.2fV", CtrlData[CTL_NUM_1].rated_param.rated_voltage/100.0);   //0.01V/bit
				fish_print("\r\nnominal_capacity: %dAH", CtrlData[CTL_NUM_1].rated_param.nominal_capacity);   //1A/bit
				fish_print("\r\nrated_power     : %.1fkwh", CtrlData[CTL_NUM_1].rated_param.rated_power/10.0);       //0.1kwh/bit
				for(i = 0; i < 4; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",4+i, CtrlData[CTL_NUM_1].rated_param.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_RATED_MSG:
				fish_print("\r\n============[id 0x256]=================");
				fish_print("\r\nrated_voltage   : %.2fV", CtrlData[CTL_NUM_2].rated_param.rated_voltage/100.0);   //0.01V/bit
				fish_print("\r\nnominal_capacity: %dAH", CtrlData[CTL_NUM_2].rated_param.nominal_capacity);   //1A/bit
				fish_print("\r\nrated_power     : %.1fkwh", CtrlData[CTL_NUM_2].rated_param.rated_power/10.0);       //0.1kwh/bit
				for(i = 0; i < 4; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",4+i, CtrlData[CTL_NUM_2].rated_param.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_BRAND_MSG:
				fish_print("\r\n============[id 0x21C]=================");
				fish_print("\r\nbrand_number   : %d", CtrlData[CTL_NUM_1].brand_materials.brand_number);
				fish_print("\r\nbat_materials  : %d", CtrlData[CTL_NUM_1].brand_materials.bat_materials);
				fish_print("\r\nbat_serial_num : %d", CtrlData[CTL_NUM_1].brand_materials.bat_serial_num);
				for(i = 0; i < 4; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",4+i, CtrlData[CTL_NUM_1].rated_param.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_2_BRAND_MSG:
				fish_print("\r\n============[id 0x21D]=================");
				fish_print("\r\nbrand_number   : %d", CtrlData[CTL_NUM_2].brand_materials.brand_number);
				fish_print("\r\nbat_materials  : %d", CtrlData[CTL_NUM_2].brand_materials.bat_materials);
				fish_print("\r\nbat_serial_num : %d", CtrlData[CTL_NUM_2].brand_materials.bat_serial_num);
				for(i = 0; i < 4; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",4+i, CtrlData[CTL_NUM_2].rated_param.reserve[i]);
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_BAT_1_INFO:
				fish_print("\r\n============[id 0x220]=================");
				fish_print("\r\npack_index0 : %d", CtrlData[CTL_NUM_1].bat_info.pack_index0);
				fish_print("\r\npack_total  : %d", CtrlData[CTL_NUM_1].bat_info.pack_total);
				fish_print("\r\ncell_total  : %d", CtrlData[CTL_NUM_1].bat_info.cell_total);
				fish_print("\r\nbat_volt    : %d", CtrlData[CTL_NUM_1].bat_info.bat_volt);
				fish_print("\r\n---------------------------------------");
				fish_print("\r\npack_index1        : %d", CtrlData[CTL_NUM_1].bat_info.pack_index1);
				fish_print("\r\ncell_volt_max_num  : %d", CtrlData[CTL_NUM_1].bat_info.cell_volt_max_num);
				fish_print("\r\ncell_volt_min_num  : %d", CtrlData[CTL_NUM_1].bat_info.cell_volt_min_num);
				fish_print("\r\ncell_volt_max      : %dmV", CtrlData[CTL_NUM_1].bat_info.cell_volt_max);
				fish_print("\r\ncell_volt_min      : %dmV", CtrlData[CTL_NUM_1].bat_info.cell_volt_min);
				fish_print("\r\n---------------------------------------");
				for(i = 0; i < CtrlData[CTL_NUM_1].bat_info.cell_total; i++)
				{
					fish_print("\r\npack_index: %d, bat_num: %d, bat_volt: %dmV", 2+i, \
																				  CtrlData[CTL_NUM_1].bat_info.cell_info[i].bat_num, \
																				  CtrlData[CTL_NUM_1].bat_info.cell_info[i].bat_volt);					
				}
				fish_print("\r\n=======================================\r\n");
				break;
				
			case CTRL_BAT_2_INFO:
				fish_print("\r\n============[id 0x221]=================");
				fish_print("\r\npack_index0 : %d", CtrlData[CTL_NUM_2].bat_info.pack_index0);
				fish_print("\r\npack_total  : %d", CtrlData[CTL_NUM_2].bat_info.pack_total);
				fish_print("\r\ncell_total  : %d", CtrlData[CTL_NUM_2].bat_info.cell_total);
				fish_print("\r\nbat_volt    : %d", CtrlData[CTL_NUM_2].bat_info.bat_volt);
				fish_print("\r\n---------------------------------------");
				fish_print("\r\npack_index1        : %d", CtrlData[CTL_NUM_2].bat_info.pack_index1);
				fish_print("\r\ncell_volt_max_num  : %d", CtrlData[CTL_NUM_2].bat_info.cell_volt_max_num);
				fish_print("\r\ncell_volt_min_num  : %d", CtrlData[CTL_NUM_2].bat_info.cell_volt_min_num);
				fish_print("\r\ncell_volt_max      : %dmV", CtrlData[CTL_NUM_2].bat_info.cell_volt_max);
				fish_print("\r\ncell_volt_min      : %dmV", CtrlData[CTL_NUM_2].bat_info.cell_volt_min);
				fish_print("\r\n---------------------------------------");
				for(i = 0; i < CtrlData[CTL_NUM_2].bat_info.cell_total; i++)
				{
					fish_print("\r\npack_index: %d, bat_num: %d, bat_volt: %dmV", 2+i, \
																				  CtrlData[CTL_NUM_2].bat_info.cell_info[i].bat_num, \
																				  CtrlData[CTL_NUM_2].bat_info.cell_info[i].bat_volt);
				}
				fish_print("\r\n=======================================\r\n");
				break;

			case CTRL_1_CHG_DSG:
				fish_print("\r\n============[id 0x90]=================");
				fish_print("\r\nchg_state : %d", CtrlData[CTL_NUM_1].chg_dsg.chg_state);
				fish_print("\r\ndsg_state : %d", CtrlData[CTL_NUM_1].chg_dsg.dsg_state);
				fish_print("\r\n======================================\r\n");
				break;

			case CTRL_2_CHG_DSG:
				fish_print("\r\n============[id 0x91]=================");
				fish_print("\r\nchg_state : %d", CtrlData[CTL_NUM_2].chg_dsg.chg_state);
				fish_print("\r\ndsg_state : %d", CtrlData[CTL_NUM_2].chg_dsg.dsg_state);
				fish_print("\r\n======================================\r\n");
				break;

//============================打印BMS上报信息========================================
            case BMS_1_STATE_MSG:
				fish_print("\r\n============[id 0x208]=================");
				fish_print("\r\nlow_temp_cell : %.1f℃", (BmsData[BMS_NUM_1].state.low_temp_cell/10.0 - 50));
				fish_print("\r\nlow_num       : %d#", BmsData[BMS_NUM_1].state.low_num);
				fish_print("\r\nhigh_temp_cell: %.1f℃", (BmsData[BMS_NUM_1].state.high_temp_cell/10.0 - 50));
				fish_print("\r\nhigh_num      : %d#", BmsData[BMS_NUM_1].state.high_num);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_STATE_MSG:
				fish_print("\r\n============[id 0x209]=================");
				fish_print("\r\nlow_temp_cell : %.1f℃", (BmsData[BMS_NUM_2].state.low_temp_cell/10.0 - 50));
				fish_print("\r\nlow_num       : %d#", BmsData[BMS_NUM_2].state.low_num);
				fish_print("\r\nhigh_temp_cell: %.1f℃", (BmsData[BMS_NUM_2].state.high_temp_cell/10.0 - 50));
				fish_print("\r\nhigh_num      : %d#", BmsData[BMS_NUM_2].state.high_num);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_DISP_MSG:
				fish_print("\r\n============[id 0x20C]=================");
				fish_print("\r\ncycle_num    : %d", BmsData[BMS_NUM_1].disp_msg.cycle_num);
				fish_print("\r\ncharge_times : %d", BmsData[BMS_NUM_1].disp_msg.charge_times);
				if (BmsData[BMS_NUM_1].disp_msg.rem_chg_time != 0xFFFF)
					fish_print("\r\nrem_chg_time : %d", BmsData[BMS_NUM_1].disp_msg.rem_chg_time);
				else
					fish_print("\r\nrem_chg_time : ---");
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_DISP_MSG:
				fish_print("\r\n============[id 0x20D]=================");
				fish_print("\r\ncycle_num    : %d", BmsData[BMS_NUM_2].disp_msg.cycle_num);
				fish_print("\r\ncharge_times : %d", BmsData[BMS_NUM_2].disp_msg.charge_times);
				if (BmsData[BMS_NUM_2].disp_msg.rem_chg_time != 0xFFFF)
					fish_print("\r\nrem_chg_time : %d", BmsData[BMS_NUM_2].disp_msg.rem_chg_time);
				else
					fish_print("\r\nrem_chg_time : ---");
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_CHG_DSG_MSG:
				fish_print("\r\n============[id 0x105]=================");
				fish_print("\r\nchg_state : %d", BmsData[BMS_NUM_1].c_d_msg.chg);
				fish_print("\r\ndsg_state : %d", BmsData[BMS_NUM_1].c_d_msg.dsg);
				fish_print("\r\nchg_curr  : %.2fA", BmsData[BMS_NUM_1].c_d_msg.chg_curr/100.0);
				fish_print("\r\ndsg_curr  : %.2fA", BmsData[BMS_NUM_1].c_d_msg.dsg_curr/100.0);
				fish_print("\r\nbat_volt  : %.2fV", BmsData[BMS_NUM_1].c_d_msg.bat_volt/100.0);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_CHG_DSG_MSG:
				fish_print("\r\n============[id 0x106]=================");
				fish_print("\r\nchg_state : %d", BmsData[BMS_NUM_2].c_d_msg.chg);
				fish_print("\r\ndsg_state : %d", BmsData[BMS_NUM_2].c_d_msg.dsg);
				fish_print("\r\nchg_curr  : %.2fA", BmsData[BMS_NUM_2].c_d_msg.chg_curr/100.0);
				fish_print("\r\ndsg_curr  : %.2fA", BmsData[BMS_NUM_2].c_d_msg.dsg_curr/100.0);
				fish_print("\r\nbat_volt  : %.2fV", BmsData[BMS_NUM_2].c_d_msg.bat_volt/100.0);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_TYPE_MSG:
				fish_print("\r\n============[id 0x210]=================");
				fish_print("\r\nmaterials  : %d", BmsData[BMS_NUM_1].bat_type.materials);
				fish_print("\r\ncell_total : %d", BmsData[BMS_NUM_1].bat_type.cell_total);
				fish_print("\r\ndsg_ratio  : %.1fC", BmsData[BMS_NUM_1].bat_type.dsg_ratio/10.0);
				fish_print("\r\nchg_ratio  : %.1fC", BmsData[BMS_NUM_1].bat_type.chg_ratio/10.0);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_TYPE_MSG:
				fish_print("\r\n============[id 0x211]=================");
				fish_print("\r\nmaterials  : %d", BmsData[BMS_NUM_2].bat_type.materials);
				fish_print("\r\ncell_total : %d", BmsData[BMS_NUM_2].bat_type.cell_total);
				fish_print("\r\ndsg_ratio  : %.1fC", BmsData[BMS_NUM_2].bat_type.dsg_ratio/10.0);
				fish_print("\r\nchg_ratio  : %.1fC", BmsData[BMS_NUM_2].bat_type.chg_ratio/10.0);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_SOC_MSG:
				fish_print("\r\n============[id 0x101]=================");
				fish_print("\r\nsoc      : %d%%", BmsData[BMS_NUM_1].soc_temp.soc);
				fish_print("\r\nfet_temp : %.1f℃", (BmsData[BMS_NUM_1].soc_temp.fet_temp/10.0 - 50));
				fish_print("\r\namb_temp : %.1f℃", (BmsData[BMS_NUM_1].soc_temp.amb_temp/10.0 - 50));
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_SOC_MSG:
				fish_print("\r\n============[id 0x102]=================");
				fish_print("\r\nsoc      : %d%%", BmsData[BMS_NUM_2].soc_temp.soc);
				fish_print("\r\nfet_temp : %.1f℃", (BmsData[BMS_NUM_2].soc_temp.fet_temp/10.0 - 50));
				fish_print("\r\namb_temp : %.1f℃", (BmsData[BMS_NUM_2].soc_temp.amb_temp/10.0 - 50));
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_RATED_MSG:
				fish_print("\r\n============[id 0x270]=================");
				fish_print("\r\nrated_voltage     : %.2fV", BmsData[BMS_NUM_1].rated_param.rated_voltage/100.0);
				fish_print("\r\nnominal_capacity  : %dAH", BmsData[BMS_NUM_1].rated_param.nominal_capacity);
				fish_print("\r\nresidual_capacity : %.2fAH", BmsData[BMS_NUM_1].rated_param.residual_capacity/100.0);
				fish_print("\r\nfull_chg_capacity : %.2fAH", BmsData[BMS_NUM_1].rated_param.full_chg_capacity/100.0);
				fish_print("\r\nsoh               : %d%%", BmsData[BMS_NUM_1].rated_param.soh);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_RATED_MSG:
				fish_print("\r\n============[id 0x271]=================");
				fish_print("\r\nrated_voltage     : %.2fV", BmsData[BMS_NUM_2].rated_param.rated_voltage/100.0);
				fish_print("\r\nnominal_capacity  : %dAH", BmsData[BMS_NUM_2].rated_param.nominal_capacity);
				fish_print("\r\nresidual_capacity : %.2fAH", BmsData[BMS_NUM_2].rated_param.residual_capacity/100.0);
				fish_print("\r\nfull_chg_capacity : %.2fAH", BmsData[BMS_NUM_2].rated_param.full_chg_capacity/100.0);
				fish_print("\r\nsoh               : %d%%", BmsData[BMS_NUM_2].rated_param.soh);
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_CHG_REQ_MSG:
				fish_print("\r\n============[id 0x214]=================");
				fish_print("\r\nchg_req_mode : %d", BmsData[BMS_NUM_1].chg_req.chg_req_mode);
				fish_print("\r\nchg_req_volt : %.2fV", BmsData[BMS_NUM_1].chg_req.chg_req_volt/100.0);
				fish_print("\r\nchg_req_curr : %.2fA", BmsData[BMS_NUM_1].chg_req.chg_req_curr/100.0);
				if(BmsData[BMS_NUM_1].chg_req.chg_req_temp != 0xFFFF)
					fish_print("\r\nchg_req_temp : %.1f℃", (BmsData[BMS_NUM_1].chg_req.chg_req_temp/10.0 - 50));	
				else
					fish_print("\r\nchg_req_temp : ---℃");	
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_CHG_REQ_MSG:
				fish_print("\r\n============[id 0x215]=================");
				fish_print("\r\nchg_req_mode : %d", BmsData[BMS_NUM_2].chg_req.chg_req_mode);
				fish_print("\r\nchg_req_volt : %.2fV", BmsData[BMS_NUM_2].chg_req.chg_req_volt/100.0);
				fish_print("\r\nchg_req_curr : %.2fA", BmsData[BMS_NUM_2].chg_req.chg_req_curr/100.0);
				if(BmsData[BMS_NUM_2].chg_req.chg_req_temp != 0xFFFF)
					fish_print("\r\nchg_req_temp : %.1f℃", (BmsData[BMS_NUM_2].chg_req.chg_req_temp/10.0 - 50));	
				else
					fish_print("\r\nchg_req_temp : ---℃");	
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_CHG_RATED_MSG:
				fish_print("\r\n============[id 0x251]=================");
				fish_print("\r\nmax_chg_sig_volt : %.2fV", BmsData[BMS_NUM_1].chg_rated.max_chg_sig_volt/100.0);
				fish_print("\r\nmax_chg_volt     : %.2fV", BmsData[BMS_NUM_1].chg_rated.max_chg_volt/100.0);
				fish_print("\r\nchg_req_curr     : %.2fA", BmsData[BMS_NUM_1].chg_rated.max_chg_curr/100.0);
				fish_print("\r\nmax_chg_temp     : %.1f℃", (BmsData[BMS_NUM_1].chg_rated.max_chg_temp/10.0 - 50));	
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_CHG_RATED_MSG:
				fish_print("\r\n============[id 0x252]=================");
				fish_print("\r\nmax_chg_sig_volt : %.2fV", BmsData[BMS_NUM_2].chg_rated.max_chg_sig_volt/100.0);
				fish_print("\r\nmax_chg_volt     : %.2fV", BmsData[BMS_NUM_2].chg_rated.max_chg_volt/100.0);
				fish_print("\r\nchg_req_curr     : %.2fA", BmsData[BMS_NUM_2].chg_rated.max_chg_curr/100.0);
				fish_print("\r\nmax_chg_temp     : %.1f℃", (BmsData[BMS_NUM_2].chg_rated.max_chg_temp/10.0 - 50));	
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_1_FAULT_MSG:
				fish_print("\r\n============[id 0x261]=================");
				fish_print("\r\nFault1: 0x%02X", BmsData[BMS_NUM_1].Fault.byte1.data);
				fish_print("\r\nFault2: 0x%02X", BmsData[BMS_NUM_1].Fault.byte2.data);
				/*for(i = 0; i < 6; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",2+i, BmsData[BMS_NUM_1].Fault.reserve[i]);*/
				fish_print("\r\n=======================================\r\n");
				break;

			case BMS_2_FAULT_MSG:
				fish_print("\r\n============[id 0x262]=================");
				fish_print("\r\nFault1: 0x%02X", BmsData[BMS_NUM_2].Fault.byte1.data);
				fish_print("\r\nFault2: 0x%02X", BmsData[BMS_NUM_2].Fault.byte2.data);
				/*for(i = 0; i < 6; i++)
					fish_print("\r\nreserve[%d]: 0x%02X",2+i, BmsData[BMS_NUM_2].Fault.reserve[i]);*/
				fish_print("\r\n=======================================\r\n");
				break;
				
			default:
				break;
		}
	}
#endif //#if 0
}
#endif //#ifndef  USE_MJ_MULTI_MODE 

/*
*********************************************************************************************************
*	函 数 名: test_voice_play
*	功能说明: 测试语音播放
*	形    参：addr -- 语音地址
*	返 回 值: 无
*********************************************************************************************************
*/
static void test_voice_play(uint8_t addr)
{
    if (fish_flag == SYS_MODE_PASS)
        return;
    fish_print("\r\n--- addr-->%02d ---\r\n", addr);
    
    if(GetN9300BusyState() == N9300_BUSY)
    {
        fish_print("正在播放语音，请稍等...\r\n");
        return;
    }
    
    OneWriteSendCmd(addr);
}

/*
*********************************************************************************************************
*	函 数 名: fish_check
*	功能说明: 判断输入命令
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void fish_check(uint8_t *buf)
{
    switch (buf[0])
    {
    case 'A':
        if((buf[1] =='T')&&(buf[2] =='+'))
        {
            ConfigParamParse(buf,sizeof(buf));
        }
        break;
    case '?':
        fish_menu();
        break;
    case 'r':
        fish_reset();
        break;
    case 'c':
        fish_check_stack();
        break;
    case 'f':
        if (strstr((char*)buf, "fish") != NULL)
            fish_entry();
        break;
    case 'q':
        fish_quit();
        break;
    case 's':
        #ifdef ENABLE_EXT_RS485
        config_rs485_output(buf[1]-'0');
        #endif
        #ifdef ENABLE_CAN_COMM
        config_can_output(buf[1]-'0');
        #endif
        break;
    case 'p':
        control_bat_box_lock(buf[1]-'0');
        break;
    case 'b':
        //config_lionBat_output(buf[1]-'0');
        config_can_test(buf[1]-'0');
        break;
    case 'i':
		if((buf[1] =='d')&&(buf[2] =='='))
        {
            #ifdef  USE_BH_LEASE_MODE 
            
            #endif
            
            #ifdef USE_MJ_MULTI_MODE
        	dbg_can_id(buf, sizeof(buf));
            #endif
		}
        break;
    case 'm':
        test_voice_play((buf[1]-'0')*10 + (buf[2]-'0'));
        break;
    case 'l':
        fish_list_var();
        break;
    case 'u':
        fish_default_init();
        break;
//    case '{':
//        fish_config((char*)buf);
//        break;
    default:
        break;
    }
}

#ifdef  USE_VIRTUAL_COM
void FishCommRecvProc(uint8_t rev_data)
{
    OS_ERR err;
    
    if (VirComRecvPtr == 0)
    {  
        VirtualCom_RX_BUF[VirComRecvPtr ++] = rev_data;
    }
    else
    {
        if (VirComRecvPtr < MAX_BUF_LEN)
        {
            VirtualCom_RX_BUF[VirComRecvPtr ++] = rev_data;
        }
        
        if((VirtualCom_RX_BUF[VirComRecvPtr - 1] == 0x0A)&&(VirtualCom_RX_BUF[VirComRecvPtr - 2] == 0x0D))
        {
            //if(fish_recv_sem)
            {
                OSSemPost(&fish_recv_sem,OS_OPT_POST_1,&err);
            }
        }
    }
}

//用于模拟串口采用中断方式接收，读取\r\n结尾的一帧数据   billy
static int fish_read_from_uart(uint8_t *buf)
{
    int recv_len = VirComRecvPtr;
    
    memcpy(buf,VirtualCom_RX_BUF,VirComRecvPtr);
    memset(VirtualCom_RX_BUF,0,MAX_BUF_LEN);
    VirComRecvPtr = 0;
    
    return recv_len;
}

#else //非模拟串口

#ifdef FISH_USE_INT_RX
static uint16_t FishRxCnt = 0;
void FishCommRecvProc(uint8_t rev_data)
{
    OS_ERR err;
    
    if (FishRxCnt == 0)
    {  
        fishcom.FishRxBuf[FishRxCnt ++] = rev_data;
    }
    else
    {
        if (FishRxCnt < MAX_BUF_LEN)
        {
            fishcom.FishRxBuf[FishRxCnt ++] = rev_data;
        }
        
        if((fishcom.FishRxBuf[FishRxCnt - 1] == 0x0A)&&(fishcom.FishRxBuf[FishRxCnt - 2] == 0x0D))
        {
            //if(fish_recv_sem)
            {
                OSSemPost(&fish_recv_sem,OS_OPT_POST_1,&err);
            }
        }
    }
}

//用于模拟串口采用中断方式接收，读取\r\n结尾的一帧数据   billy
static int fish_read_from_uart(uint8_t *buf)
{
    int recv_len = FishRxCnt;
    
    memcpy(buf,fishcom.FishRxBuf,FishRxCnt);
    memset(fishcom.FishRxBuf, 0, MAX_BUF_LEN);
    FishRxCnt = 0;
    
    return recv_len;
}
#endif //#ifdef  FISH_USE_INT_RX
#endif //#ifdef  USE_VIRTUAL_COM


/*
*********************************************************************************************************
*	函 数 名: fish_read_from_uart
*	功能说明: 从硬件串口读取数据
*	形    参：buf -- 指向接收缓存区地址
*	返 回 值: 返回接收数据长度
*********************************************************************************************************
*/
#ifdef FISH_USE_DMA_RX
static int fish_read_from_uart(uint8_t *buf)
{
    int recv_len = fishcom.rx_size;
    
    memcpy(buf,fishcom.FishRxBuf,recv_len);
    memset(fishcom.FishRxBuf,0,MAX_BUF_LEN);
    fishcom.rx_size = 0;
    
    return recv_len;
}
#endif

/*
*********************************************************************************************************
*	函 数 名: fish_read
*	功能说明: 以阻塞方式读取缓存区数据
*	形    参：buf -- 指向接收缓存区地址
*	返 回 值: 返回接收数据长度
*********************************************************************************************************
*/
static int fish_read(uint8_t *buf, uint32_t timeout)
{
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&fish_recv_sem,
              (OS_TICK      )timeout,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    if(OS_ERR_TIMEOUT == err)
        return 0;
    
    return fish_read_from_uart(buf);
}

/*
*********************************************************************************************************
*	函 数 名: FishTaskProc
*	功能说明: fish任务
*	形    参：p_arg --- 可选参数
*	返 回 值: 无
*********************************************************************************************************
*/
void FishTaskProc(void *p_arg)
{
    #ifndef  USE_SERIAL_TASK_PRINTF
    #ifdef ENABLE_IDWG
    OS_ERR err;
    #endif
    #endif
    (void)p_arg;
    
    //printf("\r\nfish task run!!!");
    
    for(;;)
    {
        if(fish_read(fish_rev_buf, 4) != 0)  //[20] 4*5=20ms
        {
            fish_check(fish_rev_buf);
            memset(fish_rev_buf, 0, sizeof(fish_rev_buf));
        }
        
        led_task_proc();
        
        #ifndef  USE_SERIAL_TASK_PRINTF
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )FISH_TASK_FEED_DOG,  /* 设置bit5 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: fish_create
*	功能说明: 创建fish任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void fish_create(void)
{
    uint8_t err;
    
    OSSemCreate ((OS_SEM    *)&fish_send_sem,
                 (CPU_CHAR  *)"fish_send_sem",
                 (OS_SEM_CTR )1,
                 (OS_ERR    *)&err);
    OSSemCreate ((OS_SEM    *)&fish_recv_sem,
                 (CPU_CHAR  *)"fish_recv_sem",
                 (OS_SEM_CTR )0,
                 (OS_ERR    *)&err);
    
    OSTaskCreate((OS_TCB     *)&FishTackTCB,
                 (CPU_CHAR   *)"FishTackTCB",
                 (OS_TASK_PTR ) FishTaskProc,
                 (void       *) 0,
                 (OS_PRIO     ) FISH_TASK_PRIO,
                 (CPU_STK    *)&FishTaskStk[0],
                 (CPU_STK_SIZE) FISH_STK_SIZE / 10,
                 (CPU_STK_SIZE) FISH_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err); 
}

