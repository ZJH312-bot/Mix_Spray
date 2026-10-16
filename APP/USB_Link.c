/**
  ******************************************************************************
  * @file    USB_Link.c
  * @brief   上下位机(USB CDC)连接异常检测 + 自动软重连  —— APP 层模块
  ******************************************************************************
  * 检测方式：判据①②只看 MCU 这边的 USB 设备栈状态；判据③用上位机反馈帧当心跳。
  *           全程不改动上报帧内容，也不把连接状态上报给上位机（连不上时本来也无从上报）。
  *
  * 判据① dev_state != USBD_STATE_CONFIGURED 持续 USB_LINK_OFFLINE_MS
  *       —— 拔线、PC 休眠、USB 复位、枚举失败。
  *       依据：拔线后主机停发 SOF，USB 外设 3ms 后产生 SUSP 中断，
  *             协议栈把 dev_state 置为 USBD_STATE_SUSPENDED，所以能识别出来。
  * 判据② CDC 类里的 TxState 持续非 0 超过 USB_LINK_TX_STUCK_MS
  *       —— 线还插着，但上位机软件关闭/卡死：CDC 的 IN 传输由主机驱动，
  *         上位机不读串口时，上一次上报就一直挂在 TxState 里没人取走。
  *         这里只"读"这个状态，不参与上报流程。
  * 判据③ 连续 USB_LINK_ACK_TIMEOUT_MS 收不到上位机反馈帧(0xFF)
  *       —— 上位机每收到一帧上报就回一帧反馈帧，是可靠心跳；命令帧只在操作时下发，当不了心跳。
  *         见 USB_LINK_ACK_CHECK。
  *
  * 处理方式：软重连 —— 不阻塞主循环、不复位 MCU、不影响电机 PWM
  *   USBD_Stop()  → 内部 HAL_PCD_Stop() 清 BCDR.DPPU，主机认为 D+ 已拔掉
  *   等 DETACH_MS → 让主机完成"设备移除"处理（Windows 需要这个时间）
  *   USBD_Start() → 置回 BCDR.DPPU 并整体重写 CNTR
  *                  (顺带清掉 SUSP 中断里置的 FSUSP/LPMODE，否则挂起态起不来)
  ******************************************************************************
  */

#include "USB_Link.h"
#include "usb.h"            /* USB_Rx_Flush() / system_state_data */
#include "Motor_Control.h"  /* Motor_MIX_SPEED_Set / Motor_SPRAY_SPEED_Set */
#include "usb_device.h"     /* hUsbDeviceFS */
#include "usbd_core.h"      /* USBD_Stop / USBD_Start */
#include "usbd_cdc.h"       /* USBD_CDC_HandleTypeDef（读取 TxState） */

/* 单实例：零初始化即可，具体初值由 USB_Link_Init() 设定 */
USB_Link_t usb_link;

/**
 * @brief  读一下 CDC 上一次上报是否还没被主机取走
 * @retval 1 = 卡住(主机没取)  0 = 已被取走 / 暂无可判状态
 * @note   只读状态，不改动帧层的收发流程
 */
static uint8_t USB_Link_IsTxStuck(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL) return 0U;   /* 类实例未建立(未配置) → 交给判据①处理 */
    return (hcdc->TxState != 0U) ? 1U : 0U;
}

/* ======================= 状态指示 LED =======================
   PA0(LED1)，高电平点亮。三种错误状态用闪法区分：
     ①USB 未连接(拔线/PC休眠/重连断开窗口)     → 熄灭
     ②已连上但上报没人取(上位机软件关了/卡死) → 快闪
     ③已连上但收不到反馈帧(上位机应用层不动)   → 慢闪
   正常态不写这个引脚：灯由 usb.c 收到命令帧时点亮(常亮)。
   注意：不要在判据分支里另写 PA0，否则会把闪烁“钉死”。
   =========================================================== */
#define USB_LINK_LED_PORT       GPIOA
#define USB_LINK_LED_PIN        GPIO_PIN_0
#define USB_LINK_LED_SLOW_MS     500U   /* 慢闪：500ms 翻转一次 ≈ 1Hz */
#define USB_LINK_LED_FAST_MS     100U   /* 快闪：100ms 翻转一次 ≈ 5Hz */

#define USB_LINK_LED_ON()   HAL_GPIO_WritePin(USB_LINK_LED_PORT, USB_LINK_LED_PIN, GPIO_PIN_SET)
#define USB_LINK_LED_OFF()  HAL_GPIO_WritePin(USB_LINK_LED_PORT, USB_LINK_LED_PIN, GPIO_PIN_RESET)

/**
 * @brief  状态指示 LED：按 link->err_code 决定 熄灭 / 快闪 / 慢闪 / 不干预
 * @note   非阻塞，按 led_tick 翻转；每个主循环周期调一次
 */
static void USB_Link_LedUpdate(USB_Link_t *link, uint32_t now)
{
    uint32_t period;

    /* ① USB 未连接（含重连断开窗口）：直接熄灭，不等判据超时 */
    if ((link->err_code == USB_LINK_ERR_NO_USB)
        || (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED))
    {
        USB_LINK_LED_OFF();
        link->led_tick = now;
        return;
    }

    /* 正常：不碰这个引脚（常亮由 usb.c 收到命令帧时点亮） */
    if (link->err_code == USB_LINK_ERR_NONE)
    {
        link->led_tick = now;
        return;
    }

    /* ② 上报没人取 → 快闪    ③ 收不到反馈帧 → 慢闪 */
    period = (link->err_code == USB_LINK_ERR_TX_STUCK) ? USB_LINK_LED_FAST_MS
                                                       : USB_LINK_LED_SLOW_MS;

    if ((now - link->led_tick) >= period)
    {
        link->led_tick = now;
        HAL_GPIO_TogglePin(USB_LINK_LED_PORT, USB_LINK_LED_PIN);
    }
}

/**
 * @brief  记录一次连接异常（自动重连 / 只检测 两种模式共用）
 * @note   只在"异常开始"时累加一次 err_cnt；异常持续期间重复调用是幂等的。
 *         本函数不碰 system_state_data，也不碰 USB 设备栈。
 */
static void USB_Link_LogAbnormal(USB_Link_t *link)
{
    link->state = USB_LINK_STATE_OFFLINE;
    if (link->abn_logged == 0U)
    {
        link->abn_logged = 1U;
        link->err_cnt++;
    }
}

/**
 * @brief  记一帧合法帧（命令帧/反馈帧都算），用于"通讯恢复后连续几帧才恢复电机"的判据
 * @note   两帧间隔超过 USB_LINK_FRAME_GAP_MS 视为不连续，重新计数
 */
static void USB_Link_CountGoodFrame(USB_Link_t *link)
{
    uint32_t now = HAL_GetTick();

    if ((now - link->good_frame_tick) > USB_LINK_FRAME_GAP_MS)
    {
        link->good_frame_cnt = 0;     /* 间隔太久 → 重新数 */
    }
    link->good_frame_tick = now;

    if (link->good_frame_cnt < 0xFFU) link->good_frame_cnt++;
}

/**
 * @brief  失联安全停机：先记住停机前的开关位，再把电机全部停掉
 * @note   记下的开关位用于通讯恢复后自动写回（见 USB_Link_RestoreMotor）
 */
static void USB_Link_StopMotor(USB_Link_t *link)
{
    if (link->motor_stopped == 0U)
    {
        link->motor_saved_ctrl = system_state_data.Motor_Control;   /* 记住停机前的状态 */
        link->motor_stopped    = 1U;
        link->good_frame_cnt   = 0U;    /* 从停机这一刻开始重新数帧 */
    }

    system_state_data.Motor_Control     = 0;
    system_state_data.spray_motor_speed = 0;
    system_state_data.mix_motor_speed   = 0;
}

#if (USB_LINK_RESTORE_FRAMES > 0U)
/**
 * @brief  把停机前记下的开关位写回，电机自动回到停机前的设定
 * @note   速度不回写(保持 0)，让限流状态机缓升，相当于软启动
 */
static void USB_Link_RestoreMotor(USB_Link_t *link)
{
    system_state_data.Motor_Control = link->motor_saved_ctrl;

    link->motor_stopped  = 0U;
    link->good_frame_cnt = 0U;
}
#endif

/**
 * @brief  收到上位机反馈帧(0xFF)时调用（usb.c 的 USB_Rx_Parse 内部调用）
 * @note   上位机"每收到一帧下位机上报就回一帧反馈帧"，所以这就是链路心跳。
 *         只记时间戳，不解析帧内容。
 */
void USB_Link_NotifyAckFrame(USB_Link_t *link)
{
    link->ack_tick = HAL_GetTick();
    link->ack_seen = 1;

    USB_Link_CountGoodFrame(link);   /* 反馈帧也是合法帧，参与恢复电机的连续帧计数 */
}

/**
 * @brief  收到上位机命令帧(0x00，校验通过)时调用（usb.c 的 USB_Rx_Parse 内部调用）
 * @note   参与"连续几帧合法帧才恢复电机"的计数；
 *         若故障已排除后又收到命令帧，说明上位机已经在主动控制，
 *         就取消自动写回，以免覆盖上位机刚下发的设定。
 */
void USB_Link_NotifyCmdFrame(USB_Link_t *link)
{
    USB_Link_CountGoodFrame(link);

    if ((link->motor_stopped != 0U) && (link->err_code == USB_LINK_ERR_NONE))
    {
        link->motor_stopped = 0U;    /* 上位机已接管，不再自动写回 */
    }
}

/**
 * @brief  初始化连接检测（放在 MX_USB_DEVICE_Init() 之后调用）
 */
void USB_Link_Init(USB_Link_t *link)
{
    uint32_t now = HAL_GetTick();

    link->state           = USB_LINK_STATE_OFFLINE;
    link->sub_state       = USB_LINK_SUB_IDLE;
    link->give_up         = 0;
    link->abn_logged      = 0;
    link->ack_seen        = 0;
    link->err_code        = USB_LINK_ERR_NONE;
    link->motor_stopped   = 0;
    link->motor_saved_ctrl = 0;
    link->good_frame_cnt  = 0;
    link->init_tick       = now;
    link->sub_tick        = now;
    link->offline_tick    = 0;
    link->stuck_tick      = 0;
    link->healthy_tick    = 0;
    link->ack_tick        = now;
    link->good_frame_tick = now;
    link->led_tick        = now;
    link->err_cnt         = 0;
    link->reconnect_cnt   = 0;
    link->reconnect_total = 0;
}

/* 以下两个函数只在"允许自动重连"时才需要，关掉时用 #if 排除，避免 Keil 报
   #177-D "function declared but never referenced" 警告 */
#if (USB_LINK_AUTO_RECONNECT)

/**
 * @brief  清掉一次重连前遗留的判定时间戳
 */
static void USB_Link_ClearCounters(USB_Link_t *link)
{
    link->offline_tick = 0;
    link->stuck_tick   = 0;
    link->healthy_tick = 0;
    link->ack_tick     = HAL_GetTick();   /* 重连后重新开始等反馈帧，避免立刻再触发 */
}

/**
 * @brief  启动一次软重连（只断开；重挂载由 USB_Link_Process 分阶段完成，不阻塞主循环）
 */
static void USB_Link_Reconnect(USB_Link_t *link)
{
    /* 重连次数用尽：放弃自动重连。
       之后只要硬件重新枚举(插拔线 / PC 唤醒)使 dev_state 回到 CONFIGURED，
       上报能被主机正常取走并稳定 USB_LINK_HEALTHY_MS 后会自动归还次数，不需要重新上电。 */
    if (link->reconnect_cnt >= USB_LINK_RETRY_MAX)
    {
        link->give_up = 1;
        link->state   = USB_LINK_STATE_OFFLINE;
#if (USB_LINK_GIVEUP_STOP_MOTOR)
        USB_Link_StopMotor(link);
#endif
        return;
    }

    link->reconnect_cnt++;
    link->reconnect_total++;

    /* 停止 USB 设备栈：内部 HAL_PCD_Stop() 会清 BCDR.DPPU，等效把 D+ 拔掉。
       注意 USBD_Stop() 不会改 dev_state，这里必须手工置为 DEFAULT：
       否则 usb.c 的上报路径仍以为"已连接"，而此刻 CDC 类实例已被释放
       (pClassData = NULL)，一旦发送就会踩空指针。 */
    USBD_Stop(&hUsbDeviceFS);
    hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;

    /* 复位本模块的判定状态 */
    link->state = USB_LINK_STATE_OFFLINE;
    USB_Link_ClearCounters(link);

    /* 丢弃接收缓冲里的半帧/旧帧，避免重连后被当成新命令 */
    USB_Rx_Flush();

    link->sub_state = USB_LINK_SUB_DETACH;
    link->sub_tick  = HAL_GetTick();
}

#endif /* USB_LINK_AUTO_RECONNECT */

/**
 * @brief  连接检测与软重连状态机，主循环里周期调用
 * @note   非阻塞：整个重连过程不占用主循环，电流限流等实时任务不受影响
 */
void USB_Link_Process(USB_Link_t *link)
{
    uint32_t now = HAL_GetTick();

    /* ---------- 开机宽限：等主机完成第一次枚举，避免一上电就误判 ---------- */
    if ((now - link->init_tick) < USB_LINK_STARTUP_GRACE_MS)
    {
        return;
    }

    /* 状态指示 LED（宽限期结束后才开始指示） */
    USB_Link_LedUpdate(link, now);

    /* ---------- 软重连的等待阶段 ---------- */
    if (link->sub_state == USB_LINK_SUB_DETACH)
    {
        if ((now - link->sub_tick) >= USB_LINK_DETACH_MS)
        {
            USBD_Start(&hUsbDeviceFS);   /* 恢复 D+ 上拉 + 使能中断 → 主机重新枚举 */
            link->sub_state = USB_LINK_SUB_ATTACH;
            link->sub_tick  = now;
        }
        return;
    }

    if (link->sub_state == USB_LINK_SUB_ATTACH)
    {
        if ((now - link->sub_tick) >= USB_LINK_ATTACH_MS)
        {
            link->sub_state = USB_LINK_SUB_IDLE;
            link->sub_tick  = now;
        }
        return;
    }

    /* ---------- 通讯恢复：连续收到足够多帧合法帧 → 写回停机前的电机状态 ---------- */
#if (USB_LINK_RESTORE_FRAMES > 0U)
    if ((link->motor_stopped != 0U) && (link->good_frame_cnt >= USB_LINK_RESTORE_FRAMES))
    {
        USB_Link_RestoreMotor(link);
    }
#endif

    /* ---------- 连接异常判据 ---------- */

    /* 已放弃自动重连：只等硬件重新枚举后由"上报被正常取走"自动恢复 */
    if (link->give_up)
    {
        return;
    }

    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        link->state        = USB_LINK_STATE_ONLINE;
        link->offline_tick = 0;
        link->abn_logged   = 0;     /* 链路正常，允许下次异常重新计数 */
        link->err_code     = USB_LINK_ERR_NONE;   /* 先按正常算，下面判据命中再改 */

        /* 判据②：上报一直没被主机取走 → 上位机软件关闭/卡死 */
        if (USB_Link_IsTxStuck())
        {
            link->healthy_tick = 0;
            if (link->stuck_tick == 0U)
            {
                link->stuck_tick = (now == 0U) ? 1U : now;   /* 0 作哨兵值 */
            }
            else if ((now - link->stuck_tick) >= USB_LINK_TX_STUCK_MS)
            {
                USB_Link_LogAbnormal(link);
#if (USB_LINK_AUTO_RECONNECT)
                USB_Link_Reconnect(link);
#endif
            link->err_code = USB_LINK_ERR_TX_STUCK;   /* LED：快闪 */
            USB_Link_StopMotor(link);                 /* 失联安全停机(记住停机前状态) */
            }
            return;
        }

        /* 判据③：上位机反馈帧心跳断了（曾收到过反馈帧才启用，避免上位机不支持该机制时误报） */
#if (USB_LINK_ACK_CHECK)
//        if (link->ack_seen != 0U
//            && (now - link->ack_tick) >= USB_LINK_ACK_TIMEOUT_MS)
				if ( (now - link->ack_tick) >= USB_LINK_ACK_TIMEOUT_MS)
        {
            USB_Link_LogAbnormal(link);
            link->err_code = USB_LINK_ERR_NO_ACK;     /* LED：慢闪 */
            USB_Link_StopMotor(link);                 /* 失联安全停机(记住停机前状态) */
#if (USB_LINK_AUTO_RECONNECT)
            USB_Link_Reconnect(link);
#endif
            return;
        }
#endif

        /* 上报能被正常取走：稳定一段时间后归还重连次数（说明链路真的恢复了） */
        link->stuck_tick = 0;
        if (link->healthy_tick == 0U)
        {
            link->healthy_tick = (now == 0U) ? 1U : now;
        }
        else if ((now - link->healthy_tick) >= USB_LINK_HEALTHY_MS)
        {
            link->healthy_tick  = 0;
            link->reconnect_cnt = 0;
            link->give_up       = 0;
        }
    }
    else
    {
        /* 判据①：USB 未配置 —— 拔线 / PC 休眠 / 枚举失败 */
        link->state        = USB_LINK_STATE_OFFLINE;
        link->err_code     = USB_LINK_ERR_NO_USB;      /* LED：立即熄灭 */
        link->stuck_tick   = 0;
        link->healthy_tick = 0;

        if (link->offline_tick == 0U)
        {
            link->offline_tick = (now == 0U) ? 1U : now;  /* 0 作哨兵值，避开 tick=0 */
        }
        else if ((now - link->offline_tick) >= USB_LINK_OFFLINE_MS)
        {
            USB_Link_LogAbnormal(link);
            USB_Link_StopMotor(link);                 /* 失联安全停机(记住停机前状态) */
#if (USB_LINK_AUTO_RECONNECT)
            USB_Link_Reconnect(link);
#endif
        }
    }
}
