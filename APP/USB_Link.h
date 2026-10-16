/**
  ******************************************************************************
  * @file    USB_Link.h
  * @brief   上下位机(USB CDC)连接异常检测 + 自动软重连  —— APP 层模块
  * @note    分层说明：
  *           usbd_cdc_if.c  底层，只收/发字节，不做任何处理（本模块不改动它）
  *           usb.c          APP 帧层，组帧/解析（本模块不改动它的收发流程）
  *           USB_Link.c     本模块，只读 USB 设备栈自身的状态判断链路是否正常
  *
  * @note    不改动上报帧内容，也不把连接状态上报给上位机（连不上时本来也无从上报）。
  *          判据①②只看 MCU 这边的 USB 设备栈状态；判据③用上位机反馈帧(0xFF)当心跳，
  *          因为命令帧只在操作时下发，只有"每收到一帧上报就回一帧"的反馈帧才是稳定心跳。
  *
  * @note    状态指示 LED（PA0/LED1）三种错误状态用闪法区分：
  *            ①USB 未连接(拔线/PC休眠/重连断开窗口)     → 熄灭
  *            ②已连上但上报没人取(上位机软件关了/卡死) → 快闪 5Hz
  *            ③已连上但收不到反馈帧(上位机应用层不动)   → 慢闪 1Hz
  *          正常态不写这个引脚：灯由 usb.c 收到命令帧时点亮(常亮)。闪灯参数在 USB_Link.c 顶部。
  *
  * @note    失联停机后会自动恢复：停机时记住 Motor_Control 的开关位，通讯恢复后
  *          连续收到 USB_LINK_RESTORE_FRAMES 帧合法帧就写回，电机回到停机前的设定
  *          （速度不回写，由限流缓升，相当于软启动）。
  *          若故障排除后上位机又下发了命令帧，则取消自动写回，以上位机为准。
  ******************************************************************************
  */

#ifndef __USB_LINK_H__
#define __USB_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================
 *                        可调参数
 * ============================================================ */

/* 判据①：USB 处于"未配置"状态持续多久，判定为连接异常
   —— 覆盖：拔线、PC 休眠、USB 复位、枚举失败
   注意：别调得太小(建议 >=300ms)，否则开机正常枚举过程会被误判 */
#define USB_LINK_OFFLINE_MS        1000U

/* 判据②：上报数据持续多久没被主机取走，判定为连接异常
   —— 覆盖：线还插着但上位机软件关闭/卡死、驱动挂住
   依据：CDC 的 IN 传输由主机驱动，上位机不读串口时 CDC 类的 TxState 会一直非 0
         (上一次上报还挂在端点里没人取)。上报周期 1s，故 5000ms ≈ 连续5次没人取 */
#define USB_LINK_TX_STUCK_MS       5000U

/* 开机宽限：上电后这么久内不做判定（等主机完成第一次枚举） */
#define USB_LINK_STARTUP_GRACE_MS  3000U

/* 恢复正常判定：上报能被主机正常取走并持续这么久，才认为链路真恢复(归还重连次数)
   避免"上位机软件一直关着"时无限重连 */
#define USB_LINK_HEALTHY_MS        5000U

/* 判据③：上位机反馈帧(0xFF)心跳。
   上位机每收到一帧下位机上报就回一帧反馈帧，所以"有没有回"直接反映上位机应用层在不在正常工作。
   一旦收到过反馈帧(说明上位机支持该机制)，之后就一直要求有反馈：
   连续 USB_LINK_ACK_TIMEOUT_MS 收不到 → 判连接异常。
   上报周期 1s，所以 5000ms = 允许连丢 5 帧。
   1 = 启用；0 = 关闭该判据 */
#define USB_LINK_ACK_CHECK         1U
#define USB_LINK_ACK_TIMEOUT_MS 5000U

/* 是否允许自动软重连：
   1 = 检测到异常后自动软重连（会让本机的 COM 口短暂消失再重现，上位机必须重新打开串口）
   0 = 只检测不动作：只更新 usb_link.state / err_cnt，供本地指示与调试
       —— 上位机还没实现"断线后自动重连"时建议用这个
   注意：无论哪种模式，都不会动 system_state_data，也不复位 MCU，
         电机按原设定继续运行（本地电流限流照常生效）。 */
#define USB_LINK_AUTO_RECONNECT       0U

/* 软重连参数 */
#define USB_LINK_DETACH_MS          500U  /* D+ 断开保持时间(给主机识别"设备移除") */
#define USB_LINK_ATTACH_MS          200U  /* D+ 接回后等待重新枚举的时间 */
#define USB_LINK_RETRY_MAX            5U  /* 连续重连次数上限，超过则放弃自动重连 */

/* 重连次数用尽后是否把电机置 0（失联安全停机）：
   0 = 保持失联前的运行状态（电流限流仍在本地生效）
   1 = 立即停机 */
#define USB_LINK_GIVEUP_STOP_MOTOR    0U

/* 失联停机后的自动恢复：
   停机时记住 Motor_Control 的开关位；通讯恢复后连续收到 USB_LINK_RESTORE_FRAMES
   帧合法帧(命令帧/反馈帧都算)，就把记下的开关位写回，电机自动回到停机前的设定。
   两帧间隔超过 USB_LINK_FRAME_GAP_MS 视为不连续，重新计数(反馈帧 1s 一帧，给 2s 容错)。
   USB_LINK_RESTORE_FRAMES = 0 则关闭自动恢复(停机后必须由上位机重新下发指令) */
#define USB_LINK_RESTORE_FRAMES     3U
#define USB_LINK_FRAME_GAP_MS    2000U

/* ============================================================
 *                        状态定义
 * ============================================================ */
#define USB_LINK_STATE_OFFLINE   0U   /* 连接异常 */
#define USB_LINK_STATE_ONLINE    1U   /* 连接正常(USB已枚举) */

/* 软重连子状态（结构体内部使用） */
#define USB_LINK_SUB_IDLE        0U   /* 正常，执行断联判据 */
#define USB_LINK_SUB_DETACH      1U   /* D+ 已断开，等待 */
#define USB_LINK_SUB_ATTACH      2U   /* D+ 已接回，等待重新枚举 */

/* 当前生效的异常类型（三种错误状态用 LED 闪法区分） */
#define USB_LINK_ERR_NONE        0U   /* 无异常：灯不干预，由 usb.c 收到命令帧时点亮(常亮) */
#define USB_LINK_ERR_NO_USB      1U   /* ① USB 未连接：拔线/PC休眠/枚举失败 → 灯熄灭 */
#define USB_LINK_ERR_TX_STUCK    2U   /* ② 上报没人取：上位机软件关闭/卡死  → 灯快闪 */
#define USB_LINK_ERR_NO_ACK      3U   /* ③ 收不到反馈帧：上位机应用层不动 → 灯慢闪 */

/* ============================================================
 *                        对象定义
 * ============================================================ */
typedef struct
{
    /* --- 链路状态 --- */
    uint8_t  state;             /* USB_LINK_STATE_ONLINE / USB_LINK_STATE_OFFLINE */
    uint8_t  sub_state;         /* 软重连子状态 USB_LINK_SUB_xxx */
    uint8_t  give_up;           /* 1 = 重连次数用尽，已放弃自动重连 */
    uint8_t  abn_logged;        /* 本轮异常是否已计数(防同一轮重复累加 err_cnt) */
    uint8_t  ack_seen;          /* 是否收到过上位机反馈帧(用来识别上位机是否支持该机制) */
    uint8_t  err_code;          /* 当前生效的异常类型 USB_LINK_ERR_xxx（用于 LED 指示） */
    uint8_t  motor_stopped;     /* 1 = 已因失联停机，等通讯恢复后自动写回 */
    uint8_t  motor_saved_ctrl;  /* 停机前的 Motor_Control 开关位 */
    uint8_t  good_frame_cnt;    /* 连续收到的合法帧数(命令帧/反馈帧) */

    /* --- 时间戳 --- */
    uint32_t init_tick;         /* 初始化时刻(用于开机宽限) */
    uint32_t sub_tick;          /* 子状态进入时刻 */
    uint32_t offline_tick;      /* 判定为离线的起始时刻(0=在线) */
    uint32_t stuck_tick;        /* 上报开始卡住的时刻(0=未被卡住) */
    uint32_t healthy_tick;      /* 上报开始正常的时刻(0=尚未正常) */
    uint32_t ack_tick;          /* 最近一次收到上位机反馈帧的时刻(心跳) */
    uint32_t led_tick;          /* LED 翻转节拍 */
    uint32_t good_frame_tick;   /* 最近一帧合法帧的时刻(用于判断帧是否连续) */

    /* --- 统计(调试用) --- */
    uint32_t err_cnt;           /* 累计判定出的连接异常次数 */
    uint32_t reconnect_cnt;     /* 本轮已用掉的重连次数 */
    uint32_t reconnect_total;   /* 开机以来累计重连次数 */
} USB_Link_t;

/* 单实例（定义在 USB_Link.c） */
extern USB_Link_t usb_link;

/* ============================================================
 *                        对外接口
 * @note  判定样本来自 USB 设备栈自身的状态(dev_state 与 CDC 类的 TxState)，
 *        加上帧层通知的"收到合法帧"；全部在主循环上下文调用（不在中断里），
 *        故结构体不用 volatile。
 * ============================================================ */
void USB_Link_Init(USB_Link_t *link);           /* 放在 MX_USB_DEVICE_Init() 之后 */
void USB_Link_Process(USB_Link_t *link);        /* 主循环里周期调用（非阻塞） */
void USB_Link_NotifyAckFrame(USB_Link_t *link); /* usb.c 解析到上位机反馈帧(0xFF)时调用 */
void USB_Link_NotifyCmdFrame(USB_Link_t *link); /* usb.c 解析到上位机命令帧(0x00)时调用 */

/* 状态查询（也可直接读结构体字段） */
#define USB_Link_IsOnline(link)        ((link)->state == USB_LINK_STATE_ONLINE)
#define USB_Link_IsReconnecting(link)  ((link)->sub_state != USB_LINK_SUB_IDLE)
#define USB_Link_IsGiveUp(link)        ((link)->give_up != 0U)

#ifdef __cplusplus
}
#endif
#endif /* __USB_LINK_H__ */
