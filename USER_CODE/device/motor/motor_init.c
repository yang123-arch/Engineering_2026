//=====================================================================================
//                                    motor_init[电机初始化]
//
//      驱动注册 / 设备声明 / 型号参数 / 分组。新增一个电机只改本文件：
//        1. g_motor_devices[] 里加一行（名字、种类、总线、收发 ID、所属组）
//        2. MOTOR_IDX_xxx 里加一个下标
//        3. 若该型号的额定值需要收窄，在 s_motor_params[] 里加一行
//      框架、驱动都不需要动。
//=====================================================================================

#include "motor_init.h"
#include "Define.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DM_read.h"
#include "Motor_LINGKONG_Driver.h"
#include "motor_core.h"

/*===|==================== 一、设备声明 ====================|===*/

/**
 * @brief 全部电机设备表
 * @note  group 用 motor_group_id_t 直接声明，motor_init 会据此自动入组，
 *        不需要另外维护"设备 -> 组"的映射表。
 *        tx_id / rx_id 必须与 Define.h 的宏一致；一帧多机驱动按 rx_id
 * 自己推槽位。
 *
 *        各组电机的声明模板：
 *
 *  底盘 3508 x4（大疆 0x200/0x1FF 双帧，反馈 0x201~0x208）：
 *    {"chassis_m1", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS, &hfdcan1,
 * 0x200, 0x201},
 *    {"chassis_m2", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS, &hfdcan1,
 * 0x200, 0x202},
 *    {"chassis_m3", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS, &hfdcan1,
 * 0x200, 0x203},
 *    {"chassis_m4", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS, &hfdcan1,
 * 0x200, 0x204},
 *    {"chassis_m5", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS, &hfdcan1,
 * 0x1FF, 0x205},
 *
 *  达妙单机（一条报文一个电机）：
 *    {"gimbal_yaw", MOTOR_TYPE_DM_8006, MOTOR_GROUP_GIMBAL, &hfdcan2, 0x01,
 * 0x101},
 *
 *  达妙一拖四（反馈 0x110~0x113 对应通道 0~3）：
 *    {"leg1_0", MOTOR_TYPE_DM_1TO4, MOTOR_GROUP_CHASSIS, &hfdcan1, 0x100,
 * 0x110},
 *
 *  凌空（一帧一个电机，ID 形如 0x140+ID）：
 *    {"gimbal_j7", MOTOR_TYPE_LK_MG6010, MOTOR_GROUP_GIMBAL, &hfdcan3, 0x147,
 * 0x147},
 */

/* DJI 侧仍用兼容型号 DJI_LEGACY：PID 输出是 ±16384 电调原始值，驱动内部
   ±20 <-> ±16384 恒等映射，与旧版 set_channel_currents 行为逐位一致。
   达妙侧用具体型号：q/v/t 量程等于电机寄存器（Motor_DMAIAO_Type.h），
   set_value 收 N·m，data->Torque 也是 N·m。Chassis.c / Gimbal.c 的 PID 参数
   按 MOTOR_DM_DJI_K_X 从旧工程整定值换算过来，输出和用 DM_DJI 时一致。
   要用 Motor_DM_read 读寄存器时，把要读的行改成 MOTOR_TYPE_DM_READ。*/

motor_device_t g_motor_devices[] = {
    /* name      type_id            group                hfdcan    tx_id
     * rx_id
     */
    {"chassis_1", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS,
     &Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID,
     Chassis_Motor1_Feedback_CAN_ID},
    {"chassis_2", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS,
     &Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID,
     Chassis_Motor2_Feedback_CAN_ID},
    {"chassis_3", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS,
     &Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID,
     Chassis_Motor3_Feedback_CAN_ID},
    {"chassis_4", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_CHASSIS,
     &Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID,
     Chassis_Motor4_Feedback_CAN_ID},

    {"chassis_leg1", MOTOR_TYPE_DM_8009P, MOTOR_GROUP_CHASSIS,
     &Chassis_Leg1_CAN, Chassis_Leg1_Send_CAN_ID, Chassis_Leg1_Feedback_CAN_ID},
    {"chassis_leg2", MOTOR_TYPE_DM_8009P, MOTOR_GROUP_CHASSIS,
     &Chassis_Leg2_CAN, Chassis_Leg2_Send_CAN_ID, Chassis_Leg2_Feedback_CAN_ID},

    {"gimbal_yaw", MOTOR_TYPE_DM_8006, MOTOR_GROUP_GIMBAL, &Gimbal_Yaw_CAN,
     Gimbal_Yaw_Send_CAN_ID, Gimbal_Yaw_Feedback_CAN_ID},
    {"gimbal_pitch1", MOTOR_TYPE_DM_8009, MOTOR_GROUP_GIMBAL,
     &Gimbal_Pitch1_CAN, Gimbal_Pitch1_Send_CAN_ID,
     Gimbal_Pitch1_Feedback_CAN_ID},
    {"gimbal_pitch2", MOTOR_TYPE_DM_8009, MOTOR_GROUP_GIMBAL,
     &Gimbal_Pitch2_CAN, Gimbal_Pitch2_Send_CAN_ID,
     Gimbal_Pitch2_Feedback_CAN_ID},
    {"camera_y", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_GIMBAL, &Camera_Y_CAN,
     Gimbal_Camera_Y_Send_CAN_ID, Gimbal_Camera_Y_Feedback_CAN_ID},
    {"camera_x", MOTOR_TYPE_DJI_LEGACY, MOTOR_GROUP_GIMBAL, &Camera_X_CAN,
     Gimbal_Camera_X_Send_CAN_ID, Gimbal_Camera_X_Feedback_CAN_ID},
    {"gimbal_j4", MOTOR_TYPE_DM_4310, MOTOR_GROUP_GIMBAL, &Gimbal_J4_CAN,
     Gimbal_J4_Send_CAN_ID, Gimbal_J4_Feedback_CAN_ID},
    {"gimbal_j5", MOTOR_TYPE_DM_4340, MOTOR_GROUP_GIMBAL, &Gimbal_J5_CAN,
     Gimbal_J5_Send_CAN_ID, Gimbal_J5_Feedback_CAN_ID},
    {"gimbal_j6", MOTOR_TYPE_DM_4310, MOTOR_GROUP_GIMBAL, &Gimbal_J6_CAN,
     Gimbal_J6_Send_CAN_ID, Gimbal_J6_Feedback_CAN_ID},
    {"gimbal_j7", MOTOR_TYPE_LK_MG4010E, MOTOR_GROUP_GIMBAL, &Gimbal_J7_CAN,
     Gimbal_J7_Send_CAN_ID, Gimbal_J7_Feedback_CAN_ID},
    {"gimbal_clap", MOTOR_TYPE_DM_3507, MOTOR_GROUP_GIMBAL, &Gimbal_Clap_CAN,
     Gimbal_Clap_Send_CAN_ID, Gimbal_Clap_Feedback_CAN_ID},
};

const uint16_t g_motor_device_count =
    (uint16_t)(sizeof(g_motor_devices) / sizeof(g_motor_devices[0]));

/*===|==================== 二、组名表 ====================|===*/

static motor_device_t *s_chassis_devices[8];
static motor_device_t *s_gimbal_devices[10];
static motor_device_t *s_shoot_devices[8];

static motor_group_t s_chassis_group = {
    .name = "chassis", .devices = s_chassis_devices, .count = 0, .capacity = 8};
static motor_group_t s_gimbal_group = {
    .name = "gimbal", .devices = s_gimbal_devices, .count = 0, .capacity = 10};
static motor_group_t s_shoot_group = {
    .name = "shoot", .devices = s_shoot_devices, .count = 0, .capacity = 8};

/* 索引必须与 motor_group_id_t 对应 */
static motor_group_t *const s_groups[MOTOR_GROUP_MAX] = {
    [MOTOR_GROUP_NONE] = NULL,
    [MOTOR_GROUP_CHASSIS] = &s_chassis_group,
    [MOTOR_GROUP_GIMBAL] = &s_gimbal_group,
    [MOTOR_GROUP_SHOOT] = &s_shoot_group,
};

motor_group_t *motor_group_get(motor_group_id_t id) {
  if (id < 0 || id >= MOTOR_GROUP_MAX)
    return NULL;
  return s_groups[id];
}

int motor_group_has_type(const motor_group_t *group, motor_type_id_t type_id) {
  if (group == NULL || group->devices == NULL)
    return 0;

  for (uint16_t i = 0; i < group->count; i++) {
    if (group->devices[i] != NULL && group->devices[i]->type_id == type_id)
      return 1;
  }
  return 0;
}

/*===|==================== 四、驱动注册（内联） ====================|===*/

/**
 * @brief 注册全部电机驱动
 * @note  每个驱动只暴露一个 Xxx_Register()，内部调用 motor_driver_register；
 *        新增驱动在这里加一行
 */
static inline void Motor_Driver_Register_All(void) {
  Motor_DAMIAO_Register();   // 达妙：各单机型号 + DM_DJI + 一拖四
  Motor_DJI_Register();      // 大疆：3508/2006/6020
  Motor_LINGKONG_Register(); // 凌空：MG6010/MG8010
  // Motor_DM_Read_Register();  // 达妙寄存器读取（临时，见 Motor_DM_read.h）
}

/*===|==================== 五、分组组装 ====================|===*/

/**
 * @brief 按设备表里的 group 字段自动入组
 */
static void Motor_Group_Build(void) {
  for (uint16_t i = 0; i < g_motor_device_count; i++) {
    motor_device_t *dev = &g_motor_devices[i];

    motor_group_t *group = motor_group_get((motor_group_id_t)dev->group);
    if (group == NULL || group->devices == NULL)
      continue;
    if (group->count >= group->capacity)
      continue;

    group->devices[group->count++] = dev;
  }
}

/*===|==================== 六、init ====================|===*/

int motor_init(void) {
  /* 1. 注册驱动 */
  Motor_Driver_Register_All();

  /* 2. 注册设备：框架建立设备链表、填默认值 */
  if (motor_device_register(g_motor_devices, g_motor_device_count) != 0)
    return -1;

  /* 3. 匹配驱动与设备：绑 ops，并执行驱动的 probe（挂上下文、分配通道） */
  int matched = motor_driver_match();
  if (matched != (int)g_motor_device_count)
    return -1;

  /* 4. 按设备声明的 group 自动入组 */
  Motor_Group_Build();

  return 0;
}

/*===|==================== 七、控制环接口 ====================|===*/

void motor_commit_all(motor_group_id_t id) {
  motor_group_t *group = motor_group_get(id);
  if (group == NULL || group->count == 0U)
    return;

  /* 一帧多机的驱动在这里发帧；没有对应设备就整个跳过，不白占总线。
     新增"一帧多机"驱动时，在下面加一行判定 + 一次 commit 调用即可。 */
  if (motor_group_has_type(group, MOTOR_TYPE_DJI_3508) ||
      motor_group_has_type(group, MOTOR_TYPE_DJI_2006) ||
      motor_group_has_type(group, MOTOR_TYPE_DJI_6020) ||
      motor_group_has_type(group, MOTOR_TYPE_DJI_LEGACY))
    Motor_DJI_CommitAll();

  if (motor_group_has_type(group, MOTOR_TYPE_DM_1TO4))
    Motor_DM1to4_CommitAll();
}

void motor_enable_all(motor_group_id_t id) {
  motor_group_t *group = motor_group_get(id);
  if (group == NULL)
    return;

  for (uint16_t i = 0; i < group->count; i++) {
    motor_device_t *dev = group->devices[i];
    /* 已经使能的跳过：Robo_Restart 这类入口可能被反复调用，
       不跳过会每次都往总线上灌一轮使能帧 */
    if (dev == NULL || dev->ops == NULL || dev->enabled != 0U)
      continue;

    motor_enable(dev);
  }
}

void motor_disable_all(motor_group_id_t id) {
  motor_group_t *group = motor_group_get(id);
  if (group == NULL)
    return;

  for (uint16_t i = 0; i < group->count; i++) {
    motor_device_t *dev = group->devices[i];
    /* 已经失能的跳过：遥控断连时 Robo_Stop 每拍都会进来。
       失能帧丢了由驱动兜底（达妙 set_value 发现回码仍是"已使能"会补发） */
    if (dev == NULL || dev->ops == NULL || dev->enabled == 0U)
      continue;

    motor_disable(dev);
  }
}
