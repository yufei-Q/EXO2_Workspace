# Keil Watch单电机MIT测试

当前工程中 `MIT_WATCH_TEST_MODE` 已设为 `1U`。该模式暂停USB上位机控制，只通过Keil Debug的Watch窗口控制一台电机，MIT控制帧频率为500 Hz。

## 安全准备

首次测试应卸载或架空电机输出端，并准备快速断开48 V动力电源。上电时所有控制量为0，电机默认不使能。确保电机已经设置为MIT模式，CAN ID与测试ID一致。

## 加入Watch的变量

```text
mit_watch_motor_id
mit_watch_enable_request
mit_watch_clear_error_request
mit_watch_set_zero_request
mit_watch_motor_enabled
mit_watch_last_status
mit_watch_position
mit_watch_velocity
mit_watch_kp
mit_watch_kd
mit_watch_torque
dm_motors[0].feedback.status
dm_motors[0].feedback.position
dm_motors[0].feedback.velocity
dm_motors[0].feedback.torque
dm_motors[0].feedback.mos_temperature
dm_motors[0].feedback.rotor_temperature
```

`mit_watch_motor_id`默认为1。测试其他电机时，必须先失能，确认 `mit_watch_motor_enabled`为0，再修改ID。若选择2号电机，反馈数组下标应改为1，依此类推。

`mit_watch_last_status`的值为：0表示HAL_OK，1表示HAL_ERROR，2表示HAL_BUSY，3表示HAL_TIMEOUT。

## 操作顺序

1. 保持 `mit_watch_enable_request = 0`。
2. 先设置安全控制量：位置和速度目标为0，Kp、Kd和转矩也为0。
3. 需要清错时，将 `mit_watch_clear_error_request`改成1，程序执行一次后自动清零。
4. 需要设置零点时，保持电机失能，将 `mit_watch_set_zero_request`改成1，程序执行一次后自动清零。
5. 将 `mit_watch_enable_request`改成1。使能成功后，`mit_watch_motor_enabled`变为1。
6. 从很小的Kp、Kd或转矩开始逐步修改，不要直接输入较大数值。
7. 测试结束时，先把Kp和转矩改为0，再将 `mit_watch_enable_request`改成0。
8. 确认 `mit_watch_motor_enabled`变成0后再断开电源。

调试器暂停CPU时，500 Hz控制帧也会停止，因此不要把暂停按钮作为唯一安全措施。

## 恢复正常控制

在 `Core/Inc/main.h` 中改为：

```c
#define MIT_WATCH_TEST_MODE  0U
```

重新编译并烧录后，即恢复USB上位机和7台电机控制。

