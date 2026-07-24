# D4340P control guide

The project initializes four 48 V D4340P motors on CAN1 at 1 Mbps. Motor IDs
are fixed at `0x01` through `0x04`, and Master ID is fixed at `0x000`.

The application intentionally does not enable a motor or send a motion command
at startup. After the mechanical assembly is secured and the emergency stop is
available, a typical MIT-mode sequence is:

```c
DM_D4340P_ClearError(&d4340p_motors[0]);
HAL_Delay(10U);
DM_D4340P_Enable(&d4340p_motors[0]);
HAL_Delay(10U);

/* Send periodically. For position control, kd must not be zero. */
DM_D4340P_MitControl(&d4340p_motors[0],
                     0.0f,   /* position, rad */
                     0.0f,   /* velocity, rad/s */
                     3.0f,   /* kp */
                     0.5f,   /* kd */
                     0.0f);  /* feedforward torque, N*m */
```

State feedback for motor `n` is stored by the CAN interrupt in
`d4340p_motors[n].feedback`. Check `status` before using the measurement in a
control loop.

The driver uses PMAX = 12.5 rad, VMAX = 20 rad/s, and TMAX = 28 N*m. These
values must match the motor settings. Configure the motors for MIT mode and
1 Mbps CAN with the Damiao host tool before use.
