#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

/**
 * @brief FreeRTOS task entry point that continuously:
 *        1) Sends trajectory setpoints to engaged motors,
 *        2) Processes batch commands (if any),
 *        3) Monitors for repeated CAN failures and triggers forced shutdown.
 */
void motor_control_task(void *arg);

#endif // MOTOR_CONTROL_TASK_H
