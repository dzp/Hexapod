#include "MPU_task.h"
#include "mpu6050.h"
#include "cmsis_os.h"

extern "C"
{
  void MPU_Task(void const *argument)
  {
    static uint32_t code_time, tic, toc;
    osDelay(100);
    mpu6050.Init();
    osDelay(8000); //等待陀螺仪初始化
    while (1)
    {
      tic = xTaskGetTickCount();
      mpu6050.dmp_get_data();
      toc = xTaskGetTickCount();
      code_time = toc - tic; // 用时2ms
      osDelay(10);
    }
  }
}
