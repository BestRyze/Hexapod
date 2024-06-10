#include "MPU_task.h"
#include "mpu6050.h"
#include "cmsis_os.h"

extern "C"
{
  void MPU_Task(void const *argument)
  {
    osDelay(100);
    //mpu6050.Init();
    osDelay(8000); //µÈ´ýÍÓÂÝÒÇ³õÊ¼»¯
    while (1)
    {
      //mpu6050.dmp_get_data();
      osDelay(10);
    }
  }
}
