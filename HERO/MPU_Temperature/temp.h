#ifndef __TEMP_H
#define __TEMP_H

#include <system.h>

void MPU_TempPID_Init_IO(void);
void Tempeture_PID(void);

extern short Temperature_Value;
#endif
