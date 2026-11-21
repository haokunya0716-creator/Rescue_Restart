/*
* app_button.h
 *
 *  Created on: May 6, 2025
 *      Author: gaoxi
 */

#ifndef INC_APP_BUTTON_H_
#define INC_APP_BUTTON_H_


/* 对应按键单击时置 1，由调用者读取和清零。 */
extern int key1_flag;
extern int key2_flag;
extern int key3_flag;

#include "main.h"

void App_Button_Init(void);
void App_Button_Proc(void);


#endif /* INC_APP_BUTTON_H_ */
