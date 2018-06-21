#pragma once 

#include <iostream>

/***
 *a: the 0-8 bit, or data[0]  
 * return: mode, 
 * 0：待机模式；
 * 1：自动驾驶模式；
 * 2：保留；
 * 4：手动模式；
 * 5：手动介入恢复模式；
 * 6：警告模式；
 * 7：错误模式；
 * 8-15：预留；
 ***/
unsigned char can_get_mode(unsigned char a){
    return a & 0x0f;
}

// 假定0为底位
void can_set_steering(unsigned char data[8], unsigned int steerling_angel, unsigned char turn_speed_limit){
    data[0] = steerling_angel >> 8;
    data[1] = steerling_angel & 0x00ff;
    data[4] = turn_speed_limit; 
}


