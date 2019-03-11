/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/*
 *  \file rq_sensor_com.h
 *  \date June 19, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotic.com>
 *  \maintener Nicolas Lauzier <nicolas@robotiq.com>
 */

#ifndef RQ_SENSOR_COM_H
#define RQ_SENSOR_COM_H

#include "rq_int.h"
#include <stdio.h>
#include <stdbool.h>

#define MP_BUFF_SIZE    1024

#ifdef __cplusplus    //__cplusplus是cpp中自定义的一个宏
extern "C" {          //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
#endif

/** while循环式查询端口是否连接成功 **/
INT_8 rq_sensor_com(void);

/** 读取传感器生产信息 **/
void rq_sensor_com_read_info_high_lvl(void);

/** 启动数据流 **/
INT_8 rq_com_start_stream(void);


void rq_com_listen_stream(void);
bool rq_com_get_stream_detected(void);
bool rq_com_get_valid_stream(void);

//Accesseur
void rq_com_get_str_serial_number(INT_8 * serial_number);
void rq_com_get_str_firmware_version(INT_8 * firmware_version);
void rq_com_get_str_production_year(INT_8 * production_year);

float rq_com_get_received_data(UINT_8 i);
bool rq_com_got_new_message(void);
void rq_com_do_zero_force_flag(void);
void stop_connection(void);

bool Calibration_FT300_X();
bool Calibration_FT300_Y();
bool Calibration_FT300_Z();

void FT300_Disconnect();
#ifdef __cplusplus
}
#endif
#endif //RQ_SENSOR_COM_H