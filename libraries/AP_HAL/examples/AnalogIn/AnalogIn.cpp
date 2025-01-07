/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
    Analogin.cpp : We loop through the 16 pins and output the analog voltage of the pins
    Analogin.cpp : 我们遍历16个引脚并输出每个引脚的模拟电压值
*/
#include <AP_HAL/AP_HAL.h>

void setup();    //declaration of the setup() function  //声明setup()函数
void loop();     //declaration of the loop() function   //声明loop()函数

const AP_HAL::HAL& hal = AP_HAL::get_HAL();    //create a reference to AP_HAL::HAL object to get access to hardware specific functions. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>  
                                               //创建一个AP_HAL::HAL对象的引用以访问硬件特定功能

AP_HAL::AnalogSource* chan;    //declare a pointer to AnalogSource object. AnalogSource class can be found in : AP_HAL->AnalogIn.h
                              //声明一个指向AnalogSource对象的指针。AnalogSource类定义在AP_HAL->AnalogIn.h中

// the setup function runs once when the board powers up
// setup函数在开发板上电时只运行一次
void setup(void) {
    hal.console->printf("Starting AP_HAL::AnalogIn test\r\n");    //print a starting message  //打印启动消息
    chan = hal.analogin->channel(0);    //initialization of chan variable. AnalogIn class can be found in : AP_HAL->AnalogIn.h
                                       //初始化chan变量。AnalogIn类定义在AP_HAL->AnalogIn.h中
}

static int8_t pin;    //8 bit integer to hold the pin number.Pin number range is [0,15]
                     //8位整数用于存储引脚编号。引脚编号范围是[0,15]

//the loop function runs over and over again forever
//loop函数会永远重复运行
void loop(void) {
    //get the average voltage reading
    //获取平均电压读数
    float v  = chan->voltage_average();    //note:the voltage value is divided into 1024 segments    
                                          //注意:电压值被分成1024段
    //start a new line after going through the 16 pins
    //遍历完16个引脚后开始新的一行
    if (pin == 0) {
        hal.console->printf("\n");
    }
    //print the voltage value(3 decimal places) alongside the pin number 
    //打印引脚编号和电压值(保留3位小数)
    hal.console->printf("[%u %.3f] ",
              (unsigned)pin, (double)v);
    //increment the pin number
    //增加引脚编号
    pin = (pin+1) % 16;
    //set pin corresponding to the new pin value
    //设置对应新引脚值的引脚
    IGNORE_RETURN(chan->set_pin(pin));
    //give a delay of 100ms
    //延时100毫秒
    hal.scheduler->delay(100);
}

AP_HAL_MAIN(); //HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>
               //声明main函数的HAL宏。更多信息请参见<https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>
