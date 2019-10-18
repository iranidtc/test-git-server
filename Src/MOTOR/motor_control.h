//#ifndef _motor_control_included_
//#include 

#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H


#include "stm32f3xx_hal.h"


#define p_digit 0xA8           //sevensegment wirring arrangment D-E-DP-A-C-F-G-B
#define dp_digit 0x20
#define c_digit 0x2B
#define v_digit 0x32
#define A_digit 0xA0
#define seven_segment_latch portb.4
#define content 0
extern char start_key,stop_key,down_key,up_key,ok_key,escap_key,right_rotation,free_run_state;
extern char fault_state,run_state,parameter_change,first_digit,second_digit,third_digit,fourth_digit,active_display_content,sevensegment_digit_num_display,finished_time_start_dc_injection,finished_time_stop_dc_injection,stop_dc_injection_state;
extern char seven_segment_code[20],display_digit[4],num_digit[4];
extern uint8_t led_7segment_data[2];
extern int parameter_state,last_parameter_state,last_parameter_content,display_content,motor_var_remain,abs_frequency,abs_ref_frequency,output_current,dc_link_voltage,output_voltage,temperature;
extern int parameter_content_max[1000],parameter_content_min[1000],parameter_content[1000],parameter_state_max[10];

void seven_segment_display();

void inverter_start();

void inverter_stop();

void dp_determin();

void digit_calculate(int _num);
 
void load_display_digit();

void seven_segment_display();

void user_interface();
 

#endif