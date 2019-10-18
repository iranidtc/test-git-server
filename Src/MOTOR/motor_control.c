
#include "motor_control.h"

extern SPI_HandleTypeDef hspi3;

char start_key=0,stop_key=0,down_key=0,up_key=0,ok_key=0,escap_key=0,right_rotation=0,free_run_state=0,finished_time_start_dc_injection=0,finished_time_stop_dc_injection=0,stop_dc_injection_state=0,start_dc_injection_state=0,error_over_voltage=0,error_under_voltage=0,error_over_heat=0,ups_state=0,multifunction_input_enable=0;
char fault_state=0,run_state=0,parameter_change=0,first_digit,second_digit,third_digit,fourth_digit;
char active_display_content,sevensegment_digit_num_display=0,heatsink_temperature=0;
char seven_segment_code[20],num_digit[4],display_digit[4];
uint8_t led_7segment_data[2]={0};
int parameter_state,last_parameter_state,last_parameter_content,display_content,motor_var_remain=0,abs_frequency,abs_ref_frequency,output_current,dc_link_voltage,output_voltage,temperature,counter_start_dc_injection=0,counter_stop_dc_injection=0,dc_bus_voltage=0;
int parameter_content_max[1000],parameter_content_min[1000],parameter_content[1000],parameter_state_max[10];


void seven_segment_display();

void frequency_determin();

void dp_determin();

/*
void multi_function_input_logic_one (unsigned int array)
{

    switch (content[array]) {

    case 0 :
        input_forward_run=1;	//auto_run has high periority for run command 
    break;

    case 1 :
        input_reverse_run=1;		//auto_run has high periority for run command 
    break;

    case 2 :
        input_emergency_stop=1;
        input_reverse_run=0;
        input_forward_run=0;
        input_auto_run=0;
	auto_run_finish_flage=0;
	input_lebas_highspeed=0;
	lebas_highspeed_finish_flag=0;
    break;

    case 3 :
        input_fault_reset=1;
    break;

    case 4 :
        input_jog_operation=1;
    break;

    case 5 :
        multi_step_frequency_low_flag=1;
    break;

    case 6 :
        multi_step_frequency_medium_flag=1;
    break;

    case 7 :
        multi_step_frequency_high_flag=1;
    break;

    case 8 :
        multi_acc_dec_low_flag=1;
    break;

    case 9 :
        multi_acc_dec_medium_flag=1;
    break;

    case 10 :
        multi_acc_dec_high_flag=1;
    break;

    case 17 :
        io_pulse_3_wire=1;
    break;
    
    case 25:
        input_auto_run=1;
    break;
    
    case 26:
	if (input_auto_run==0) input_lebas_highspeed=1;
    break;
    }

}


void multi_function_input_logic_zero (unsigned int array1)
{

    switch (content[array1]) {

    case 0 :
        if (io_pulse_3_wire==0 || (input_reverse_run==1 && content[3]!=2) || (input_auto_run && auto_run_finish_flage==0)) input_forward_run=0;
    break;

    case 1 :
        if (io_pulse_3_wire==0 || (input_forward_run==1 && content[3]!=2) ||  (input_auto_run && auto_run_finish_flage==0 && content[3]!=2)) input_reverse_run=0;
    break;

    case 2 :
        if (io_pulse_3_wire==0 || input_reverse_run==1 || input_forward_run==1 || input_auto_run==1) {
            input_emergency_stop=0;
//            input_reverse_run=0;
  //          input_forward_run=0;
            }
    break;

    case 3 :
        input_fault_reset=0;
    break;

    case 4 :
        input_jog_operation=0;
    break;

    case 5 :
        multi_step_frequency_low_flag=0;
    break;

    case 6 :
        multi_step_frequency_medium_flag=0;
    break;

    case 7 :
        multi_step_frequency_high_flag=0;
    break;

    case 8 :
        multi_acc_dec_low_flag=0;
    break;

    case 9 :
        multi_acc_dec_medium_flag=0;
    break;

    case 10 :
        multi_acc_dec_high_flag=0;
    break;

    case 17 :
        io_pulse_3_wire=0;
    break;
	
	case 25:
        if (io_pulse_3_wire==0) input_auto_run=0;	//in auto run mode and autorun rotation equal 3 that cause rotation by fx and rx but freq determine by auto run
	break;
 
 	case 26:
 		if (io_pulse_3_wire==0) input_lebas_highspeed=0;
 	break;
	};

}


void digital_input_multifunction(char _input_multifunction){

  switch (_output_multifunction){

    case 0 :    //motor contactor
    if (run_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 1 :     //brake contactor
    if (run_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 2 :     //fault
    if (fault_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 3 :     //enable
    if (multifunction_input_enable) _digital_state=1;
    else _digital_state=0;
    break;
  }
    
}

*/


char digital_output_multifunction(char _output_multifunction){
char _digital_state;
  
  switch (_output_multifunction){

    case 0 :    //motor contactor
    if (run_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 1 :     //brake contactor
    if (run_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 2 :     //fault
    if (fault_state) _digital_state=1;
    else _digital_state=0;
    break;

    case 3 :     //enable
    if (multifunction_input_enable) _digital_state=1;
    else _digital_state=0;
    break;
  }

return _digital_state;
}

void protection_functions(){

  if (run_state){
    
    //voltage check
     if (dc_bus_voltage>800){
      error_over_voltage=1;
      fault_state=1;
      run_state=0;
    }
    else if (dc_bus_voltage<350 || (ups_state && dc_bus_voltage<200)){
          error_under_voltage=1;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
          fault_state=1;
          run_state=0;
    }
    else if (dc_bus_voltage>385 || (ups_state && dc_bus_voltage>250)){
          error_under_voltage=0;
          error_over_voltage=0;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    }
  
    //temperature check
    if (heatsink_temperature>95){
      error_over_heat=1;
      fault_state=1;
      run_state=0;
    }
    else if (heatsink_temperature<85) error_over_heat=1;
  
  }

}


void time_check(){
  if (start_dc_injection_state){
    counter_start_dc_injection++;
    if (counter_start_dc_injection>parameter_content[content]){
      finished_time_start_dc_injection=1;
      start_dc_injection_state=0;
      counter_start_dc_injection=0;
    }
  }
  else if (stop_dc_injection_state){
    counter_stop_dc_injection++;
    if (counter_stop_dc_injection>parameter_content[content]){
      finished_time_stop_dc_injection=1;
      stop_dc_injection_state=0;
      counter_stop_dc_injection=0;
    }
  }
  if (finished_time_stop_dc_injection) run_state=0;
}


void inverter_start(){
  if (!fault_state){
    if (!finished_time_start_dc_injection && parameter_content[content]) start_dc_injection_state=1;
    else frequency_determin();
    run_state=1;
  }
}

void inverter_stop(){
  if (free_run_state) run_state=0;
  else if (parameter_content[content]){
    if (abs_frequency<parameter_content[content] && finished_time_stop_dc_injection) stop_dc_injection_state=1;
  }
  else if (abs_frequency==abs_ref_frequency) run_state=0;
}


void digit_calculate(int _num){
  num_digit[3]=_num/1000;
  num_digit[2]=(_num-num_digit[3]*1000)/100;
  num_digit[1]=(_num-num_digit[3]*1000-num_digit[2]*100)/10;
  num_digit[0]=_num-(_num/10)*10;
}
 
void load_display_digit(){
      display_digit[3]=seven_segment_code[num_digit[3]]; 
      display_digit[2]=seven_segment_code[num_digit[2]];
      display_digit[1]=seven_segment_code[num_digit[1]];
      display_digit[0]=seven_segment_code[num_digit[0]];
}


void seven_segment_display(){
  if (parameter_state!=last_parameter_state || parameter_content[parameter_state]!=last_parameter_content){
    
    if (parameter_state>99 && !active_display_content){
       digit_calculate(parameter_state);      
       load_display_digit();      
       display_digit[2]=display_digit[2]+dp_digit;
       display_digit[3]=p_digit; 
    }
    
    else if (active_display_content){
      digit_calculate(parameter_content[parameter_state]);
      load_display_digit();
      dp_determin();
    }
    
    else if (parameter_state==0){
      if (run_state){
        digit_calculate(abs_frequency);
        load_display_digit();        
        display_digit[2]=display_digit[2]+dp_digit;
      }
      else{
        digit_calculate(abs_ref_frequency);
        load_display_digit();        
        display_digit[2]=display_digit[2]+dp_digit;        
      }
    }
    else if (parameter_state==1){
        digit_calculate(output_current);
        load_display_digit();        
        display_digit[1]=display_digit[1]+dp_digit;                
        display_digit[3]=A_digit;
    }
    else if (parameter_state==2){
        digit_calculate(dc_link_voltage);
        load_display_digit();        
        display_digit[3]=v_digit;        
    }
    else if (parameter_state==3){
        digit_calculate(output_voltage);
        load_display_digit();        
        display_digit[3]=v_digit;                
    }
    else if (parameter_state==4){
        digit_calculate(temperature);
        load_display_digit();        
        display_digit[1]=display_digit[1]+dp_digit;                
        display_digit[3]=c_digit;      
    }
  }
  
  sevensegment_digit_num_display++;
  if (sevensegment_digit_num_display>3) sevensegment_digit_num_display=0;
  
  led_7segment_data[0]=display_digit[sevensegment_digit_num_display];
  
  led_7segment_data[1]=0;
  if (parameter_state==0) led_7segment_data[1]=led_7segment_data[1] | 0x08;
  if (parameter_state==1) led_7segment_data[1]=led_7segment_data[1] | 0x02;
  if (run_state) led_7segment_data[1]=led_7segment_data[1] | 0x01;
  if (right_rotation) led_7segment_data[1]=led_7segment_data[1] | 0x04;

  if (sevensegment_digit_num_display==0)    led_7segment_data[1]=led_7segment_data[1]  | 0x10;
  else if (sevensegment_digit_num_display==1)    led_7segment_data[1]=led_7segment_data[1]  | 0x20;
  else if (sevensegment_digit_num_display==2)    led_7segment_data[1]=led_7segment_data[1]  | 0x40;
  else if (sevensegment_digit_num_display==3)    led_7segment_data[1]=led_7segment_data[1]  | 0x80;
  

  HAL_SPI_Transmit(&hspi3, &led_7segment_data[0], 2 ,1);
 // HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
}

void user_interface(){
    seven_segment_display();
    if (fault_state){
      if (stop_key){
        parameter_state=last_parameter_state;
      }
    }
    else if (start_key){
      inverter_start();
    }
    else if (stop_key){
       inverter_stop(); 
    }
    else if (ok_key){
      if (active_display_content){
        active_display_content=0;
        if (run_state==0) {
          parameter_content[parameter_state]=display_content;
          parameter_change=1;
        }
      }
      else if (parameter_state>99) active_display_content=1;
    }
    else if (escap_key){
        if (active_display_content) active_display_content=0;   
        else{
           motor_var_remain=(parameter_state-(parameter_state/100)*100);
           if (motor_var_remain==0) parameter_state=0;
           else parameter_state=parameter_state-motor_var_remain;
        }
    }
    else if (up_key){
      if (active_display_content){
        if (parameter_content[parameter_state]<parameter_content_max[parameter_state]) parameter_content[parameter_state]++;
      }
      else {
        motor_var_remain=parameter_state-(parameter_state/100)*100;
        if (parameter_state_max[parameter_state/100]>motor_var_remain) parameter_state++;
      }
    }
    else if (down_key){
      if (active_display_content){
        if (parameter_content[parameter_state]>parameter_content_min[parameter_state]) parameter_content[parameter_state]--;
      }
      else {
        motor_var_remain=parameter_state-(parameter_state/100)*100;
        if (motor_var_remain>0) parameter_state--;        
      }
    }
}
