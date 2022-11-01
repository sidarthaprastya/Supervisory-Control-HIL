#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "fsm.h"

#define GPIO_OUTPUT_A    2
#define GPIO_OUTPUT_B    4
#define GPIO_OUTPUT_C    5
#define GPIO_OUTPUT_D    18
#define GPIO_OUTPUT_E    19
#define GPIO_OUTPUT_F    21
#define GPIO_OUTPUT_G    22
#define GPIO_OUTPUT_H    23
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_A) |(1ULL<<GPIO_OUTPUT_B) |(1ULL<<GPIO_OUTPUT_C)|      (1ULL<<GPIO_OUTPUT_D)|(1ULL<<GPIO_OUTPUT_E)| (1ULL<<GPIO_OUTPUT_F)|(1ULL<<GPIO_OUTPUT_G)| (1ULL<<GPIO_OUTPUT_H))


#define GPIO_INPUT_PB    15
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_PB)
#define ESP_INTR_FLAG_DEFAULT 0
#define TIMER_DIVIDER         16  
#define TIMER_SCALE           (TIMER_BASE_CLK /TIMER_DIVIDER)  
#define DELAY_S               0.25
#define NUMBER_OF_LED         8
#define TIMER1_INTERVAL_SEC   (DELAY_S * NUMBER_OF_LED)

#define KP_vel  50.0
#define KI_vel  15.0
#define KD_vel  0.1

#define KP_acc  0.8
#define KI_acc  0.1
#define KD_acc  0.0

const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
double current_time_sec = 0, last_time_sec = 0, last_reset_time = 0; // untuk debounce sampling time

float vel = 0.0, pos = 0.0, accel = 0.0; //kecepatan dari motor
float output_pid = 0.0;
float setpoint = 0.0;
int next_valid = 1, button = 1, start_program = 0;
int counter, last_debounce, feed, state, sel;
float input;
float Kp, Ki, Kd;

float integral = 0;
float last_err = 0;

void pid(float input, float *output, float setpoint, float kp, float ki, float kd, float time){
	
    float error = setpoint - input;
    integral += error * time;
    float deriv = (last_err - error)/time;
    last_err = error;
    *output = kp * error + ki * integral + kd * deriv;
}

void button_config(){
    gpio_config_t io_conf;

    io_conf.intr_type = 0;
    io_conf.mode = GPIO_MODE_DEF_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_DEF_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Falling Edge
    io_conf.pull_up_en = 1; // enable interrupt
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

void timer_config(){
    // TIMER DEBOUNCE****************************
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP, 
        .counter_en = TIMER_START, 
        .alarm_en = TIMER_ALARM_EN, 
        .auto_reload = TIMER_AUTORELOAD_DIS,
    }; 
    // gunakan timer group dan hardware timer yang valid
    timer_init(0, 0, &config); //TIMERG0, hw_timer[0]
    timer_set_counter_value(0, 0, 0x00000000ULL);
    timer_start(0, 0);
}

//static void IRAM_ATTR gpio_isr_handler (void* arg) {
//    timer_get_counter_time_sec(0, 0, &current_time_sec);
//    if (current_time_sec - last_time_sec > DELAY_S) {
//        button = 1;
//        //start_program = 1;
//    }
//}

void main_control(void *pvParam){
	while(1){
		TickType_t xLastWakeTime1 = xTaskGetTickCount();
        if(scanf("%f;%f;%f", &vel, &pos, &accel) == 3){
            next_valid = 1;
        }
        if(next_valid == 1 ){ //&& start_program == 1

            if(state == 1 && (vel >= 1.0 || pos >= 2.0)){
                feed = 1;
            }
            else if(state == 2 && pos >= 1.5 ){
                feed = 1;
            }
            else if(state == 3 && pos >= 2.0){
                feed = 1;
            }
            else if(state == 5 && vel <= -1.0){
                feed = 1;
            }
            else if(state == 6 && pos <= 0.5){
                feed = 1;
            }
            else if(state == 7 && pos <= 0.0){
                feed = 1;
            }

            fsm(&counter, &button, &feed, &state, &sel, &setpoint);
            if(sel == 0){
                Kp = KP_acc;
                Ki = KI_acc;
                Kd = KD_acc;
                input = accel;
            }
            else{
                Kp = KP_vel;
                Ki = KI_vel;
                Kd = KD_vel;
                input = vel;
            }

            pid(input, &output_pid, setpoint, Kp, Ki, Kd, 0.01);
            printf("%d;%.8f\r\n", sel, output_pid);
            // Serial.print(sel);
            // Serial.print(";");
            // Serial.print(vel);
            // Serial.print(";");
            // Serial.println(output_pid);
            next_valid = 0;
        }
        vTaskDelayUntil(&xLastWakeTime1, 30/portTICK_PERIOD_MS);
	}
}

void button_task(void *pvParam){
	while(1){
		TickType_t xLastWakeTime = xTaskGetTickCount();
		timer_get_counter_time_sec(0, 0, &current_time_sec);
    		if (gpio_get_level(GPIO_INPUT_PB) == 0 && (current_time_sec - last_time_sec > DELAY_S)) {
        		button = 1;
        		//start_program = 1;
			    timer_get_counter_time_sec(0, 0, &last_time_sec);
    		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

void app_main()
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_DEF_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Falling Edge
    io_conf.pull_up_en = 1; // enable interrupt
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP, 
        .counter_en = TIMER_START, 
        .alarm_en = TIMER_ALARM_EN, 
        .auto_reload = TIMER_AUTORELOAD_DIS,
    }; 
    // gunakan timer group dan hardware timer yang valid
    timer_init(0, 0, &config); //TIMERG0, hw_timer[0]
    timer_set_counter_value(0, 0, 0x00000000ULL);
    timer_start(0, 0);
    // button_config();
    // timer_config();
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //gpio_isr_handler_add(GPIO_INPUT_PB, gpio_isr_handler, (void*) GPIO_INPUT_PB);
    //A uart read/write example without event queue;
    // xTaskCreate(GPIO_INPUT_PB, "uart_echo_task", 1024, NULL, 10, GPIO_INPUT_PB);

	xTaskCreatePinnedToCore(button_task, "Button Task", 2048, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(main_control, "Main Task", 2048, NULL, 1, NULL, 1);

}