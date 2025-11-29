#include <can2040.h>
#include <stdlib.h>
#include <hardware/regs/intctrl.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include "task.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "queue.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)
#define HIGH_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define MEDIUM_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define LOW_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define PRIORITY_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

#define MSG_FAST  1
#define MSG_SLOW 2000
#define HIGH_PRIO 1
#define LOW_PRIO 2

static struct can2040 cbus;
static QueueHandle_t queue;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if(notify & CAN2040_NOTIFY_RX)
    {        
        xQueueSendFromISR(queue, msg, NULL);
    }     
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY - 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void CanTransmitTask(void *pvParams)
{
    struct can2040_msg msg;
    msg.id  = LOW_PRIO;
    msg.dlc = 8;

    msg.data[0] = 'h';
    msg.data[1] = 'e';
    msg.data[2] = 'l';
    msg.data[3] = 'l';
    msg.data[4] = 'o';

    while(1)
    {
        if(can2040_transmit(&cbus, &msg) < 0)
        {
            printf("Failed: No space for msg to be queued");
        }
        vTaskDelay(pdMS_TO_TICKS(MSG_SLOW));
    }
}


void CanReceiveTask(void *pvParams)
{
    struct can2040_msg rcvdMsg;  
    while(1)
    {
        if (xQueueReceive(queue, &rcvdMsg, portMAX_DELAY) == pdTRUE) {
            
            char buf[9];
            uint8_t len = rcvdMsg.dlc;

            for (uint8_t i = 0; i < len; i++) 
            {
                buf[i] = (char)rcvdMsg.data[i];
            }

            buf[len] = '\0';
            printf("MSG ID= %lu DLC=%u Data='%s'\n", (unsigned long) rcvdMsg.id, rcvdMsg.dlc, buf);
        }
    }
}


int main(){
    stdio_init_all();
    sleep_ms(5000);

    queue = xQueueCreate(64, sizeof(struct can2040_msg));
    configASSERT(queue != NULL);
    canbus_setup();

    xTaskCreate(CanReceiveTask, "receieve_thread", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, NULL);
    xTaskCreate(CanTransmitTask, "transmit_thread", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, NULL);
    vTaskStartScheduler(); 

    return 0;
}