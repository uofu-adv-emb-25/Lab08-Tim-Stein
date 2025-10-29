#include <can2040.h>
#include <hardware/regs/intctrl.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include "task.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)
#define HIGH_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define MEDIUM_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define LOW_PRIORITY_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define PRIORITY_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static struct can2040 cbus;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if(CAN2040_NOTIFY_RX == notify)
    {
        char received[8];
        uint16_t msg_size = (msg->dlc);

        for(int i = 0; i < msg_size; i++)
        {
            received[i] = (char) msg->data[i];
        }

        for(int j = 0; j < msg_size; j++)
        {
            if(j == 0) {printf("MSG RCVD: ");}
            else
            {
                printf("%c\n", msg->data[j]);
            }
        }
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
    while(1)
    {
        msg.id = 0x11;
        msg.dlc = 5; // hello is 5 bytes
        
        msg.data[0] = 'h';
        msg.data[1] = 'e';
        msg.data[2] = 'l';
        msg.data[3] = 'l';
        msg.data[4] = 'o';
        msg.data[5] = 0;
        msg.data[6] = 0;
        msg.data[7] = 0;

        (void) can2040_transmit(&cbus, &msg);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(){
    stdio_init_all();
    sleep_ms(5000);
    canbus_setup();

    xTaskCreate(CanTransmitTask, "low_priority_thread", PRIORITY_TASK_STACK_SIZE, NULL, LOW_PRIORITY_TASK_PRIORITY, NULL);
    vTaskStartScheduler(); 

    return 0;
}