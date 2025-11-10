#include <can2040.h>
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

static struct can2040 cbus;
static QueueHandle_t queue;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if(CAN2040_NOTIFY_RX == notify)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue, msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    uint32_t gpio_rx = 5, gpio_tx = 4;

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
    struct can2040_msg msg = {0};
    msg.id  = 0x11;
    msg.dlc = 5;

    msg.data[0] = 'h';
    msg.data[1] = 'e';
    msg.data[2] = 'l';
    msg.data[3] = 'l';
    msg.data[4] = 'o';

    for (;;) {
        (void)can2040_transmit(&cbus, &msg);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void messageTask(void *pvParams)
{
    struct can2040_msg rx;
    for (;;) {

        if (xQueueReceive(queue, &rx, portMAX_DELAY) == pdTRUE) {
            char buf[9] = {0};
            size_t n = (rx.dlc <= 8) ? rx.dlc : 8;

            for (size_t i = 0; i < n; ++i) 
            {
                buf[i] = (char)rx.data[i];
            }

            buf[n] = '\0';
            printf("RX ID=0x%08lx DLC=%u Data='%s'\n", (unsigned long)rx.id, rx.dlc, buf);
        }
    }
}


int main(){
    stdio_init_all();
    sleep_ms(5000);

    queue = xQueueCreate(10, sizeof(struct can2040_msg));
    configASSERT(queue != NULL);
    canbus_setup();

    xTaskCreate(messageTask, "receieve_thread", PRIORITY_TASK_STACK_SIZE, NULL, MEDIUM_PRIORITY_TASK_PRIORITY, NULL);
    //xTaskCreate(CanTransmitTask, "transmit_thread", PRIORITY_TASK_STACK_SIZE, NULL, MEDIUM_PRIORITY_TASK_PRIORITY, NULL);
    vTaskStartScheduler(); 

    return 0;
}