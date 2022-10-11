#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define LEDA_PIO            PIOD
#define LEDA_PIO_ID         ID_PIOD
#define LEDA_PIO_PIN        30
#define LEDA_PIO_PIN_MASK (1u << LEDA_PIO_PIN)

#define LEDB_PIO            PIOA
#define LEDB_PIO_ID         ID_PIOA
#define LEDB_PIO_PIN        6
#define LEDB_PIO_PIN_MASK (1u << LEDB_PIO_PIN)

#define LEDC_PIO            PIOC
#define LEDC_PIO_ID         ID_PIOC
#define LEDC_PIO_PIN        19
#define LEDC_PIO_PIN_MASK (1u << LEDC_PIO_PIN)

#define LEDD_PIO            PIOA
#define LEDD_PIO_ID         ID_PIOA
#define LEDD_PIO_PIN        2
#define LEDD_PIO_PIN_MASK (1u << LEDD_PIO_PIN)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         ID_PIOD
#define BUT1_PIO_PIN       28
#define BUT1_PIO_PIN_MASK (1u << BUT1_PIO_PIN)

#define BUT2_PIO            PIOC
#define BUT2_PIO_ID         ID_PIOC
#define BUT2_PIO_PIN        31
#define BUT2_PIO_PIN_MASK (1u << BUT2_PIO_PIN)

#define BUT3_PIO            PIOA
#define BUT3_PIO_ID         ID_PIOA
#define BUT3_PIO_PIN        19
#define BUT3_PIO_PIN_MASK (1u << BUT3_PIO_PIN)

/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback1(void);
static void BUT_init(void);

QueueHandle_t QueueModo;
QueueHandle_t QueueSteps;
SemaphoreHandle_t xSemaphore;
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback1(void) {
	int angulo;
	angulo = 1;
	xQueueSendFromISR(QueueModo, &angulo, 0);
}

void but_callback2(void) {
	int angulo;
	angulo = 2;
	xQueueSendFromISR(QueueModo, &angulo, 0);
}

void but_callback3(void) {
	int angulo;
	angulo = 3;
	xQueueSendFromISR(QueueModo, &angulo, 0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	BUT_init();
	int valor;
	int passos;
	// 	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	// 	gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
		if (xQueueReceive(QueueModo, &valor, 1000)){
			if (valor == 1){
				gfx_mono_draw_string("180 Graus", 0,0, &sysfont);
				passos = 180/0.17578125;
				xQueueSend(QueueSteps, &passos, 0);
			}
			else if (valor == 2){
				gfx_mono_draw_string("90 Graus ", 0,0, &sysfont);
				passos = 90/0.17578125;
				xQueueSend(QueueSteps, &passos, 0);
			}
			else if (valor == 3){
				gfx_mono_draw_string("45 Graus ", 0,0, &sysfont);
				passos = 45/0.17578125;
				xQueueSend(QueueSteps, &passos, 0);
			}
		}
		
	}
}


static void task_motor(void *pvParameters){
	gfx_mono_ssd1306_init();
	BUT_init();
	int passos;
	int i =0;
	int j = 0;
	for (;;){
		if (xQueueReceive(QueueSteps, &passos, 1000)){
			printf("%d", passos);
			for(i=0;i<passos;i+=1){
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if( xSemaphoreTake(xSemaphore, 500 / portTICK_PERIOD_MS) == pdTRUE ){
					j+=1;
					if (j > 4){
						j =1;
					}
					if (j==1){
						pio_set(LEDA_PIO, LEDA_PIO_PIN_MASK);
						pio_clear(LEDB_PIO, LEDB_PIO_PIN_MASK);
						pio_clear(LEDC_PIO, LEDC_PIO_PIN_MASK);
						pio_clear(LEDD_PIO, LEDD_PIO_PIN_MASK);
					}
					if (j==2){
						pio_clear(LEDA_PIO, LEDA_PIO_PIN_MASK);
						pio_set(LEDB_PIO, LEDB_PIO_PIN_MASK);
						pio_clear(LEDC_PIO, LEDC_PIO_PIN_MASK);
						pio_clear(LEDD_PIO, LEDD_PIO_PIN_MASK);
					}
					if (j==3){
						pio_clear(LEDA_PIO, LEDA_PIO_PIN_MASK);
						pio_clear(LEDB_PIO, LEDB_PIO_PIN_MASK);
						pio_set(LEDC_PIO, LEDC_PIO_PIN_MASK);
						pio_clear(LEDD_PIO, LEDD_PIO_PIN_MASK);
					}
					if (j==4){
						pio_clear(LEDA_PIO, LEDA_PIO_PIN_MASK);
						pio_clear(LEDB_PIO, LEDB_PIO_PIN_MASK);
						pio_clear(LEDC_PIO, LEDC_PIO_PIN_MASK);
						pio_set(LEDD_PIO, LEDD_PIO_PIN_MASK);
					}
				}
				
			}
		}
	}
}
	/************************************************************************/
	/* funcoes                                                              */
	/************************************************************************/

	static void configure_console(void) {
		const usart_serial_options_t uart_serial_options = {
			.baudrate = CONF_UART_BAUDRATE,
			.charlength = CONF_UART_CHAR_LENGTH,
			.paritytype = CONF_UART_PARITY,
			.stopbits = CONF_UART_STOP_BITS,
		};

		/* Configure console UART. */
		stdio_serial_init(CONF_UART, &uart_serial_options);

		/* Specify that stdout should not be buffered. */
		setbuf(stdout, NULL);
	}
	void RTT_Handler(void) {
		uint32_t ul_status;
		ul_status = rtt_get_status(RTT);

		/* IRQ due to Alarm */
		if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
			xSemaphoreGiveFromISR(xSemaphore, 0);
		}
	}
	static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

		uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
		
		rtt_sel_source(RTT, false);
		rtt_init(RTT, pllPreScale);
		
		if (rttIRQSource & RTT_MR_ALMIEN) {
			uint32_t ul_previous_time;
			ul_previous_time = rtt_read_timer_value(RTT);
			while (ul_previous_time == rtt_read_timer_value(RTT));
			rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
		}

		/* config NVIC */
		NVIC_DisableIRQ(RTT_IRQn);
		NVIC_ClearPendingIRQ(RTT_IRQn);
		NVIC_SetPriority(RTT_IRQn, 4);
		NVIC_EnableIRQ(RTT_IRQn);

		/* Enable RTT interrupt */
		if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
		else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		
	}

	static void BUT_init(void) {
		pmc_enable_periph_clk(BUT1_PIO_ID);
		pmc_enable_periph_clk(BUT2_PIO_ID);
		pmc_enable_periph_clk(BUT3_PIO_ID);
		/* configura prioridae */
		NVIC_EnableIRQ(BUT1_PIO_ID);
		NVIC_SetPriority(BUT1_PIO_ID, 4);

		/* conf botão como entrada */
		pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_PIN_MASK, 60);
		pio_enable_interrupt(BUT1_PIO, BUT1_PIO_PIN_MASK);
		pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback1);
		
		/* configura prioridae */
		NVIC_EnableIRQ(BUT2_PIO_ID);
		NVIC_SetPriority(BUT2_PIO_ID, 4);

		/* conf botão como entrada */
		pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_PIN_MASK, 60);
		pio_enable_interrupt(BUT2_PIO, BUT2_PIO_PIN_MASK);
		pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback2);
		
		/* configura prioridae */
		NVIC_EnableIRQ(BUT3_PIO_ID);
		NVIC_SetPriority(BUT3_PIO_ID, 4);

		/* conf botão como entrada */
		pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_PIN_MASK, 60);
		pio_enable_interrupt(BUT3_PIO, BUT3_PIO_PIN_MASK);
		pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback3);
		
		pmc_enable_periph_clk(LEDA_PIO_ID);
		pio_configure(LEDA_PIO, PIO_OUTPUT_0, LEDA_PIO_PIN_MASK, PIO_DEFAULT);
		pio_set_output(LEDA_PIO, LEDA_PIO_PIN_MASK, 0, 0, 0);
		
		pmc_enable_periph_clk(LEDB_PIO_ID);
		pio_configure(LEDB_PIO, PIO_OUTPUT_0, LEDB_PIO_PIN_MASK, PIO_DEFAULT);
		pio_set_output(LEDB_PIO, LEDB_PIO_PIN_MASK, 0, 0, 0);
		
		pmc_enable_periph_clk(LEDC_PIO_ID);
		pio_configure(LEDC_PIO, PIO_OUTPUT_0, LEDC_PIO_PIN_MASK, PIO_DEFAULT);
		pio_set_output(LEDC_PIO, LEDC_PIO_PIN_MASK, 0, 0, 0);
		
		pmc_enable_periph_clk(LEDD_PIO_ID);
		pio_configure(LEDD_PIO, PIO_OUTPUT_0, LEDD_PIO_PIN_MASK, PIO_DEFAULT);
		pio_set_output(LEDD_PIO, LEDD_PIO_PIN_MASK, 0, 0, 0);

	}

	/************************************************************************/
	/* main                                                                 */
	/************************************************************************/


	int main(void) {
		/* Initialize the SAM system */
		sysclk_init();
		board_init();

		/* Initialize the console uart */
		configure_console();
		
		QueueModo = xQueueCreate(32, sizeof(int) );
		QueueSteps = xQueueCreate(32, sizeof(int) );
		xSemaphore = xSemaphoreCreateBinary();
		
		if (QueueModo == NULL){
			printf("falha em criar a fila \n");
		}
		
		if (QueueSteps == NULL){
			printf("falha em criar a fila \n");
		}
		/* Create task to control oled */
		if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
			printf("Failed to create modo task\r\n");
		}
		
		if (xTaskCreate(task_motor, "modmotor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
			printf("Failed to create motor task\r\n");
		}

		/* Start the scheduler. */
		vTaskStartScheduler();

		/* RTOS não deve chegar aqui !! */
		while(1){
		}

		/* Will only get here if there was insufficient memory to create the idle task. */
		return 0;
	}
