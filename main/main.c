// librerias
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h" 
#include <math.h>
#include "esp_event.h"
#include "freertos/queue.h"




// define entradas para el boton
#define ADC_CHANNEL         ADC1_CHANNEL_0 // Canal ADC que se utilizará
#define ADC_WIDTH           ADC_WIDTH_BIT_12 // Resolución del ADC
#define ADC_ATTENUATION     ADC_ATTEN_DB_11 // Atenuación del ADC
#define QUEUE_SIZE          10 // Tamaño de la cola

//Led 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

#define ledR            15
#define ledG            5
#define ledB            4


#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT // Set duty resolution to 14 bits
#define LEDC_FREQUENCY          (3000)            // Frequency in Hertz. Set frequency at 5 kHz

#define MIN_INTENSITY           10
#define MAX_INTENSITY           100


//Variables del UART
// Define la configuración UART
#define UART_NUM UART_NUM_0 // Puedes cambiar el número de UART si es necesario
#define UART_BAUD_RATE 115200
#define TXD_PIN (GPIO_NUM_4) // Reemplaza GPIO_NUM_4 con el número de pin que desees
#define RXD_PIN (GPIO_NUM_5) // Reemplaza GPIO_NUM_5 con el número de pin que desees
#define LED_RANGE_START 'R' // Nuevo prefijo para identificar comandos de rango
#define LED_RANGE_LENGTH 6 // Longitud total del comando de rango (incluido 'R' y otros caracteres)
#define BUF_SIZE 1024 // Cambia el tamaño según tus necesidades
#define portTICK_RATE_MS portTICK_PERIOD_MS







//valores para la temperatura RGB
int temp_valor1=0;
int temp_valor2=30;
int temp_valor3=50;
//inicializa el led 
esp_err_t init_led(void);

//Variables ADC

#define Res0 10.0  // Resistencia a 25 grados Celsius
#define Bits_ADC 4096 // Resolución del ADC (12 bits)
#define Beta 113000.0   // Valor Beta del termistor

int raw = 0;
int res_2 = 1000;
float temp = 1.0;
float v_res = 0.0;
float res_ntc = 0.0;
int temp_index = 0;

#define MAX_INTENSITY          100
#define MAX_DUTY_CYCLE          8091 // Máximo valor de duty cycle para el 100%


#define BUTTON_PIN (26) // Cambia esto al pin GPIO donde está conectado tu botón

static int delay_ms = 1000; // Inicialmente, espera 1 segundo entre mensajes
static int speed_index = 0; // Índice de velocidad actual

// Definir las velocidades disponibles (en milisegundos)
static int speeds[] = {1000, 800, 600, 400, 200};




// Definición de la cola para enviar valores ADCy uart
QueueHandle_t adc_queue;
QueueHandle_t uart_queue;

int current_intensity = 0; // Intensidad actual de los LED




//Inicializo el pin del led
esp_err_t init_led(void) 
{
    // Inicializo el pin del led ROJO
    gpio_reset_pin(ledR);
    gpio_set_direction(ledR, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led VERDE
    gpio_reset_pin(ledG);
    gpio_set_direction(ledG, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led AZUL
    gpio_reset_pin(ledB);
    gpio_set_direction(ledB, GPIO_MODE_OUTPUT);
    return ESP_OK;
}




// Inicializar LEDC para controlar la intensidad de los LED
void ledc_init() {
   // Configurar el temporizador LEDC para los tres LEDs
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,  // Frecuencia de salida de 5 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configurar los canales LEDC para los tres LEDs
    ledc_channel_config_t ledc_channel_R = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0, // Canal 0 para el LED Rojo
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ledR, // Pin del LED Rojo
        .duty = 0,        // Inicialmente, establece el ciclo de trabajo en 0%
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_R));

    ledc_channel_config_t ledc_channel_G = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1, // Canal 1 para el LED Verde
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ledG, // Pin del LED Verde
        .duty = 0,        // Inicialmente, establece el ciclo de trabajo en 0%
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_G));

    ledc_channel_config_t ledc_channel_B = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2, // Canal 2 para el LED Azul
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ledB, // Pin del LED Azul
        .duty = 0,        // Inicialmente, establece el ciclo de trabajo en 0%
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_B));

    ledc_fade_func_install(0);
}



// Función para configurar la intensidad del LED en función de la temperatura
void set_led_intensity(void *pvParameters) {
    uint32_t current_intensity; // Variable para la intensidad del LED
    uint32_t  intensity ; // Cantidad de incremento en cada paso
    uint32_t maxDutyCycle = 8091;

    while (xQueueReceive(uart_queue, &intensity, portMAX_DELAY)) {
        if (intensity < MIN_INTENSITY) {
        current_intensity = MIN_INTENSITY;
    } else if (intensity > MAX_INTENSITY) {
        current_intensity = MAX_INTENSITY;
    } else {
        current_intensity = (intensity*maxDutyCycle)/ MAX_INTENSITY;
    }
    // Configurar el ciclo de trabajo del canal LEDC
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2,  current_intensity );
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1,  current_intensity );
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0,  current_intensity );
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);


     vTaskDelay(pdMS_TO_TICKS(100)); // Espera un segundo antes de cambiar la intensidad
    }
}


// Funcion para dectectar el boton
void button_task(void *pvParameter) {
    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE // Detectar flanco de bajada (cuando se presiona el botón)
    };

    gpio_config(&button_config);

    while (1) {
        if (gpio_get_level(BUTTON_PIN) == 0) {
            // El botón está presionado, así que avanzamos a la siguiente velocidad
            speed_index = (speed_index + 1) % 5;
            delay_ms = speeds[speed_index];
            //printf("Velocidad actual: %d ms\n", delay_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Esperar un corto tiempo antes de volver a verificar
    }
}


// Cambia la velocidad de impresion 
void message_task(void *pvParameter) {
    uint32_t  temperas;
    
    float prom_temp = 0;
    while (xQueueReceive(adc_queue, &temperas, portMAX_DELAY) == pdTRUE) {
        
       // printf("Temperatura: %.2ld°C\n", temperas);
       // printf("Mensaje cada %d ms\n", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Esperar el tiempo actual entre mensajes
        prom_temp= temperas;
        if ( prom_temp < temp_valor2) {
            gpio_set_level(ledB,1);
            gpio_set_level(ledR,0);
            gpio_set_level(ledG,0);

        }

        else if ( temp_valor2 < prom_temp &&  prom_temp < temp_valor3){
            gpio_set_level(ledR,0);
            gpio_set_level(ledG,1);
            gpio_set_level(ledB,0);
        }
        else if ( temp_valor3 < prom_temp)
        {
            gpio_set_level(ledR,1);
            gpio_set_level(ledG,0);
            gpio_set_level(ledB,0);
        }

        //printf ("La temperatura es: %.2f\n", prom_temp);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar un segundo para volver a medir la temperatura
    


        
    }
}


// Tarea para leer el valor ADC y enviar la temperatura a la cola de temperatura
void task_read_adc(void *pvParameter) {
    // Configurar el ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTENUATION);

    while (1) {
        // Leer el valor del ADC
        uint32_t adc_value = adc1_get_raw(ADC_CHANNEL);
        uint32_t temp = 0;
        // Calcular la temperatura en función del valor ADC (ecuación de Steinhart-Hart)
        float r_ntc = Res0 * (Bits_ADC / (float)adc_value - 1.0);
        float temperature = 1.0 / (1.0 / 298.15 + 1.0 / Beta * log(r_ntc / Res0)) - 273.15;
        temp =  temperature;
         xQueueSend(adc_queue, &temp, portMAX_DELAY);
        // Imprimir el valor del ADC y la temperatura
        printf("Temperatura: %.2ld°C\n", temp);

        // Esperar 1 segundo antes de la próxima lectura de temperatura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Funcion de Comunicacion
void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}


// Funcion para recibir el mensaje
void uart_receive_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    u_int32_t datos;
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);

            data[len] = 0; // Agrega un terminador nulo al final de los datos
          //  printf("Datos recibidos: %s\n", (char *)data);
            xQueueSend(uart_queue, &datos, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(1000));

        /*crea una cola para enviar la informacion, */
    }
    free(data);
    vTaskDelay(pdMS_TO_TICKS(1000));
}


// funcion principal 
void app_main() {

        // Set the LEDC peripheral configuration
    

    uart_queue = xQueueCreate(QUEUE_SIZE, sizeof(uart_event_t));


      xTaskCreate(uart_receive_task, "uart_receive_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);


    ledc_init();
    
     ESP_ERROR_CHECK(gpio_set_direction(ledB, GPIO_MODE_OUTPUT));
        init_led();


    // Crear la cola para los valores ADC
    adc_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint32_t));

    if (adc_queue == NULL) {
        printf("Error al crear la cola ADC.\n");
        return;}
    


    // Crear la tarea para leer el ADC
    xTaskCreate(task_read_adc, "read_adc_task", 2048, NULL, 5, NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // Instalar el servicio ISR de GPIO
  // Crear una tarea para manejar el botón
    xTaskCreatePinnedToCore(button_task,"button_task",2048,NULL,5,NULL,0);

    // Crear una tarea para imprimir mensajes
    xTaskCreate(message_task, "message_task", 2048,  NULL,  1, NULL);
    xTaskCreate(set_led_intensity, "set_led_intensity_task", 2048, NULL, 5, NULL);

    // Crear la tarea para aumentar la intensidad de los LED
      uart_init();
    xTaskCreate(uart_receive_task, "uart_receive_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    }
















  