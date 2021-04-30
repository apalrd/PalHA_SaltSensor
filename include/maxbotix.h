/* Code for Maxbotix ultrasonic sensor */
#include <stdint.h>


/* UART number to use (RX only) */
#define MAXBOTIX_UART_NUM UART_NUM_1

/* UART pin to use (default 36) */
#define MAXBOTIX_UART_PIN 36

/* Number of samples in the sample buffer */
#define MAXBOTIX_SAMPLE_BUFFER_SIZE 128

/* Maximum age of samples in milliseconds before buffer is reset */
#define MAXBOTIX_SAMPLE_AGE_MAX 2000


/* Function prototypes */
void maxbotix_init(void);
uint16_t maxbotix_get_latest(void);
float maxbotix_get_median(float pct,int16_t min_count,int16_t max_count,int16_t *act_count);