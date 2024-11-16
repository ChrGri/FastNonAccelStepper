#include "driver/pcnt.h"
#include "esp_log.h"

// Define the GPIO pin for pulse input
#define PCNT_INPUT_SIG_IO 18

// Define the PCNT unit to use
#define PCNT_UNIT PCNT_UNIT_0

// Define the high limit for the PCNT counter
#define PCNT_H_LIM_VAL 5000

// Variable to store the multiturn count
volatile int multiturn = 0;





// PCNT interrupt handler
static void IRAM_ATTR pcnt_isr_handler(void *arg) {
  uint32_t intr_status ;//= PCNT.int_st.val;

  pcnt_get_event_status(PCNT_UNIT, &intr_status);

  if (intr_status & PCNT_EVT_H_LIM) {
    multiturn++; // Increment the multiturn count
    //PCNT.int_clr.val = (1 << PCNT_UNIT); // Clear the interrupt flag
    pcnt_counter_clear(PCNT_UNIT);

    Serial.print("Multiturn count: ");
    Serial.println(multiturn);  
  }

}

void setup() {
  Serial.begin(115200);



pcnt_config_t pcntConfig;
  pcntConfig.pulse_gpio_num = PCNT_INPUT_SIG_IO;  // PCNT input pin
  pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED; // No control pin
  pcntConfig.channel = PCNT_CHANNEL_0;         // Channel 0
  pcntConfig.unit = PCNT_UNIT;                 // PCNT unit
  pcntConfig.pos_mode = PCNT_COUNT_INC;        // Count rising edges
  pcntConfig.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig.lctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig.counter_h_lim = PCNT_H_LIM_VAL;//INT16_MAX;        // High limit
  pcntConfig.counter_l_lim = -PCNT_H_LIM_VAL;//INT16_MIN;        // Low limit



  // Initialize PCNT unit
  pcnt_unit_config(&pcntConfig);

  // Configure and enable the interrupt 
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT, pcnt_isr_handler, NULL);
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM); // Enable interrupt on high limit

  // Start PCNT unit
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  pinMode(PCNT_INPUT_SIG_IO, OUTPUT);
  //pinMode(PCNT_DIR_GPIO, OUTPUT);

}

void loop() {
  // Print the multiturn count
  //Serial.print("Multiturn count: ");
  //Serial.println(multiturn);
  //delay(1000); 

  int16_t pulse_count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

  if ((pulse_count % 100) == 0)
  {
    Serial.println(pulse_count);
  }

  
  // Toggle direction based on pulse count to keep counting up and down
  digitalWrite(PCNT_INPUT_SIG_IO, HIGH);  // Count up
  delay(1);
  digitalWrite(PCNT_INPUT_SIG_IO, LOW);   // Count down
  delay(1);

}