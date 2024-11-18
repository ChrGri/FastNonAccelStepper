#include <driver/mcpwm.h>
#include <driver/pcnt.h>

#define PWM_FREQUENCY (int32_t)300000   // 300 Hz for testing
#define PWM_DUTY_CYCLE 50.0 // 50% duty cycle
#define PWM_GPIO_PIN 18     // PWM output pin
#define PCNT_INPUT_PIN PWM_GPIO_PIN //25   // Separate PCNT input pin
#define PCNT_DIR_GPIO GPIO_NUM_20    // GPIO 20 for direction control

#define PCNT_UNIT PCNT_UNIT_0 // PCNT unit to use
#define PCNT_MIN_MAX_THRESHOLD_UNIT_0 (int16_t)INT16_MAX//6400
#define PCNT_MIN_MAX_THRESHOLD_UNIT_1 (int16_t)300

bool pwmEnabled = true; // Flag to toggle PWM output


portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;


volatile int overflowCount = 0; // Variable to track overflows


// Queue to handle pulse counter events
xQueueHandle pcnt_evt_queue;

// Structure to pass data from ISR to the main loop
typedef struct {
  int unit;              // PCNT unit that caused the interrupt
  uint32_t status;       // Event status
  unsigned long timeStamp; // Timestamp of the interrupt
} pcnt_evt_t;


/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
// ISR to handle PCNT events
static void IRAM_ATTR pcnt_intr_handler(void *arg) {

  //pcnt_counter_clear(PCNT_UNIT_0); // Reset the counter 

  portENTER_CRITICAL_ISR(&timer_mux); 

  uint32_t status;
  pcnt_evt_t evt;
  portBASE_TYPE HPTaskAwoken = pdFALSE;

  // Retrieve the event status for the current PCNT unit
  pcnt_get_event_status(PCNT_UNIT, &status);

  if (status & PCNT_EVT_H_LIM) {
    evt.unit = PCNT_UNIT;
    evt.status = PCNT_EVT_H_LIM;
    evt.timeStamp = millis();

    overflowCount++; // Increment the overflow count
    

    xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
    if (HPTaskAwoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }

  if (status & PCNT_EVT_L_LIM) {
    evt.unit = PCNT_UNIT;
    evt.status = PCNT_EVT_L_LIM;
    evt.timeStamp = millis();

    overflowCount--; // Increment the overflow count
    //pcnt_counter_clear(PCNT_UNIT_0); // Reset the counter 

    xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
    if (HPTaskAwoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }

  //PCNT.int_clr.val = BIT(PCNT_UNIT_0);

  //pcnt_event_disable(PCNT_UNIT, PCNT_EVT_H_LIM);
  //pcnt_event_disable(PCNT_UNIT, PCNT_EVT_L_LIM);
  portEXIT_CRITICAL_ISR(&timer_mux);

}



void setup() {
  Serial.begin(115200);

  

  // Configure the MCPWM unit with frequency and duty cycle
  mcpwm_config_t pwmConfig;
  pwmConfig.frequency = 200000;//PWM_FREQUENCY;         // Set frequency to 300 Hz
  pwmConfig.cmpr_a = PWM_DUTY_CYCLE;           // Set duty cycle to 50%
  pwmConfig.cmpr_b = 0.0;                      // Unused in this example
  pwmConfig.counter_mode = MCPWM_UP_COUNTER;   // Count up mode
  pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;     // Duty mode 0

  // Initialize MCPWM unit 0, timer 0
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);

  // Initialize the PCNT module: #0
  pcnt_config_t pcntConfig_0;
  pcntConfig_0.pulse_gpio_num = PCNT_INPUT_PIN;  // PCNT input pin
  pcntConfig_0.ctrl_gpio_num = PCNT_DIR_GPIO; // No control pin
  pcntConfig_0.channel = PCNT_CHANNEL_0;         // Channel 0
  pcntConfig_0.unit = PCNT_UNIT_0;                 // PCNT unit
  pcntConfig_0.pos_mode = PCNT_COUNT_INC;        // Count rising edges
  pcntConfig_0.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig_0.lctrl_mode = PCNT_MODE_REVERSE;      // Keep the counter mode
  pcntConfig_0.hctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig_0.counter_h_lim = PCNT_MIN_MAX_THRESHOLD_UNIT_0;//INT16_MAX;        // High limit
  pcntConfig_0.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD_UNIT_0;//INT16_MIN;        // Low limit


  // Initialize the PCNT module: #1
  /*pcnt_config_t pcntConfig_1;
  pcntConfig_1.pulse_gpio_num = PCNT_INPUT_PIN;  // PCNT input pin
  pcntConfig_1.ctrl_gpio_num = PCNT_PIN_NOT_USED; // No control pin
  pcntConfig_1.channel = PCNT_CHANNEL_0;         // Channel 0
  pcntConfig_1.unit = PCNT_UNIT_1;                 // PCNT unit
  pcntConfig_1.pos_mode = PCNT_COUNT_INC;        // Count rising edges
  pcntConfig_1.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig_1.lctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig_1.hctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig_1.counter_h_lim = PCNT_MIN_MAX_THRESHOLD_UNIT_1;//INT16_MAX;        // High limit
  pcntConfig_1.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD_UNIT_1;//INT16_MIN;        // Low limit

  */
  esp_err_t pcntInitStatus = pcnt_unit_config(&pcntConfig_0);
  //pcntInitStatus = pcnt_unit_config(&pcntConfig_1);


  if (pcntInitStatus != ESP_OK) {
    Serial.printf("PCNT init failed with error: %d\n", pcntInitStatus);
  } else {
    Serial.println("PCNT initialized successfully");
  }


  


  
  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  //pcnt_set_filter_value(PCNT_UNIT_1, 100);
  //pcnt_filter_enable(PCNT_UNIT_1);


  // Set counter value to zero
  pcnt_counter_clear(PCNT_UNIT_0);
  //pcnt_counter_clear(PCNT_UNIT_1);


  
  // Enable the PCNT unit
  pcnt_counter_resume(PCNT_UNIT_0);
  //pcnt_counter_resume(PCNT_UNIT_1);




  // PCNT event
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

  pcnt_isr_service_install(0); // Install the PCNT interrupt service
  pcnt_isr_handler_add(PCNT_UNIT, pcnt_intr_handler, NULL); // Attach the ISR

  
  /* Register ISR handler and enable interrupts for PCNT unit */
  //pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
  //pcnt_intr_enable(PCNT_UNIT);

  

  


  digitalWrite(PCNT_INPUT_PIN, LOW);
  digitalWrite(PCNT_DIR_GPIO, LOW);
  pinMode(PCNT_INPUT_PIN, OUTPUT);
  pinMode(PCNT_DIR_GPIO, OUTPUT);



  // Initialize the MCPWM GPIO pin
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_GPIO_PIN);
  

  gpio_iomux_in(PWM_GPIO_PIN, PCNT_SIG_CH0_IN0_IDX);


  // Create queue for PCNT events
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

  pcnt_counter_clear(PCNT_UNIT);

}



void loop() {
  // Toggle the MCPWM output every second
  if (pwmEnabled) {
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0); // Start the PWM
    //Serial.println("PWM Enabled");
  } else {
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);  // Stop the PWM
    //Serial.println("PWM Disabled");
  }

  // Read and print the pulse count
  int16_t pulseCount_0 = 0;
  int16_t pulseCount_1 = 0;
  esp_err_t pcntReadStatus_0 = pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_0);
  //esp_err_t pcntReadStatus_1 = pcnt_get_counter_value(PCNT_UNIT_1, &pulseCount_1);

  long totalPulseCount = (overflowCount * PCNT_MIN_MAX_THRESHOLD_UNIT_0) + pulseCount_0; 


  Serial.printf("Pulse Count: %d,    %d,    %d\n", pulseCount_0, overflowCount, totalPulseCount);



  // Check PCNT event queue
  /*pcnt_evt_t evt;
  if (xQueueReceive(pcnt_evt_queue, &evt, 0)) {
    Serial.printf("PCNT Event: Unit=%d, Status=%d, Timestamp=%lu\n",
                  evt.unit, evt.status, evt.timeStamp);
  }*/

  

  if (totalPulseCount <= -50000) {
        digitalWrite(PCNT_DIR_GPIO, HIGH);  // Count up
    }

    /*if (totalPulseCount >= 50000) {
        digitalWrite(PCNT_DIR_GPIO, LOW);  // Count up
    }*/

    
  // Clear the counter for the next measurement
  //pcnt_counter_clear(PCNT_UNIT);

  pwmEnabled = !pwmEnabled;  // Toggle the flag
  delay(100);               // Wait for 1 second
}