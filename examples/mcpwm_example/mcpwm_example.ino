/****************************************************************************************************************/
/*                                                                                                              */
/*                      Includes                                                                                 */
/*                                                                                                              */
/****************************************************************************************************************/
#include <driver/mcpwm.h>
#include <driver/pcnt.h>




/****************************************************************************************************************/
/*                                                                                                              */
/*                      Defines                                                                                 */
/*                                                                                                              */
/****************************************************************************************************************/

#define MAX_SPEED_IN_HZ 300000


#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE 20000
#define PWM_FREQUENCY (int32_t)30000   // 300 Hz for testing
#define PWM_DUTY_CYCLE 50.0 // 50% duty cycle
#define PWM_GPIO_PIN 18     // PWM output pin
#define PCNT_INPUT_PIN PWM_GPIO_PIN //25   // Separate PCNT input pin
#define PCNT_DIR_GPIO GPIO_NUM_20    // GPIO 20 for direction control


#define PCNT_MIN_MAX_THRESHOLD_UNIT_0 (int16_t)INT16_MAX//6400
#define PCNT_MIN_MAX_THRESHOLD_UNIT_1 (int16_t)300




volatile int dynamic_target_pos_i322 = PCNT_MIN_MAX_THRESHOLD_UNIT_0;
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



/****************************************************************************************************************/
/*                                                                                                              */
/*                      MCPWM related function                                                                  */
/*                                                                                                              */
/****************************************************************************************************************/
void init_mcpwm(uint8_t stepPin_, uint8_t dirPin)
{
  // Configure the MCPWM unit with frequency and duty cycle
  mcpwm_config_t pwmConfig;
  pwmConfig.frequency = 200000;//PWM_FREQUENCY;         // Set frequency to 300 Hz
  pwmConfig.cmpr_a = PWM_DUTY_CYCLE;           // Set duty cycle to 50%
  pwmConfig.cmpr_b = 0.0;                      // Unused in this example
  pwmConfig.counter_mode = MCPWM_UP_COUNTER;   // Count up mode
  pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;     // Duty mode 0

  // Initialize MCPWM unit 0, timer 0
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);
} 




/****************************************************************************************************************/
/*                                                                                                              */
/*                      Multiturn PCNT related functios                                                         */
/*                                                                                                              */
/****************************************************************************************************************/

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
  pcnt_get_event_status(PCNT_UNIT_0, &status);

  if (status & PCNT_EVT_H_LIM) {
    evt.unit = PCNT_UNIT_0;
    evt.status = PCNT_EVT_H_LIM;
    evt.timeStamp = millis();

    overflowCount++; // Increment the overflow count
    

    xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
    if (HPTaskAwoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }

  if (status & PCNT_EVT_L_LIM) {
    evt.unit = PCNT_UNIT_0;
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


void init_pcnt_multiturn(uint8_t stepPin_, uint8_t dirPin)
{
  // Initialize the PCNT module: #0
  pcnt_config_t pcntConfig_0;
  pcntConfig_0.pulse_gpio_num = stepPin_;  // PCNT input pin
  pcntConfig_0.ctrl_gpio_num = dirPin; // No control pin
  pcntConfig_0.channel = PCNT_CHANNEL_0;         // Channel 0
  pcntConfig_0.unit = PCNT_UNIT_0;                 // PCNT unit
  pcntConfig_0.pos_mode = PCNT_COUNT_INC;        // Count rising edges
  pcntConfig_0.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig_0.lctrl_mode = PCNT_MODE_REVERSE;      // Reverse the counter mode
  pcntConfig_0.hctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig_0.counter_h_lim = PCNT_MIN_MAX_THRESHOLD_UNIT_0;//INT16_MAX;        // High limit
  pcntConfig_0.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD_UNIT_0;//INT16_MIN;        // Low limit
  
  // enable overflow counter
  // See compensate overflow loss
  //pcnt_unit_config_t::accum_count

  esp_err_t pcntInitStatus = pcnt_unit_config(&pcntConfig_0);
  if (pcntInitStatus != ESP_OK) {
    Serial.printf("PCNT init failed with error: %d\n", pcntInitStatus);
  } else {
    Serial.println("PCNT initialized successfully");
  }

  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  // Activate pcnt
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // PCNT event
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

  // Register ISR handler and enable interrupts for PCNT unit
  pcnt_isr_service_install(0); // Install the PCNT interrupt service
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_intr_handler, NULL); // Attach the ISR

  // Set counter value to zero
  pcnt_counter_clear(PCNT_UNIT_0);

  // Enable the PCNT unit
  pcnt_counter_resume(PCNT_UNIT_0);

  pcnt_counter_clear(PCNT_UNIT_0);

  // Create queue for PCNT events
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
}

long readCurrentPosition()
{
  int16_t pulseCount_0 = 0;
  esp_err_t pcntReadStatus_0 = pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_0);
  long totalPulseCount = (overflowCount * PCNT_MIN_MAX_THRESHOLD_UNIT_0) + pulseCount_0;
  return totalPulseCount;
}



/****************************************************************************************************************/
/*                                                                                                              */
/*                      Control PCNT related functios                                                           */
/*                                                                                                              */
/****************************************************************************************************************/

// ISR to handle PCNT events
static void IRAM_ATTR pcnt_controll_intr_handler(void *arg) {

  // When limits are met --> interrupt fires --> stop MCPWM
  uint32_t status;
  // Retrieve the event status for the current PCNT unit
  pcnt_get_event_status(PCNT_UNIT_1, &status);

  // stop the MCPWM output when target was hit
  if (status & PCNT_EVT_H_LIM) {
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);  
  }
  if (status & PCNT_EVT_L_LIM) {
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);  
  }

}


void init_pcnt_control(uint8_t stepPin_, uint8_t dirPin)
{
  // Initialize the PCNT module: #0
  pcnt_config_t pcntConfig_0;
  pcntConfig_0.pulse_gpio_num = stepPin_;  // PCNT input pin
  pcntConfig_0.ctrl_gpio_num = dirPin; // No control pin
  pcntConfig_0.channel = PCNT_CHANNEL_0;         // Channel 0
  pcntConfig_0.unit = PCNT_UNIT_1;                 // PCNT unit
  pcntConfig_0.pos_mode = PCNT_COUNT_INC;        // Count rising edges
  pcntConfig_0.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig_0.lctrl_mode = PCNT_MODE_REVERSE;      // Keep the counter mode
  pcntConfig_0.hctrl_mode = PCNT_MODE_KEEP;      // Keep the counter mode
  pcntConfig_0.counter_h_lim = dynamic_target_pos_i322;//INT16_MAX;        // High limit
  pcntConfig_0.counter_l_lim = -dynamic_target_pos_i322;//INT16_MIN;        // Low limit

  esp_err_t pcntInitStatus = pcnt_unit_config(&pcntConfig_0);
  if (pcntInitStatus != ESP_OK) {
    Serial.printf("PCNT init failed with error: %d\n", pcntInitStatus);
  } else {
    Serial.println("PCNT initialized successfully");
  }

  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_UNIT_1, 100);
  pcnt_filter_enable(PCNT_UNIT_1);

  // Activate pcnt
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);

  // PCNT event
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

  // Register ISR handler and enable interrupts for PCNT unit
  pcnt_isr_service_install(0); // Install the PCNT interrupt service
  pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_controll_intr_handler, NULL); // Attach the ISR

  // Set counter value to zero
  pcnt_counter_clear(PCNT_UNIT_1);

  // Enable the PCNT unit
  pcnt_counter_resume(PCNT_UNIT_1);

  //pcnt_counter_clear(PCNT_UNIT_1);

  // Create queue for PCNT events
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

}




void updateTargetPosition(long targetPos_) {

  // Calculate the requested position change
  long curPos_ = readCurrentPosition();
  long requestedPositionChange_ = targetPos_ - curPos_;
  int16_t requestedPositionChange_clamped = constrain(requestedPositionChange_, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);


  // Stop the MCPWM output to avoid conflicts while reconfiguring
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);  

  //if (requestedPositionChange_ != 0)
  if (abs(requestedPositionChange_clamped) > 0)
  {
    
    // Determine new cyclic limits
    int16_t newHighLimit = abs(requestedPositionChange_clamped);
    int16_t newLowLimit = -abs(requestedPositionChange_clamped);

    //Serial.printf("Pos: %d\n", requestedPositionChange_clamped);

    // Remove existing ISR and clear interrupts
    //pcnt_isr_handler_remove(PCNT_UNIT_1);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    
    pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, newHighLimit);
    pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, newLowLimit);

    // Re-enable events and reinstall the ISR
    //pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    //pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
    //pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_controll_intr_handler, NULL);

    // Resume PCNT
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);


    // Restart the MCPWM output
    if (requestedPositionChange_ > 0)
    {
      digitalWrite(PCNT_DIR_GPIO, HIGH);  // Count up
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
    else
    {
      digitalWrite(PCNT_DIR_GPIO, LOW);  // Count up
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
  
  }
  
   
}



void setMaxSpeed(long speed)
{
  long maxSpeed = constrain(speed, 0, MAX_SPEED_IN_HZ);
}

/****************************************************************************************************************/
/*                                                                                                              */
/*                      Misc function                                                                           */
/*                                                                                                              */
/****************************************************************************************************************/










/****************************************************************************************************************/
/*                                                                                                              */
/*                                                                                                              */
/*                                                                                                              */
/****************************************************************************************************************/


void setup() {
  Serial.begin(115200);

  uint8_t stepPin = PWM_GPIO_PIN;
  uint8_t dirPin = PCNT_DIR_GPIO;


  // init MCPWM
  init_mcpwm(stepPin, dirPin);

  // init the multiturn position observer pcnt 
  init_pcnt_multiturn(stepPin, dirPin);
  init_pcnt_control(stepPin, dirPin);

  // configure pin modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, stepPin);

  // connect PCNT to GPIO pin
  gpio_iomux_in(stepPin, PCNT_SIG_CH0_IN0_IDX);
  gpio_iomux_in(stepPin, PCNT_SIG_CH0_IN1_IDX);

  // reset pcnt counters
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_1);

}



float sineFrequencyInHz = 1;
float sineAmplitudeInSteps = 1000;

static long targetPosition = 0;

void loop() {

  

  float tInSeconds = (float)millis() / 1000.0f;


  // choose target pattern
  uint8_t targetPattern_u8 = 1;
  float targetPos_fl32 = 0;
  switch (targetPattern_u8)
  {
    case 0: 
      targetPosition += 500;  // Increment target position cyclically
      if (targetPosition > 50000) targetPosition = -50000;  // Reset after reaching max range
      break;
    case 1:
      // sine wave
      targetPos_fl32 = sineAmplitudeInSteps * sin( 2.0f * M_PI * sineFrequencyInHz * tInSeconds);
      targetPosition = targetPos_fl32;
      break;
    case 2:
      // step
      if ( fmod(tInSeconds, 2) > 1)
      {targetPosition = -sineAmplitudeInSteps;}
      else
      {targetPosition = sineAmplitudeInSteps;}
      break;

  }
  

  // Update PCNT limits and target
  updateTargetPosition(targetPosition);

  delay(10);
  int16_t pulseCount_1 = 0;
  esp_err_t pcntReadStatus_0 = pcnt_get_counter_value(PCNT_UNIT_1, &pulseCount_1);

  // Read and print the pulse count
  long totalPulseCount = readCurrentPosition();

  // print routine
  //Serial.printf("target:%d,cnt1:%d,cnt2:%d\n", targetPosition, pulseCount_1, totalPulseCount);		//the first variable for plotting
  Serial.printf("target:%d,currentPos:%d\n", targetPosition, totalPulseCount);		//the first variable for plotting

  delay(10);  // Adjust delay as needed
}
