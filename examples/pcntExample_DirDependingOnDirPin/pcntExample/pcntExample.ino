#include <driver/pcnt.h>

#define PCNT_UNIT PCNT_UNIT_0        // Use Pulse Counter Unit 0
#define PCNT_INPUT_GPIO GPIO_NUM_19  // GPIO 19 for pulse input
#define PCNT_DIR_GPIO GPIO_NUM_20    // GPIO 20 for direction control
#define PULSE_TARGET_COUNT 100       // Set the target count for the interrupt

// Global flag to indicate that the target count was reached
volatile bool target_reached = false;

// Interrupt Service Routine for PCNT
void IRAM_ATTR pcnt_isr_handler(void* arg) {
    // Set flag to indicate that the target count was reached
    target_reached = true;
    
    // Clear the interrupt status to acknowledge it
    pcnt_counter_clear(PCNT_UNIT); // Clear the counter to reset to zero
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Configure PCNT unit
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = PCNT_INPUT_GPIO;  // GPIO for pulse input
    pcnt_config.ctrl_gpio_num = PCNT_DIR_GPIO;     // GPIO for direction control
    pcnt_config.channel = PCNT_CHANNEL_0;          // Use channel 0
    pcnt_config.unit = PCNT_UNIT;                  // Use PCNT_UNIT
    pcnt_config.pos_mode = PCNT_COUNT_INC;         // Count on positive edge
    pcnt_config.neg_mode = PCNT_COUNT_DIS;         // Ignore negative edge
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;    // Reverse count when control is low
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;       // Keep counting up when control is high
    pcnt_config.counter_h_lim = PULSE_TARGET_COUNT; // Set high limit

    // Initialize PCNT unit with the configuration
    pcnt_unit_config(&pcnt_config);

    // Set counter to zero initially
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    // Set up interrupt on high limit
    //pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    //pcnt_isr_register(pcnt_isr_handler, NULL, 0, NULL);
    //pcnt_intr_enable(PCNT_UNIT);

    // Configure GPIOs
    pinMode(PCNT_INPUT_GPIO, OUTPUT);
    pinMode(PCNT_DIR_GPIO, OUTPUT);
}

void loop() {
    // Check if target count was reached
    if (target_reached) {
        Serial.println("Target count reached!");
        target_reached = false;  // Reset the flag
    }

    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

    // Toggle direction based on pulse count
    if (pulse_count <= -(PULSE_TARGET_COUNT-1)) {
        digitalWrite(PCNT_DIR_GPIO, HIGH);  // Count up
    }

    if (pulse_count >= (PULSE_TARGET_COUNT-1)) {
        digitalWrite(PCNT_DIR_GPIO, LOW);   // Count down
    }

    // Generate a pulse on GPIO 19
    digitalWrite(PCNT_INPUT_GPIO, HIGH);
    delay(10);  // Short pulse duration
    digitalWrite(PCNT_INPUT_GPIO, LOW);
    delay(10);  // Delay before next pulse

    Serial.println(pulse_count);
}
