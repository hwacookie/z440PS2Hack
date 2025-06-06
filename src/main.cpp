/*
 * Copyright (c) 2025 hwacookie@coregames.de
 * All rights reserved. GPL-3 license applies.
 *
 * Sketch for 5V Arduino Pro Mini (ATmega168P, 16MHz)
 * - Emulates PS/2 Keyboard to send a specific key sequence at boot.
 * - Collects sensor data (DHT22, PIR, 4x Thermistors) and sends as JSON datagram.
 * - Powers from PS/2 port (ensure port can supply enough current for all components).
 * - Includes logging (using F() macro for RAM saving) and LED blink during initial delay.
 */

#include <Arduino.h>
#include <ps2dev.h> // Harvie's PS/2 device emulation library
#include <DHT.h>    // Adafruit DHT Sensor Library
#include <math.h>   // For log() in thermistor calculation

// Forward declaration for PS/2 keyboard reset callback
void keyboardResetCallback();

volatile bool startupSequenceRunning = false;

// --- Pin Definitions ---
#define PS2_CLK_PIN 2  // CLK pin
#define PS2_DATA_PIN 3 // DATA pin
#define DHT_PIN 9      // DHT22 data pin connected to Arduino D9
#define DHT_TYPE DHT22
#define PIR_PIN 4 // Digital pin for PIR sensor output
#define THERMISTOR_PIN_1 A0
#define THERMISTOR_PIN_2 A1
#define THERMISTOR_PIN_3 A2
#define THERMISTOR_PIN_4 A3
#define BOOT_LED_PIN LED_BUILTIN

// LED lock status pins (default to LED_BUILTIN)
#define SCROLL_LOCK_LED_PIN LED_BUILTIN // bit 0: Scroll Lock
#define NUM_LOCK_LED_PIN LED_BUILTIN    // bit 1: Num Lock
#define CAPS_LOCK_LED_PIN LED_BUILTIN   // bit 2: Caps Lock
#define KEYBOARD_LED_PIN LED_BUILTIN    // the master LED pin (used if we have jusft one LED for all keyboard status)

// --- Thermistor Configuration ---
#define FIXED_RESISTOR 10000.0
#define NOMINAL_RESISTANCE 10000.0
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3950.0
#define ADC_MAX 1023.0
#define ADC_VREF 5.0

// --- Debugging Configuration ---
// Set to "Serial"" to enable debug output, or comment-out to disable debug output
//#define DEBUG_LOG_DEVICE Serial

// --- Global Objects ---
PS2dev keyboard(PS2_CLK_PIN, PS2_DATA_PIN);
DHT dht(DHT_PIN, DHT_TYPE);
bool keyboard_initialized = false; // will be set to true once we receive the first reset_callback from the PS/2 keyboard

// --- Global Variables for Sensor Data & Configuration ---
#define SENSOR_READ_INTERVAL_MILLISECONDS 5000
unsigned long previousSensorReadMillis = 0;
unsigned char kbd_leds = 0;

// Variables to hold current sensor values for JSON datagram
float currentHumidity = NAN;
float currentDhtTempC = NAN;
float currentThermistorTemps[4] = {NAN, NAN, NAN, NAN};
bool isMotionCurrentlyActive = false; // Renamed from motionDetectedState for clarity in JSON

// PIR State (internal for edge detection)
bool internalPirState = LOW;
unsigned long lastMotionTriggerTime = 0;
#define MOTION_DEBOUNCE_DELAY 5000 // Ignore PIR re-triggers for 5 seconds

// --- Key Sequence Definition ---
enum KeyActionType
{
    NORMAL_KEY,
    SPECIAL_KEY
};
struct TimedKeyAction
{
    int delaySeconds;
    KeyActionType keyType;
    union
    {
        PS2dev::ScanCodes normalKey;
        PS2dev::SpecialScanCodes specialKey;
    } key;
    const char *keyName;
};
const TimedKeyAction keySequence[] = {

    // This I used for a "more complicated" boot sequence
    /*
    {11, NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::F8},         "F8"},
    {11, SPECIAL_KEY, {.specialKey = PS2dev::SpecialScanCodes::DOWN_ARROW}, "ARROW DOWN"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ENTER},      "ENTER"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ESCAPE},     "ESCAPE"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ENTER},      "ENTER"},
    */
    // Trying four times with a 10 second delay inbetween
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"}};
const int numKeyActions = sizeof(keySequence) / sizeof(keySequence[0]);

// Forward declarations of functions
void keyboardResetCallback();

void handle_leds()
{
// Handle individual lock LEDs
#ifdef DEBUG_LOG_DEVICE

    DEBUG_LOG_DEVICE.print(F("LOG: handle_leds() - Scroll Lock:"));
    DEBUG_LOG_DEVICE.print((kbd_leds & 0x01) ? '1' : '0');
    DEBUG_LOG_DEVICE.print(F(" Num Lock:"));
    DEBUG_LOG_DEVICE.print((kbd_leds & 0x02) ? '1' : '0');
    DEBUG_LOG_DEVICE.print(F(" Caps Lock:"));
    DEBUG_LOG_DEVICE.print((kbd_leds & 0x04) ? '1' : '0');
    DEBUG_LOG_DEVICE.println();
#endif
    // Set the state of the lock LEDs based on kbd_leds bits
    digitalWrite(SCROLL_LOCK_LED_PIN, (kbd_leds & 0x01) ? HIGH : LOW);
    digitalWrite(NUM_LOCK_LED_PIN, (kbd_leds & 0x02) ? HIGH : LOW);
    digitalWrite(CAPS_LOCK_LED_PIN, (kbd_leds & 0x04) ? HIGH : LOW);


    digitalWrite(KEYBOARD_LED_PIN, (kbd_leds) ? HIGH : LOW);

}

// --- Sensor Update Functions ---

/**
 * @brief Reads data from the DHT22 sensor and updates global variables.
 *
 * This function reads the humidity and temperature from the DHT22 sensor
 * connected to the configured pin. It updates the global `currentHumidity`
 * and `currentDhtTempC` variables. Includes basic error logging if the
 * sensor reading fails.
 */
void updateDHTData()
{
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: updateDHTData() - Reading DHT22 sensor."));
#endif
    currentHumidity = dht.readHumidity();
    currentDhtTempC = dht.readTemperature();


#ifdef DEBUG_LOG_DEVICE
    if (isnan(currentHumidity) || isnan(currentDhtTempC))
    {
        DEBUG_LOG_DEVICE.println(F("LOG: updateDHTData() - Failed to read from DHT sensor!"));
    }
    else
    {
        DEBUG_LOG_DEVICE.println(F("LOG: updateDHTData() - DHT22 data updated."));
    }
#endif
}

/**
 * @brief Reads an analog value from a thermistor and converts it to temperature in Celsius.
 *
 * THIS IS STILL A WORK IN PROGRESS, IT IS NOT YET TESTED!
 *
 * This function reads the analog voltage from a thermistor connected to the specified
 * analog pin, calculates its resistance, and then uses the Steinhart-Hart equation
 * to convert the resistance to temperature in Celsius. Includes logging for ADC value
 * and calculated temperature, and handles potential unstable ADC readings or invalid resistance values.
 *
 * @param analogPin The analog pin the thermistor is connected to.
 * @param thermistorName A descriptive name for the thermistor (used in logging).
 * @return The temperature in Celsius, or NAN if the reading is unstable or invalid.
 */
float readThermistorValue(int analogPin, const char *thermistorName)
{
    int adcValue = analogRead(analogPin);
    if (adcValue < 10 || adcValue > (ADC_MAX - 10))
    {
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.print(F("LOG: readThermistorValue() - Unstable ADC for "));
        DEBUG_LOG_DEVICE.print(thermistorName);
        DEBUG_LOG_DEVICE.print(F(": "));
        DEBUG_LOG_DEVICE.println(adcValue);
#endif
        return NAN;
    }
    float voltage = (adcValue / ADC_MAX) * ADC_VREF;
    float resistance = (FIXED_RESISTOR * voltage) / (ADC_VREF - voltage);
    if (resistance <= 0)
    {
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.print(F("LOG: readThermistorValue() - Invalid resistance for "));
        DEBUG_LOG_DEVICE.print(thermistorName);
        DEBUG_LOG_DEVICE.print(F(": "));
        DEBUG_LOG_DEVICE.println(resistance);
#endif
        return NAN;
    }
    float steinhart = log(resistance / NOMINAL_RESISTANCE) / B_COEFFICIENT + 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    if (steinhart == 0)
    {
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.print(F("LOG: readThermistorValue() - Steinhart error for "));
        DEBUG_LOG_DEVICE.println(thermistorName);
#endif
        return NAN;
    }
    float tempC = (1.0 / steinhart) - 273.15;
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.print(F("LOG: readThermistorValue() - "));
    DEBUG_LOG_DEVICE.print(thermistorName);
    DEBUG_LOG_DEVICE.print(F(" ADC: "));
    DEBUG_LOG_DEVICE.print(adcValue);
    DEBUG_LOG_DEVICE.print(F(", Temp C: "));
    DEBUG_LOG_DEVICE.println(tempC, 2);
#endif
    return tempC;
}

/**
 * @brief Updates the state of the PIR motion sensor.
 *
 * THIS IS STILL A WORK IN PROGRESS, IT IS NOT YET TESTED!
 *
 * This function reads the digital state of the PIR sensor pin. It detects
 * the rising edge (motion detected) and falling edge (motion ended) of the
 * signal. A debounce delay is applied to the motion detection to prevent
 * multiple triggers from a single event. Updates the global `isMotionCurrentlyActive`
 * variable and logs state changes.
 */
void updatePIRState()
{
    int currentPirReading = digitalRead(PIR_PIN);
    unsigned long currentTime = millis();

    if (currentPirReading == HIGH)
    {
        if (internalPirState == LOW && (currentTime - lastMotionTriggerTime > MOTION_DEBOUNCE_DELAY))
        {
#ifdef DEBUG_LOG_DEVICE
            DEBUG_LOG_DEVICE.println(F("LOG: updatePIRState() - Motion DETECTED!"));
#endif
            isMotionCurrentlyActive = true;
            lastMotionTriggerTime = currentTime;
        }
        internalPirState = HIGH; // Update internal state regardless of debounce for edge detection
    }
    else
    { // currentPirReading == LOW
        if (internalPirState == HIGH)
        {
#ifdef DEBUG_LOG_DEVICE
            DEBUG_LOG_DEVICE.println(F("LOG: updatePIRState() - Motion ended."));
#endif
            isMotionCurrentlyActive = false; // Set to false as soon as signal goes low
        }
        internalPirState = LOW;
    }
}

/**
 * @brief Sends a JSON datagram containing current sensor data over the serial connection.
 *
 * This function formats the current sensor readings (humidity, DHT temperature,
 * thermistor temperatures, and PIR motion state) into a JSON string and sends
 * it over the serial port. Handles NAN values by outputting "null" in the JSON.
 */
void sendDatagram()
{
    Serial.print(F("{\"humidity\":"));
    if (isnan(currentHumidity))
        Serial.print(F("null"));
    else
        Serial.print(currentHumidity, 1);

    Serial.print(F(",\"temp_dht\":"));
    if (isnan(currentDhtTempC))
        Serial.print(F("null"));
    else
        Serial.print(currentDhtTempC, 1);

    Serial.print(F(",\"thermistors\":["));
    for (int i = 0; i < 4; i++)
    {
        if (isnan(currentThermistorTemps[i]))
            Serial.print(F("null"));
        else
            Serial.print(currentThermistorTemps[i], 1);
        if (i < 3)
            Serial.print(F(","));
    }
    Serial.print(F("]"));

    Serial.print(F(",\"motion\":"));
    Serial.print(isMotionCurrentlyActive ? F("true") : F("false"));

    Serial.println(F("}"));
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: sendDatagram() - Datagram sent."));
#endif
}

// --- Setup and Loop ---

/**
 * @brief Performs an active delay with optional LED blinking.
 *
 * This function pauses execution for a specified number of seconds. If `blinkLed`
 * is true, the built-in LED will blink during the delay, starting slowly and
 * increasing in frequency as the delay approaches its end. PS/2 keyboard handling
 * is performed during the delay to maintain communication responsiveness.
 *
 * @param seconds The duration of the delay in seconds.
 * @param blinkLed If true, the built-in LED will blink during the delay with increasing frequency.
 */
void performActiveDelay(int seconds, bool blinkLed)
{
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.print(F("LOG: performActiveDelay() - Starting delay of "));
    DEBUG_LOG_DEVICE.print(seconds);
    DEBUG_LOG_DEVICE.println(F(" seconds..."));
#endif

    if (!blinkLed)
    {
        // Existing non-blinking delay logic
        for (int i = 0; i < seconds * 100; ++i)
        {
            if (keyboard.keyboard_handle(&kbd_leds))
            {
                handle_leds();
            }
            delay(10);
        }
    }
    else
    {
        int total_seconds = seconds;
        if (total_seconds <= 0)
            total_seconds = 1;

        for (int i = total_seconds; i > 0; i--)
        { // Loop through each second
#ifdef DEBUG_LOG_DEVICE
            DEBUG_LOG_DEVICE.print(F("LOG: performActiveDelay() - Countdown: "));
            DEBUG_LOG_DEVICE.print(i);
            DEBUG_LOG_DEVICE.println(F(" seconds..."));
#endif

            // Calculate blinks per second (N)
            float N_float;
            if (total_seconds == 1)
            {
                N_float = 1.0;
            }
            else
            {
                // Linear interpolation from 1 blink/sec (at start) to 10 blinks/sec (at end)
                N_float = 1.0 + 9.0 * (float)(total_seconds - i) / (float)(total_seconds - 1);
            }
            int N = max(1, (int)round(N_float)); // Ensure at least 1 blink per second

            float cycle_duration_ms = 1000.0 / N;
            float on_duration_ms = cycle_duration_ms / 2.0;
            float off_duration_ms = cycle_duration_ms / 2.0;

            unsigned long start_second_millis = millis();

            for (int blink_count = 0; blink_count < N; ++blink_count)
            {
                // Turn LED ON
                digitalWrite(BOOT_LED_PIN, HIGH);
                // Delay for on_duration_ms while handling keyboard
                unsigned long current_blink_start_millis = millis();
                while (millis() - current_blink_start_millis < on_duration_ms)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds();
                    }
                    delay(1); // Yield
                }

                // Turn LED OFF
                digitalWrite(BOOT_LED_PIN, LOW);
                // Delay for off_duration_ms while handling keyboard
                current_blink_start_millis = millis(); // Reuse variable
                while (millis() - current_blink_start_millis < off_duration_ms)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds();
                    }
                    delay(1); // Yield
                }
            }
            // Ensure the full second has passed before moving to the next second
            // This handles potential drift from the small delays
            unsigned long end_second_millis = millis();
            long remaining_ms_in_second = 1000 - (end_second_millis - start_second_millis);
            if (remaining_ms_in_second > 0)
            {
                unsigned long delay_start = millis();
                while (millis() - delay_start < remaining_ms_in_second)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds(); // Handle keyboard LEDs
                    }
                    delay(1); // Yield
                }
            }
        }
        digitalWrite(BOOT_LED_PIN, LOW); // Ensure off at the end
    }
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: performActiveDelay() - Delay finished."));
#endif
}

/**
 * @brief Initializes Arduino, PS/2 keyboard, sensors, and sends initial data.
 *
 * This function is called once when the Arduino powers up or resets. It performs the following actions:
 * 1. Initializes the serial communication for logging.
 * 2. Initializes the PS/2 keyboard emulation.
 * 3. Sends a predefined sequence of key presses with delays in between.
 * 4. Initializes the DHT22 temperature and humidity sensor.
 * 5. Initializes the PIR motion sensor, including a settling delay.
 * 6. Reads initial values from all connected sensors (DHT, PIR, Thermistors).
 * 7. Sends an initial JSON datagram with the first set of sensor readings.
 * 8. Sets up the timing for periodic sensor reads in the main loop.
 */

void setup()
{
    pinMode(BOOT_LED_PIN, OUTPUT);
    digitalWrite(BOOT_LED_PIN, LOW);
    Serial.begin(115200);
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Sketch starting. Serial initialized."));
#endif

#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Initializing PS/2 keyboard emulation..."));
#endif
    keyboard.keyboard_init(keyboardResetCallback);
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - PS/2 keyboard emulation initialized."));
#endif
}

void initSensors()
{
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Initializing DHT22 sensor (post-keyboard sequence)..."));
#endif
    dht.begin();
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - DHT22 sensor initialization complete."));
#endif

#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Initializing PIR sensor (post-keyboard sequence)..."));
#endif
    pinMode(PIR_PIN, INPUT);
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - PIR pin initialized. Waiting for PIR to settle..."));
#endif
    performActiveDelay(15, false); // PIR settling time
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - PIR sensor settling time complete."));
#endif
}

/**
 * @brief Main sketch loop: handles keyboard, PIR, and periodic sensor reads/sends.
 *
 * This function performs the following actions continuously:
 * 1. Handles PS/2 keyboard communication to maintain responsiveness.
 * 2. Updates the state of the PIR motion sensor.
 * 3. Checks if it's time to read sensor data based on `SENSOR_READ_INTERVAL_MILLISECONDS`.
 *    If it is time:
 *    a. Updates DHT22 sensor data (humidity and temperature).
 *    b. Reads temperature from all four thermistors.
 *    c. Sends a JSON datagram containing all current sensor readings over DEBUG_LOG_DEVICE.
 * 4. Includes a small delay to yield processing time.
 */
void loop()
{
    if (keyboard.keyboard_handle(&kbd_leds))
    {
        handle_leds();
    }

    // only run the loop if the keyboard is initialized and we are not in the startup sequence
    if (keyboard_initialized && !startupSequenceRunning)
    {
        updatePIRState(); // Continuously check PIR state

        unsigned long currentMillis = millis();
        if (currentMillis - previousSensorReadMillis >= SENSOR_READ_INTERVAL_MILLISECONDS)
        {
            previousSensorReadMillis = currentMillis;

#ifdef DEBUG_LOG_DEVICE
            DEBUG_LOG_DEVICE.println(F("LOG: loop() - Time to update sensor data and send datagram."));
#endif
            updateDHTData();
            // PIR state is already updated by updatePIRState() called earlier in loop
            currentThermistorTemps[0] = readThermistorValue(THERMISTOR_PIN_1, "Thermistor 1");
            currentThermistorTemps[1] = readThermistorValue(THERMISTOR_PIN_2, "Thermistor 2");
            currentThermistorTemps[2] = readThermistorValue(THERMISTOR_PIN_3, "Thermistor 3");
            currentThermistorTemps[3] = readThermistorValue(THERMISTOR_PIN_4, "Thermistor 4");

            sendDatagram();
        }
    }
    delay(1);
}

// reset callback handling

void runStartupSequence()
{
    for (int i = 0; i < numKeyActions; ++i)
    {
        const TimedKeyAction &action = keySequence[i];
        performActiveDelay(action.delaySeconds, action.delaySeconds > 1);
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.print(F("LOG: runStartupSequence() - Sending key: "));
#endif
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.println(action.keyName);
#endif
        if (action.keyType == NORMAL_KEY)
            keyboard.keyboard_mkbrk(action.key.normalKey);
        else if (action.keyType == SPECIAL_KEY)
            keyboard.keyboard_special_mkbrk(action.key.specialKey);
        for (int d = 0; d < 10; ++d)
        {
            if (keyboard.keyboard_handle(&kbd_leds))
            {
                handle_leds(); // Handle keyboard LEDs
            }
            delay(10);
        }
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.print(F("LOG: runStartupSequence() - Key "));
        DEBUG_LOG_DEVICE.print(action.keyName);
        DEBUG_LOG_DEVICE.println(F(" sent."));
#endif
    }
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: runStartupSequence() - Key press sequence complete."));
#endif

    initSensors();
}

void keyboardResetCallback()
{
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: keyboardResetCallback() - Running startup sequence due to keyboard reset."));
#endif
    keyboard_initialized = true; // Reset keyboard state
    startupSequenceRunning = true;
    runStartupSequence();
    startupSequenceRunning = false;
}
