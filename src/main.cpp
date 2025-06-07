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

// Forward declaration for PS/2 keyboard reset callback
void keyboardResetCallback();

void keyboard_handle();

// --- Pin Definitions ---
#define PS2_CLK_PIN 2  // CLK pin
#define PS2_DATA_PIN 3 // DATA pin
#define BOOT_LED_PIN LED_BUILTIN

// LED lock status pins (default to LED_BUILTIN)
#define SCROLL_LOCK_LED_PIN LED_BUILTIN // bit 0: Scroll Lock
#define NUM_LOCK_LED_PIN LED_BUILTIN    // bit 1: Num Lock
#define CAPS_LOCK_LED_PIN LED_BUILTIN    // bit 2: Caps Lock
#define KEYBOARD_LED_PIN LED_BUILTIN    // the master LED pin (used if we have jusft one LED for all keyboard status)

// --- Debugging Configuration ---
// Set to "Serial"" to enable debug output, or comment-out to disable debug output
#define DEBUG_LOG_DEVICE Serial

// --- Global Objects ---
PS2dev keyboard(PS2_CLK_PIN, PS2_DATA_PIN);

unsigned char kbd_leds = 0;



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
    {13, NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::F8},         "F8"},
    {15, SPECIAL_KEY, {.specialKey = PS2dev::SpecialScanCodes::DOWN_ARROW}, "ARROW DOWN"},
    {10,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ENTER},      "ENTER"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ESCAPE},     "ESCAPE"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ENTER},      "ENTER"},
    {3,  NORMAL_KEY,  {.normalKey = PS2dev::ScanCodes::ENTER},      "ENTER"},
    */
    // Trying four times with a 10 second delay inbetween
    
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"},
    {10, NORMAL_KEY, {.normalKey = PS2dev::ScanCodes::ENTER}, "ENTER"}
    
};

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



/**
 * @brief Perform an active delay with optional LED blinking.
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
                unsigned long keyboard_handle_timeout = millis();
                while (millis() - current_blink_start_millis < on_duration_ms)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds();
                        keyboard_handle_timeout = millis(); // Reset timeout on successful handle
                    } else if (millis() - keyboard_handle_timeout > 100) { // Timeout after 100ms
                        break; // Break if keyboard_handle is stuck
                    }
                    delay(1); // Yield
                }

                // Turn LED OFF
                digitalWrite(BOOT_LED_PIN, LOW);
                // Delay for off_duration_ms while handling keyboard
                current_blink_start_millis = millis(); // Reuse variable
                keyboard_handle_timeout = millis(); // Reset timeout for the next loop
                while (millis() - current_blink_start_millis < off_duration_ms)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds();
                        keyboard_handle_timeout = millis(); // Reset timeout on successful handle
                    } else if (millis() - keyboard_handle_timeout > 100) { // Timeout after 100ms
                        break; // Break if keyboard_handle is stuck
                    }
                    delay(1); // Yield
                }
            }
            // Ensure the full second has passed before moving to the next second
            // This handles potential drift from the small delays
            unsigned long end_second_millis = millis();
            unsigned long remaining_ms_in_second = 1000 - (end_second_millis - start_second_millis);
            if (remaining_ms_in_second > 0)
            {
                unsigned long delay_start = millis();
                unsigned long keyboard_handle_timeout = millis(); // Reset timeout for the final delay
                while (millis() - delay_start < remaining_ms_in_second)
                {
                    if (keyboard.keyboard_handle(&kbd_leds))
                    {
                        handle_leds(); // Handle keyboard LEDs
                        keyboard_handle_timeout = millis(); // Reset timeout on successful handle
                    } else if (millis() - keyboard_handle_timeout > 100) { // Timeout after 100ms
                        break; // Break if keyboard_handle is stuck
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
 * @brief Runs the startup sequence of key presses with delays.
 * This function sends a predefined sequence of keys with specified delays.
 */
void runStartupSequence()
{
    for (int i = 0; i < numKeyActions; ++i)
    {
        const TimedKeyAction &action = keySequence[i];
        performActiveDelay(action.delaySeconds, action.delaySeconds > 1);
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.printf(F("LOG: runStartupSequence() - Sending key: %s\n"), action.keyName);
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
            delay(1);
        }
#ifdef DEBUG_LOG_DEVICE
        DEBUG_LOG_DEVICE.printf(F("LOG: runStartupSequence() - Key %s sent.\n"), action.keyName);
#endif
    }
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: runStartupSequence() - Key press sequence complete."));
#endif

}

/**
 * @brief Setup function to initialize the PS/2 keyboard emulation and serial communication.
 */ 
void setup()
{
    pinMode(BOOT_LED_PIN, OUTPUT);
    digitalWrite(BOOT_LED_PIN, LOW);
    Serial.begin(115200);
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Sketch starting. Serial initialized."));
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - Initializing PS/2 keyboard emulation..."));
#endif
    keyboard.keyboard_init(keyboardResetCallback);
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: setup() - PS/2 keyboard emulation initialized."));
#endif
    runStartupSequence(); // Run the startup sequence of key presses
}



/**
 * @brief Main loop function to handle keyboard events.
 */
void loop()
{
    if (keyboard.keyboard_handle(&kbd_leds))
    {
        handle_leds();
    }
    delay(1);
}




/**
 * @brief Callback function for keyboard reset.
 * This function is called when the PS/2 keyboard is reset.
 * It runs the startup sequence of key presses.
 */
void keyboardResetCallback()
{
#ifdef DEBUG_LOG_DEVICE
    DEBUG_LOG_DEVICE.println(F("LOG: keyboardResetCallback() - Running startup sequence due to keyboard reset."));
#endif
//    keyboard_initialized = true; // Reset keyboard state
    runStartupSequence();
}
