# Z440 PS/2 Hack Project

This project is an Arduino sketch designed for a 5V Arduino Pro Mini (ATmega168P, 16MHz). Its primary functions are:

1.  **PS/2 Keyboard Emulation**: At boot, the Arduino emulates a PS/2 keyboard to send a specific sequence of key presses. This can be used, for example, to automate BIOS interactions or other boot-time operations.
2.  **Sensor Data Collection**: The sketch collects data from various sensors:
    *   DHT22: Temperature and humidity.
    *   PIR: Motion detection.
    *   Thermistors (4x): Temperature readings from four different points.
3.  **JSON Datagram Transmission**: The collected sensor data is formatted into a JSON datagram and sent over the serial connection.
4.  **Logging**: Includes serial logging for debugging and monitoring, using the `F()` macro to save RAM.
5.  **Boot LED Indication**: The built-in LED blinks during the initial delay sequence.

The device is intended to be powered from the PS/2 port.

## Key Features:

*   **Automated Boot Sequence**: Sends predefined keystrokes via PS/2 emulation.
*   **Multi-Sensor Integration**: Reads data from DHT22, PIR, and multiple thermistors.
*   **Data Serialization**: Outputs sensor data in JSON format for easy parsing by other systems.
*   **RAM Optimization**: Uses `F()` macro for string literals to conserve RAM on the ATmega168P.
*   **Active Delay with Feedback**: Provides visual feedback (LED blinking) during delays and maintains PS/2 keyboard responsiveness.
