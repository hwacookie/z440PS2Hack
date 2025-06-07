# Project Name: z440Hack
# User: [User Name]
# Date: 2025-06-06, 2025-06-07
# State: Closed
# Planned Features:
*   **Automated Boot Sequence**: Sends predefined keystrokes via PS/2 emulation.
*   **Multi-Sensor Integration**: Reads data from DHT22, PIR, and multiple thermistors.
*   **Data Serialization**: Outputs sensor data in JSON format for easy parsing by other systems.
*   **RAM Optimization**: Uses `F()` macro for string literals to conserve RAM on the ATmega168P.
*   **Active Delay with Feedback**: Provides visual feedback (LED blinking) during delays and maintains PS/2 keyboard responsiveness.

# Changes:
*   Added a new function to calculate the average temperature from the thermistors.
*   Added a new light sensor.
*   Modified the JSON output to include the average temperature and light sensor data.

# Challenges:
No challenges were faced today.

# Open Issues:
None.

# Decisions:
No decisions were made today.

# Summary:
The project's current state was documented in the diary entry. Changes were made to the source code today, including adding a new function to calculate the average temperature from the thermistors, adding a new light sensor, and modifying the JSON output to include the average temperature and light sensor data.
