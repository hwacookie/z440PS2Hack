# Project Name: z440Hack
# User: [User Name]
# Date: 2025-06-07
# State: Open
# Planned Features:
*

# Changes:
*

# Challenges:
* Experienced an endless loop during PS/2 communication, particularly when sending keystrokes and during device reset (sending AA). This was confusing as receiving commands seemed to work, and recent code changes didn't appear to be the root cause.

# Open Issues:
*

# Decisions:
*

# Summary:
Discovered that the endless loop issue was caused by plugging the PS/2 device (emulating a keyboard) into the mouse PS/2 port instead of the keyboard port. Although the protocols are similar, the ports are not fully interchangeable, leading to unexpected behavior during sending.

**Important Note:** When connecting the device, double-check that the green mouse cable is plugged into the purple keyboard port.
