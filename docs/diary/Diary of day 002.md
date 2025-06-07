# Diary of day 002

__Project Name:__ z440PS2Hack\
__User:__ Hauke\
__Date:__ 2025-06-06\
__State:__ Closed

## Planned Features

- Add a reset hook to detect soft resets of the attached PC.

## Changes

- Using the reset callback mechanism in PS2dev to initialize and start the keyboard sequence only after receiving a reset signal.
- Added defines to enable or disable logging.
- Added proper handling for the keyboard LEDs.

## Challenges

- Debugging PS2 communication protocol issues.
- Ensuring compatibility with hardware.

## Open Issues

- Further testing required for controller input reliability.
- Optimize code for performance and memory usage.

## Decisions


## Summary

Today, we implemented the reset callback mechanism in PS2dev to ensure the keyboard initialization sequence starts only after a reset signal is received. We also introduced configuration defines to enable or disable logging for debugging purposes. Additionally, proper handling for the keyboard LEDs was added to improve user feedback and interaction.
