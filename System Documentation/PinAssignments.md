# Hardware Pin Assignments and Default States

This document summarizes the wiring and default logic levels used across the project sketches.

## Pin map (Arduino)
- Encoder: `2` (A), `3` (B)
- Motor driver: `11` (ENA/PWM), `12` (IN1), `13` (IN2)
- Flip switch: `5` (enable/disable; wired with `INPUT_PULLUP`)
- Limit switches: `8` (left/zero), `9` (right/front); wired with `INPUT_PULLUP`
- Proximity sensors (where used): `A0`–`A3`

## Logic conventions
- Flip switch (pin 5):
  - HIGH = ON/enabled
  - LOW = OFF/override (motor drive inhibited)
- Limit switches (pins 8, 9):
  - HIGH = PRESSED/triggered
  - LOW = open/not pressed
  - Pins are configured as `INPUT_PULLUP`, so a pressed switch should drive the pin HIGH.
- Encoder channels use pull-ups enabled in code when required.
- Motor voltage commands are constrained to ±10V equivalents (PWM scaled to 0–255 on pin 11).

## Notes
- All active sketches now guard motor output with the flip switch (pin 5).
- Limit-switch checks have been updated to treat HIGH as pressed per the observed hardware wiring.
- If diagnostics (see `In Progress/SwitchStatusTest/SwitchStatusTest.ino`) show a default PRESSED state, verify wiring so the default is LOW (open) with the internal pull-up active.
