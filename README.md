# 2WSBR STM32 Firmware

## Introduction

This repository contains the firmware, MATLAB analysis scripts, and
session telemetry for a final-year BEng Mechatronic Engineering
individual project, University of Manchester, 2025-26. The project is a two-wheeled
self-balancing robot driven by stepper motors through TMC2209 drivers,
balanced by a cascade angle-rate controller running on an STM32G4 at
200 Hz. This README documents how to install, run, and reproduce the
results reported in the accompanying dissertation.

## Contextual overview

The hardware platform comprises:
- MCU: STM32G4 at 170 MHz (HSI -> PLL), HAL-based
- IMU: MPU6050 over I2C1, DLPF ~42 Hz
- Motors: 2x SM-42HB34F08AB NEMA-17 steppers, 1.8 deg/step
- Drivers: 2x TMC2209, 1/16 microstepping, VREF = 1.06 V
- Supply: 12 V battery, 12V->5V buck to STM32 VIN, board 3.3V regulator to logic rail
- Telemetry: USART2 (fast, to MATLAB) + USART1 (Bluetooth, commands)

The repository is laid out as follows:

    2WSBR_Code/   STM32CubeIDE project, main.c, HAL drivers
    matlab/       live dashboard, fitting scripts, analysis, session telemetry

Two firmware versions are preserved as annotated git tags:

- `v1.1-lqr-direct`: the deployed data-driven cascade. Cascade gains
  taken directly from the LQR synthesis output without empirical
  refinement: outer Kp = 7.023, inner Kp = 37.010. Responsible for the
  six-trial characterisation (33.8 s to 120.2 s, median 66.8 s,
  quiescent sigma 0.054 +/- 0.006 deg) reported as the dissertation
  headline.
- `v0.9-empirical`: the earlier empirically-tuned configuration with
  inner PID-F (Kp = 800, Ki = 10, Kd = 33, EMA derivative filter
  beta = 0.4) and outer command-effort regulation. Responsible for
  the 9.5 minute endurance run reported as the comparator.

Either firmware is reachable via `git checkout v0.9-empirical` or
`git checkout v1.1-lqr-direct`. The corresponding telemetry sessions
referenced in the dissertation are listed in `matlab/README.md`.

## Installation instructions

1. Open `2WSBR_Code/` in STM32CubeIDE (tested with v1.14).
2. Build -> Run on connected STM32G4 Nucleo / custom board.
3. Connect USART2 at 460800 8N1 for telemetry.
4. Optional: pair USART1 Bluetooth for runtime commands.

## How to run the software

Power on with the robot held near upright. Boot calibration averages
500 stationary IMU samples (~2.5 s) before control engages. Place on
a level surface and release gently. The safe-angle supervisor trips
motors off if |theta| exceeds 8 deg for longer than 1.5 s.

The data-driven cascade is more sensitive to startup pose and ambient
conditions than the empirical comparator. For best results allow
the board five minutes of warm-up after power-on before the first
balancing trial, and release the chassis from a careful manual hold
at upright rather than relying on a quick toss-and-release.

For MATLAB analysis, open `matlab/main_live_dashboard.m` to begin live
telemetry capture during a balancing run; saved sessions can be
re-analysed with the per-figure scripts named in `matlab/README.md`.

## Technical details

Locked control configuration (data-driven cascade, v1.1):

- Loop: 200 Hz (5 ms), polled on TIM6
- Complementary filter alpha: 0.97 (corner ~ 0.98 Hz)
- Outer angle loop: PI, Kp = 7.023, Ki = 2, soft saturator a = 5
- Inner rate loop: PI, Kp = 37.010, Ki = 15
- Output: clamped at 3800 pps, slew 30000 pps/s
- Safety: |theta| > 8 deg for 1500 ms -> motors disabled

The proportional gains (Kp_theta = 7.023, Kp_q = 37.010) are the
cascade decomposition of the reported positive LQR gain magnitudes
K_abs = [259.94, 37.01], deployed unchanged. They are not the result
of an empirical refinement step; the dissertation Appendix G.4
explains the signed MATLAB return and motor-sign convention, and
Appendix G.5 documents the decomposition arithmetic in full.

Headline result: six independent balancing trials at the locked
configuration produced durations of 33.8 to 120.2 s (median 66.8 s)
with quiescent pitch standard deviation 0.054 +/- 0.006 deg, net
wheel displacement below 2 mm over 30 s windows, and zero motor
saturation events across all six trials.

The MATLAB `matlab/` directory contains the scripts that produce
each figure and reproduce each numerical result reported in the
dissertation. See `matlab/README.md` for the script-to-figure map.

## Known issues and future improvements

Known issues:

- The deployed cascade is more sensitive to startup conditioning
  than the empirical comparator. The six-trial characterisation
  was conducted with deliberate attention to release pose and
  warm-up; an earlier evaluation set on a subsequent day with
  less careful attention produced three engagement failures out
  of five attempts.
- Long runs end in a slow sub-critical pitch growth of unconfirmed
  cause (thermal IMU drift, tether dressing, and battery droop are
  candidates). Duration to fall varies by a factor of three across
  the six characterisation trials at identical firmware gains.
- A separately-developed empirically-tuned controller balances for
  around 9.5 minutes and is retained as a comparator rather than the
  primary subject of analysis.

Future improvements identified in the dissertation include:

- A startup-conditioning routine (staged gain ramp or wait-for-steady
  pose check) to address the engagement sensitivity.
- Independent characterisation of the slow-drift candidates (IMU
  thermal drift, tether mechanical contribution, battery droop) to
  identify which is dominant.
- Replacement of the stepper drive chain with field-oriented control
  of brushless motors with wheel-side encoders, removing the
  pull-out, synchronism, and timer-floor constraints inherent to
  stepper actuation.

## Academic context

This repository is coursework submitted for assessment as a BEng
final-year individual project at the University of Manchester, May
2026. The code and methodology are original work for academic credit.
Licensing will be decided after grading.
