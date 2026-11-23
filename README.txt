BuckyBoost MPC Arduino Due bench demo

Open BuckyBoost_MPC_Demo.ino in Arduino IDE with pindefs.h in the same folder.
Arduino Due (Programming Port), Serial Monitor 115200 baud.

PWM outputs: H1 = D39 / PWMH2, H2 = D38 / PWML2.
PWM frequency ~21.99 kHz, deadtime ~167 ns.
PWM_CMR_CPOL was added to invert output polarity relative to the previous sketch.
VERIFY actual high-side and low-side active levels on oscilloscope before connecting a power stage.

SIMULATE_PLANT=1: feedback from averaged discrete-time buck plant, no physical ADC required.
Reference profile: initial 0.30 A for 6s, then 3s ramps and 3s holds:
0.30 -> 0.75 -> 0.50 -> 0.85 -> 0.40 -> 0.75 A; repeats.
The reference profile runs only in RUN (TOG1 HIGH); toggling OFF resets it.
The model responds in milliseconds; slow reference ramps make duty changes visible over seconds.

The MPC uses Phi[5][3], K_du[5] from the previously generated 5-step closed-form linear MPC, not an online cost-search solver.
Real current ADC conversion (when SIMULATE_PLANT=0): (Vadc-1.65 V)*2 A/V.

This code has not been compiled or tested on physical hardware.
