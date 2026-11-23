/* BuckyBoost: Arduino Due + Octopus Mini, bench-only software plant test.
 * Based on teammate's ECE512_Buck_MPC.ino and Final_Project.m.
 * H1 = PWMH2 Arduino D39 (PC7); H2 = PWML2 Arduino D38 (PC6).
 * CPOL inverted based on measured H1 ~17%, H2 ~82% at 82% command.
 * Scope-verify H1 high-time now follows commanded duty before attaching drivers.
 * Do not connect the power stage during this simulated-feedback test.
 * Requires the supplied pindefs.h in this sketch folder.
 */
#include "pindefs.h"

#define SIMULATE_PLANT 1
#define PWM_PERIOD_CPRD 1910u
#define DEAD_TIME_TICKS 14u
#define IDLE 0
#define RUN 1

// Real sensor conversion, used only if SIMULATE_PLANT is set to 0.
// 2 A/V means 0.5 V/A. Assumed 1.65 V at zero current.
static const float ADC_VREF = 3.3f;
static const float ADC_MAX_COUNT = 4095.0f;
static const float CURRENT_A_PER_V = 2.0f;
static const float CURRENT_ZERO_V = 1.65f;
static const float VO_DIVIDER_RATIO = 3.5f;
#define ADC_CH_IP 7u
#define ADC_CH_VO 6u

static const float VIN = 10.0f;
static const float L_H = 0.002f;
static const float C_F = 220.0e-6f;
static const float R_OHM = 11.0f;
// Every six seconds select a new target, with a 3-second linear ramp.
// Slow ramp deliberately makes duty changes visible on an ordinary scope.
static const float REF_LEVELS_A[] = {0.30f, 0.75f, 0.50f, 0.85f, 0.40f, 0.75f};
static const uint32_t REF_SEGMENT_TICKS = 132000UL; // ~6.003 s at 21.99 kHz
static const uint32_t REF_RAMP_TICKS = 66000UL;    // ~3.001 s
static const uint32_t REF_COUNT = sizeof(REF_LEVELS_A) / sizeof(REF_LEVELS_A[0]);
static const float TS = (2.0f * PWM_PERIOD_CPRD) / 84000000.0f;

// Exact ZOH discrete-time model at TS=45.47619 us, calculated from
// A=[0,-1/L;1/C,-1/(R*C)], B=[VIN/L;0].
// x[k+1] = Ad*x[k] + Bd*d[k], x=[inductor current; output voltage].
static const float Ad[2][2] = {
  {0.99766547f, -0.02250814f},
  {0.20461946f,  0.97906370f}
};
static const float Bd[2] = {0.22720371f, 0.02334530f};

// MPC: Np=5, Nu=2, qi=1, rd=0.05; recomputed for actual 1910-count PWM period.
// Augmented state xi=[i_L, v_o, previous duty].
static const float K_du[5] = {
  1.02557424f, 0.72906297f, 0.42924491f, 0.12757712f, -0.17450187f
};
static const float Phi[5][3] = {
  {0.99766547f, -0.02250814f, 0.22720371f},
  {0.99073079f, -0.04449250f, 0.45335154f},
  {0.97931387f, -0.06586050f, 0.67741055f},
  {0.96355129f, -0.08652416f, 0.89837676f},
  {0.94359732f, -0.10640041f, 1.11527925f}
};

volatile int State = IDLE;
volatile uint16_t ip_adc_raw = 2048;
volatile uint16_t vo_adc_raw = 0;
volatile float ip_meas = 0.0f;
volatile float vo_meas = 0.0f;
volatile float duty_mpc = 0.0f;
volatile float ip_ref = REF_LEVELS_A[0];
volatile float d_prev = 0.0f;
volatile float plant_i = 0.0f;
volatile float plant_v = 0.0f;
volatile uint32_t control_ticks = 0;
volatile uint32_t run_ticks = 0;

static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static void resetControlAndPlant() {
  plant_i = 0.0f;
  plant_v = 0.0f;
  ip_meas = 0.0f;
  vo_meas = 0.0f;
  d_prev = 0.0f;
  duty_mpc = 0.0f;
  ip_ref = REF_LEVELS_A[0];
  run_ticks = 0;
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 0;
}

void pwmc_setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID36;
  PWM->PWM_DIS = PWM_DIS_CHID0 | PWM_DIS_CHID2;

  // Pins 38/39 -> peripheral B: PWML2/PWMH2.
  PIOC->PIO_ABSR |= PIO_ABSR_P6 | PIO_ABSR_P7;
  PIOC->PIO_PDR |= PIO_PDR_P6 | PIO_PDR_P7;

  PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);
  PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_CLKA | PWM_CMR_CALG;
  PWM->PWM_CH_NUM[0].PWM_CPRD = PWM_PERIOD_CPRD;
  PWM->PWM_CH_NUM[0].PWM_CDTY = PWM_PERIOD_CPRD / 2;

  PWM->PWM_CH_NUM[2].PWM_CMR = PWM_CMR_DTE | PWM_CMR_CPOL | PWM_CMR_CPRE_CLKA | PWM_CMR_CALG;
  PWM->PWM_CH_NUM[2].PWM_CPRD = PWM_PERIOD_CPRD;
  PWM->PWM_CH_NUM[2].PWM_CDTY = 0;
  PWM->PWM_CH_NUM[2].PWM_DT = PWM_DT_DTH(DEAD_TIME_TICKS) |
                              PWM_DT_DTL(DEAD_TIME_TICKS);

  NVIC_SetPriority(PWM_IRQn, 0);
  NVIC_EnableIRQ(PWM_IRQn);
  PWM->PWM_IER1 = PWM_IER1_CHID0;
  PWM->PWM_ENA = PWM_ENA_CHID0;
}

#if !SIMULATE_PLANT
void adc_setup() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_CR = ADC_CR_SWRST;
  ADC->ADC_MR = ADC_MR_TRGEN_EN | ADC_MR_TRGSEL_ADC_TRIG4 | ADC_MR_PRESCAL(4);
  ADC->ADC_CHER = (1u << ADC_CH_IP) | (1u << ADC_CH_VO);
  const uint32_t last_ch = (ADC_CH_IP > ADC_CH_VO) ? ADC_CH_IP : ADC_CH_VO;
  ADC->ADC_IER = (1u << last_ch);
  NVIC_SetPriority(ADC_IRQn, 1);
  NVIC_EnableIRQ(ADC_IRQn);
  PWM->PWM_CMP[0].PWM_CMPV = PWM_CMPV_CV(10);
  PWM->PWM_CMP[0].PWM_CMPM = PWM_CMPM_CEN;
  PWM->PWM_ELMR[0] = PWM_ELMR_CSEL0;
}
void ADC_Handler() {
  ip_adc_raw = ADC->ADC_CDR[ADC_CH_IP];
  vo_adc_raw = ADC->ADC_CDR[ADC_CH_VO];
}
#endif

void setup() {
  pinMode(R1, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(BM, OUTPUT);
  pinMode(TOG1, INPUT);
  digitalWrite(R1, HIGH);
  digitalWrite(G1, LOW);
  digitalWrite(B1, LOW);
  Serial.begin(115200);
  pwmc_setup();
#if !SIMULATE_PLANT
  adc_setup();
#endif
  Serial.println("BuckyBoost MPC: virtual buck, ramped reference, inverted PWM polarity");
  Serial.println("state,t_s,i_A,v_V,duty,reference_A,h1_high_pct,h2_high_pct");
}

void PWM_Handler() {
  const uint32_t status = PWM->PWM_ISR1; // acknowledge period interrupt
  (void)status;
  PIOC->PIO_SODR = (1u << 24);  // BM high: ISR execution time
  ++control_ticks;

  if (State != RUN) {
    PWM->PWM_DIS = PWM_DIS_CHID2;
    PIOC->PIO_SODR = (1u << 14);
    PIOC->PIO_CODR = (1u << 12);
    PIOB->PIO_CODR = (1u << 14);
    PIOC->PIO_CODR = (1u << 24);
    return;
  }

  PIOC->PIO_CODR = (1u << 14);
  PIOC->PIO_SODR = (1u << 12);
  PIOB->PIO_CODR = (1u << 14);
  // Update slow reference profile at PWM rate (no millis() inside ISR).
  const uint32_t segment = (run_ticks / REF_SEGMENT_TICKS) % REF_COUNT;
  const uint32_t phase = run_ticks % REF_SEGMENT_TICKS;
  const uint32_t previous = (segment + REF_COUNT - 1u) % REF_COUNT;
  const float start_ref = (segment == 0u && run_ticks < REF_SEGMENT_TICKS)
                              ? REF_LEVELS_A[0] : REF_LEVELS_A[previous];
  const float target_ref = REF_LEVELS_A[segment];
  if (phase < REF_RAMP_TICKS) {
    const float alpha = (float)phase / (float)REF_RAMP_TICKS;
    ip_ref = start_ref + alpha * (target_ref - start_ref);
  } else {
    ip_ref = target_ref;
  }
  ++run_ticks;

#if SIMULATE_PLANT
  // At the beginning of cycle k, these are the simulated x[k] states.
  const float i = plant_i;
  const float v = plant_v;
  ip_meas = i;
  vo_meas = v;
#else
  ip_meas = ((float)ip_adc_raw * ADC_VREF / ADC_MAX_COUNT - CURRENT_ZERO_V)
            * CURRENT_A_PER_V;
  vo_meas = ((float)vo_adc_raw * ADC_VREF / ADC_MAX_COUNT)
            * VO_DIVIDER_RATIO;
#endif

  // Same unconstrained, incremental 5-step MPC structure as teammate's code.
  float du = 0.0f;
  for (int j = 0; j < 5; ++j) {
    const float y_hat = Phi[j][0] * ip_meas +
                        Phi[j][1] * vo_meas +
                        Phi[j][2] * d_prev;
    du += K_du[j] * (ip_ref - y_hat);
  }
  const float requested_duty = clamp01(d_prev + du);
  const uint32_t duty_counts = (uint32_t)(requested_duty * PWM_PERIOD_CPRD + 0.5f);
  // Track duty actually representable by the PWM timer.
  const float applied_duty = (float)duty_counts / (float)PWM_PERIOD_CPRD;
  duty_mpc = applied_duty;
  d_prev = applied_duty;
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = duty_counts;
  PWM->PWM_ENA = PWM_ENA_CHID2;

#if SIMULATE_PLANT
  // Advance the virtual buck plant by one sample using the commanded duty.
  // This is an averaged, ideal CCM model (not a switching-level simulation).
  plant_i = Ad[0][0] * i + Ad[0][1] * v + Bd[0] * applied_duty;
  plant_v = Ad[1][0] * i + Ad[1][1] * v + Bd[1] * applied_duty;
#endif
  PIOC->PIO_CODR = (1u << 24);
}

void loop() {
  // TOG1 is the RUN enable. On every transition, reset virtual plant/control.
  const int requested_state = digitalRead(TOG1) ? RUN : IDLE;
  if (requested_state != State) {
    noInterrupts();
    State = IDLE;
    PWM->PWM_DIS = PWM_DIS_CHID2;
    resetControlAndPlant();
    State = requested_state;
    interrupts();
  }

  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    int s;
    float i, v, d, ref;
    uint32_t ticks;
    noInterrupts();
    s = State;
    i = ip_meas;
    v = vo_meas;
    d = duty_mpc;
    ref = ip_ref;
    ticks = run_ticks;
    interrupts();
    Serial.print(s);
    Serial.print(','); Serial.print(ticks * TS, 4);
    Serial.print(','); Serial.print(i, 4);
    Serial.print(','); Serial.print(v, 4);
    Serial.print(','); Serial.print(d, 4);
    Serial.print(','); Serial.print(ref, 3);
    Serial.print(','); Serial.print(100.0f * d, 1);
    Serial.print(','); Serial.println(100.0f * (1.0f - d), 1);
  }
}
