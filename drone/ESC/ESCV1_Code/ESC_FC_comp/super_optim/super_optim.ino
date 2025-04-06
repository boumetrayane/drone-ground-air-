#define PWM_INPUT_PIN 2
#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 50
#define PWM_START_DUTY 75

byte bldc_step = 0;
byte motor_speed;
unsigned int i;

void setup() {
  DDRD |= 0x38; // D3, D4, D5 outputs
  PORTD = 0x00;
  DDRB |= 0x0E; // D9, D10, D11 outputs
  PORTB = 0x00;

  // Timer1: 16 kHz PWM
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = 1000; // 16 kHz

  // Timer2: 16 kHz PWM
  TCCR2A = (1 << WGM20) | (1 << COM2A1);
  TCCR2B = (1 << CS20);
  OCR2A = 500; // 16 kHz

  ACSR = 0x10; // Disable comparator
  pinMode(PWM_INPUT_PIN, INPUT);
  beeper();
}

ISR(ANALOG_COMP_vect) {
  for(i = 0; i < 10; i++) {
    if (bldc_step & 1) {
      if (!(ACSR & 0x20)) i -= 1;
    } else {
      if (ACSR & 0x20) i -= 1;
    }
  }
  move_bldc();
  bldc_step++;
  bldc_step %= 6;
}

void move_bldc() {
  switch(bldc_step) {
    case 0: AH_BL(); BEMF_C_RISING(); break;
    case 1: AH_CL(); BEMF_B_FALLING(); break;
    case 2: BH_CL(); BEMF_A_RISING(); break;
    case 3: BH_AL(); BEMF_C_FALLING(); break;
    case 4: CH_AL(); BEMF_B_RISING(); break;
    case 5: CH_BL(); BEMF_A_FALLING(); break;
  }
}

void loop() {
  byte start_duty = PWM_START_DUTY;
  i = 5000;
  cli(); // Disable interrupts
  while(i > 100) {
    SET_PWM_DUTY(start_duty);
    delayMicroseconds(i);
    move_bldc();
    bldc_step++;
    bldc_step %= 6;
    i -= 20;
    start_duty += 1;
    if (start_duty > 150) start_duty = 150;
  }
  sei(); // Enable interrupts

  motor_speed = PWM_START_DUTY;
  ACSR |= 0x08; // Enable comparator interrupt

  while(1) {
    int PWM_value = pulseIn(PWM_INPUT_PIN, HIGH, 3000);
    if (PWM_value >= 1000 && PWM_value <= 2000) {
      motor_speed = map(PWM_value, 1000, 2000, PWM_MIN_DUTY, PWM_MAX_DUTY);
    }
    SET_PWM_DUTY(motor_speed);
    delay(2);
  }
}

void BEMF_A_FALLING() { ADCSRB = 0; ACSR &= ~0x01; }
void BEMF_A_RISING() { ADCSRB = 0; ACSR |= 0x03; }
void BEMF_B_FALLING() { ADCSRA = 0; ADCSRB = (1 << ACME); ADMUX = 2; ACSR &= ~0x01; }
void BEMF_B_RISING() { ADCSRA = 0; ADCSRB = (1 << ACME); ADMUX = 2; ACSR |= 0x03; }
void BEMF_C_FALLING() { ADCSRA = 0; ADCSRB = (1 << ACME); ADMUX = 3; ACSR &= ~0x01; }
void BEMF_C_RISING() { ADCSRA = 0; ADCSRB = (1 << ACME); ADMUX = 3; ACSR |= 0x03; }

void AH_BL() { PORTB = 0x04; PORTD &= ~0x18; PORTD |= 0x20; OCR1A = 0; OCR1B = motor_speed; OCR2A = 0; }
void AH_CL() { PORTB = 0x02; PORTD &= ~0x18; PORTD |= 0x20; OCR1A = motor_speed; OCR1B = 0; OCR2A = 0; }
void BH_CL() { PORTB = 0x02; PORTD &= ~0x28; PORTD |= 0x10; OCR1A = motor_speed; OCR1B = 0; OCR2A = 0; }
void BH_AL() { PORTB = 0x08; PORTD &= ~0x28; PORTD |= 0x10; OCR1A = 0; OCR1B = 0; OCR2A = motor_speed; }
void CH_AL() { PORTB = 0x08; PORTD &= ~0x30; PORTD |= 0x08; OCR1A = 0; OCR1B = 0; OCR2A = motor_speed; }
void CH_BL() { PORTB = 0x04; PORTD &= ~0x30; PORTD |= 0x08; OCR1A = 0; OCR1B = motor_speed; OCR2A = 0; }

void SET_PWM_DUTY(byte duty) {
  if (duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  OCR1A = duty;
  OCR1B = duty;
  OCR2A = duty;
}

void beeper() {
  const int beep_duration = 100;
  const int num_beeps = 2;
  for (int j = 0; j < num_beeps; j++) {
    SET_PWM_DUTY(PWM_MAX_DUTY);
    delay(beep_duration);
    SET_PWM_DUTY(0);
    delay(beep_duration);
  }
}