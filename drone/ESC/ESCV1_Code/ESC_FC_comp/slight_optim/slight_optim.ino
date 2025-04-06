#define PWM_INPUT_PIN 2
#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 50
#define PWM_START_DUTY 150

byte bldc_step = 0;
byte motor_speed;
unsigned int i;
bool reverse = false;

void setup() {
  DDRD |= 0x38; // D3, D4, D5 outputs
  PORTD = 0x00;
  DDRB |= 0x0E; // D9, D10, D11 outputs
  PORTB = 0x00;

  // Timer1: 16 kHz PWM
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, non-inverting
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaler
  ICR1 = 1000; // 16 MHz / 1000 = 16 kHz

  // Timer2: 16 kHz PWM
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM
  TCCR2B = (1 << CS20); // No prescaler

  ACSR = 0x10; // Disable comparator initially
  pinMode(PWM_INPUT_PIN, INPUT);
  beeper();
}

ISR (ANALOG_COMP_vect) { //When an analog comparator event occurs, this routine is executed.
  for(i = 0; i < 10; i++) { 
    if (bldc_step & 1) { 
      if (!(ACSR & 0x20)) 
        i -= 1; 
    } else { 
      if (ACSR & 0x20) 
        i -= 1; 
    } 
  } 
  move_bldc(); 
  bldc_step++; 
  bldc_step %= 6; 
}

void move_bldc() { 
  if (reverse) { // If reverse is true, it executes the reverse steps.
    switch(bldc_step) { 
      case 0: 
        CH_BL(); 
        BEMF_A_FALLING(); 
        break; 
      case 1: 
        CH_AL(); 
        BEMF_B_RISING(); 
        break; 
      case 2: 
        BH_AL(); 
        BEMF_C_FALLING(); 
        break; 
      case 3: 
        BH_CL(); 
        BEMF_A_RISING(); 
        break; 
      case 4: 
        AH_CL(); 
        BEMF_B_FALLING(); 
        break; 
      case 5: 
        AH_BL(); 
        BEMF_C_RISING(); 
        break; 
    } 
  } else { // Forward direction
    switch(bldc_step) { 
      case 0: 
        AH_BL(); 
        BEMF_C_RISING(); 
        break; 
      case 1: 
        AH_CL(); 
        BEMF_B_FALLING(); 
        break; 
      case 2: 
        BH_CL(); 
        BEMF_A_RISING(); 
        break; 
      case 3: 
        BH_AL(); 
        BEMF_C_FALLING(); 
        break; 
      case 4: 
        CH_AL(); 
        BEMF_B_RISING(); 
        break; 
      case 5: 
        CH_BL(); 
        BEMF_A_FALLING(); 
        break; 
    } 
  } 
}

void loop() {
  SET_PWM_DUTY(PWM_START_DUTY);
  i = 5000;
  while(i > 100) {
    SET_PWM_DUTY(PWM_START_DUTY);
    delayMicroseconds(i);
    move_bldc();
    bldc_step++;
    bldc_step %= 6;
    i -= 20;
  }

  motor_speed = PWM_START_DUTY;
  ACSR |= 0x08; // Enable comparator interrupt

  while(1) {
    int PWM_value = pulseIn(PWM_INPUT_PIN, HIGH); // Read µs
    if (PWM_value >= 1000 && PWM_value <= 2000) {
      motor_speed = map(PWM_value, 1000, 2000, PWM_MIN_DUTY, PWM_MAX_DUTY);
    } else {
      motor_speed = 0;
    }
    SET_PWM_DUTY(motor_speed);
    delay(10); // Reduce delay for faster response
  }
}

void BEMF_A_FALLING() { 
  ADCSRB = (0 << ACME); 
  ACSR &= ~0x01; // Configures the ACSR to enable interrupts on a falling edge of the comparator output.
}

void BEMF_A_RISING() { 
  ADCSRB = (0 << ACME); // Select AIN1 as comparator negative input 
  ACSR |= 0x03; // Configures the ACSR to enable interrupts on a rising edge of the comparator output.
}

void BEMF_B_FALLING() { 
  ADCSRA = (0 << ADEN); // Disable the ADC module 
  ADCSRB = (1 << ACME); 
  ADMUX = 2; // Select analog channel 2 as comparator negative input 
  ACSR &= ~0x01; 
}

void BEMF_B_RISING() { 
  ADCSRA = (0 << ADEN); // Disable the ADC module 
  ADCSRB = (1 << ACME); 
  ADMUX = 2; // Select analog channel 2 as comparator negative input 
  ACSR |= 0x03; 
}

void BEMF_C_FALLING() { 
  ADCSRA = (0 << ADEN); // Disable the ADC module 
  ADCSRB = (1 << ACME); 
  ADMUX = 3; // Select analog channel 3 as comparator negative input 
  ACSR &= ~0x01; 
}

void BEMF_C_RISING() { 
  ADCSRA = (0 << ADEN); // Disable the ADC module 
  ADCSRB = (1 << ACME); 
  ADMUX = 3; // Select analog channel 3 as comparator negative input 
  ACSR |= 0x03;
}

void AH_BL() {
  PORTB  =  0x04;
  PORTD &= ~0x18;
  PORTD |=  0x20;
  TCCR1A =  0;            
  TCCR2A =  0x81;
}

void AH_CL() {
  PORTB  =  0x02;
  PORTD &= ~0x18;
  PORTD |=  0x20;
  TCCR1A =  0;            
  TCCR2A =  0x81;
}

void BH_CL() {
  PORTB  =  0x02;
  PORTD &= ~0x28;
  PORTD |=  0x10;
  TCCR2A =  0;            
  TCCR1A =  0x21;
}

void BH_AL() {
  PORTB  =  0x08;
  PORTD &= ~0x28;
  PORTD |=  0x10;
  TCCR2A =  0;            
  TCCR1A =  0x21;
}

void CH_AL() {
  PORTB  =  0x08;
  PORTD &= ~0x30;
  PORTD |=  0x08;
  TCCR2A =  0;            
  TCCR1A =  0x81;
}

void CH_BL() {
  PORTB  =  0x04;
  PORTD &= ~0x30;
  PORTD |=  0x08;
  TCCR2A =  0;            
  TCCR1A =  0x81;
}

void SET_PWM_DUTY(byte duty) {
  if (duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  OCR1A = duty;
  OCR1B = duty;
  OCR2A = duty;
}

void beeper() {
  const int beep_duration = 100;
  const int pause_duration = 200;
  const int num_beeps = 2;

  for (int j = 0; j < num_beeps; j++){
    SET_PWM_DUTY(PWM_MAX_DUTY);
    delay(beep_duration);
    SET_PWM_DUTY(0);
  }
}
