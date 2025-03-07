#define PWM_INPUT_PIN 2 
#define PWM_MAX_DUTY 255 
#define PWM_MIN_DUTY 50 
#define PWM_START_DUTY 100 
#define PWM_FORWARD_MIN 50 
#define PWM_FORWARD_MAX 145 
#define PWM_REVERSE_MIN 155 
#define PWM_REVERSE_MAX 255 
#define PWM_DEAD_ZONE_MIN 146 
#define PWM_DEAD_ZONE_MAX 154 

byte bldc_step = 0; 
byte motor_speed; 
unsigned int i; //// Direction flag
bool reverse = false; //// Declare motor_speed here 

void setup() { 
  DDRD |= 0x38; // Configure pins 3, 4, and 5 as outputs
  PORTD = 0x00; 
  DDRB |= 0x0E; // Configure pins 9, 10, and 11 as outputs
  PORTB = 0x31; 
  TCCR1A = 0; 
  TCCR1B = 0x01; 
  TCCR2A = 0; 
  TCCR2B = 0x01; 
  ACSR = 0x10; 
  pinMode(PWM_INPUT_PIN, INPUT); 
  // Beeper function to signal the circuit is working correctly
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
    delayMicroseconds(i); 
    move_bldc(); 
    bldc_step++;
    bldc_step %= 6;
    i -= 20; 
  } 
  
  motor_speed = PWM_START_DUTY; // Sets the initial motor speed to PWM_START_DUTY.
  ACSR |= 0x08; // Enables the analog comparator interrupt, for BEMF detection.
  
  while(1) { // This loop runs continuously, reading the PWM input and adjusting the motor's behavior accordingly.
    int PWM_value = analogRead(PWM_INPUT_PIN); 
    if (PWM_value >= PWM_FORWARD_MIN && PWM_value <= PWM_FORWARD_MAX) { 
      reverse = false; // Sets reverse to false for forward direction.
      motor_speed = map(PWM_value, PWM_FORWARD_MIN, PWM_FORWARD_MAX, PWM_MIN_DUTY, PWM_MAX_DUTY);
    } else if (PWM_value >= PWM_REVERSE_MIN && PWM_value <= PWM_REVERSE_MAX) { 
      reverse = true; // Sets reverse to true for reverse direction.
      motor_speed = map(PWM_value, PWM_REVERSE_MIN, PWM_REVERSE_MAX, PWM_MIN_DUTY, PWM_MAX_DUTY); 
    } else if (PWM_value >= PWM_DEAD_ZONE_MIN && PWM_value <= PWM_DEAD_ZONE_MAX) { 
      motor_speed = 0; 
    } 
    SET_PWM_DUTY(motor_speed); 
    delay(100); 
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
  if(duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
  if(duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  OCR1A = duty;                   // Set pin 9  PWM duty cycle
  OCR1B = duty;                   // Set pin 10 PWM duty cycle
  OCR2A = duty;                   // Set pin 11 PWM duty cycle
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