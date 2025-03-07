/*NOTES!:
 *|= (Bitwise OR Assignment):
   This operator performs a bitwise OR operation between the variable on the left and the value on the right,
   and then assigns the result back to the variable.
   Example: DDRD |= 0x38; sets specific bits in DDRD to 1 while leaving the other bits unchanged.

 *&= (Bitwise AND Assignment):
   This operator performs a bitwise AND operation between the variable on the left and the value on the right,
   and then assigns the result back to the variable.
   Example: PORTD &= ~0x18; clears specific bits in PORTD (sets them to 0) while leaving the other bits unchanged.

 *~ (Bitwise NOT):
   This operator inverts the bits of its operand.
   Example: ~0x18 inverts the bits of 0x18 (binary 00011000), resulting in 11100111.

 *= (Assignment):
   This operator assigns the value on the right to the variable on the left.
   Example: PORTD = 0x00; sets all bits of PORTD to 0.

 *; (Semicolon):
   This symbol is used to terminate statements in C/C++.
   Example: PORTB = 0x31; ends the statement setting PORTB to 0x31.

 *Analog Comparator Setup: The analog comparator is configured to compare the BEMF voltage of a non-energized phase with a reference voltage
   (typically ground or zero volts).

 *Zero-Crossing Detection: When the rotor of the BLDC motor moves, it induces a voltage in the non-energized phase. The point where this induced
   voltage crosses zero volts is called the zero-crossing.

 *Interrupt Generation: When the analog comparator detects the zero-crossing (the BEMF voltage crossing zero), it generates an interrupt.
 
 *ISR Execution: The interrupt triggers the ISR (ISR(ANALOG_COMP_vect) in your code). The ISR performs tasks like debouncing the BEMF signal
   and advancing the motor commutation step.

 *PWM Signal:
   A PWM signal is a square wave that switches between a high (ON) and low (OFF) state.
   The frequency of the PWM signal is the rate at which it completes one cycle of ON and OFF states.

 *Duty Cycle:
   The duty cycle is the ratio of the time the signal is high (ON time) to the total period of the signal (ON time + OFF time).
   It is usually expressed as a percentage: $$ \text{Duty Cycle} = \frac{\text{ON Time}}{\text{Total Period}} \times 100\% $$


 *Application in Motor Control:
   In the context of BLDC motor control, the duty cycle of the PWM signal determines the average voltage applied to the motor
   and thus controls the motor's speed. Here's how it works:
   Higher Duty Cycle: More average voltage is applied to the motor, resulting in higher speed.
   Lower Duty Cycle: Less average voltage is applied to the motor, resulting in lower speed.

 *Visualization:
   Here's a simple visual representation of different duty cycles in a PWM SIGNAL:
     25% Duty Cycle:
       1-   |----|-------|
       0.5- | ON |  OFF  |
       0-   |----|-------|
     50% Duty Cycle:
       1-   |--------|--------|
       0.5- |   ON   |   OFF  |
       0-   |--------|--------|
     75% Duty Cycle
       1-   |---------|---|
       0.5- |   ON    |OFF|
       0-   |---------|---|

*/

#define PWM_INPUT_PIN 2 //Define the pin PORTD 2 for receiving the PWM signal from the flight controler to change the motor speed.
#define PWM_MAX_DUTY 255 //define the maximum duty cycle limits for PWM control.the highest value for an 8-bit PWM signal,
#define PWM_MIN_DUTY 50 //define the minimum duty cycle limits for PWM control.ensuring that the motor does not receive too low
#define PWM_START_DUTY 100 //Defines the starting duty cycle value used when the motor is initially powered on.
#define PWM_FORWARD_MIN 50 //PWM signals with values from 50 and above (up to PWM_FORWARD_MAX) will instruct the motor to rotate forward
#define PWM_FORWARD_MAX 145 //PWM signals with values up to 145 (from PWM_FORWARD_MIN) will control the motor speed in the forward direction.
#define PWM_REVERSE_MIN 155 //PWM signals with values from 155 and above (up to PWM_REVERSE_MAX) will instruct the motor to rotate in reverse.
#define PWM_REVERSE_MAX 255 //This constant defines the maximum PWM value that represents the reverse direction. PWM
#define PWM_DEAD_ZONE_MIN 146 //The dead zone acts as a neutral area where the motor does not move to prevent jittery behavior will stop the motor or set it to a neutral state.
#define PWM_DEAD_ZONE_MAX 154 //This constant defines the maximum value of the dead zone

byte bldc_step = 0; //it can store an 8-bit unsigned integer (values from 0 to 255). It is initialized to 0. This variable likely keeps track of the current step in the BLDC motor commutation sequence.
byte motor_speed; //used to store the motor speed, but it is not initialized here (so its initial value is indeterminate until assigned).
unsigned int i; // can store an unsigned integer (values from 0 to 65535 on most Arduino boards). This variable is used as a counter or an index in loops throughout the code.
bool reverse = false; //Boolean flag to indicate motor direction bieng forward since reverse is false unless changed by the FC.

void setup() {
DDRD |= 0x38; //sets the data direction register for port D. The value 0x38 in binary is 00111000, so this configures pins 3, 4, and 5 of port D as outputs.
PORTD |= 0X00; // sets all pins of port D to LOW (0V).
DDRB |= 0X0E; //sets the data direction register for port B. The value 0x0E in binary is 00001110, so this configures pins 9, 10, and 11 of port B as outputs.
PORTB |= 0X31; //sets port B pins to the value 0x31. In binary, 0x31 is 00110001, so this sets pin 0 to HIGH, pin 4 to HIGH, and the rest to LOW.

//TCCR1A (Timer/Counter Control Register A/B...)
TCCR1A = 0; //TCCR1A IS SET TO 0: This means all the settings that control the mode and output of Timer1 are cleared.
TCCR1B = 0X01; //is set to 0x01 (binary 00000001), which: (Selects the clock source as clkI/O, which is the system clock. No prescaling is applied (CS10 bit is set, CS11 and CS12 bits are cleared)).
TCCR2A = 0; //TCCR2A IS SET TO 0: clearing all the settings that control the mode and output of Timer2.
TCCR2B = 0X01; //is set to 0x01 (binary 00000001), which: (Selects the clock source as clkI/O, similar to Timer1. No prescaling is applied (CS20 bit is set, CS21 and CS22 bits are cleared)).

//ACSR stands for Analog Comparator Control and Status Register.
ACSR = 0X10; //0x10 in binary is 00010000, which corresponds to the ACD bit being set. ACD (Analog Comparator Disable): When this bit is set, the Analog Comparator is disabled.
//Setting ACSR to 0x10 disables the analog comparator and clears the interrupt flag. This ensures that the analog comparator does not generate interrupts.

//pinMode(pin, mode) is a function that configures the specified pin to behave either as an input or an output.
 pinMode(PWM_INPUT_PIN, INPUT); 
// Beeper function to signal the circuit is working correctly
  beeper();
}

/*NOTES!:
  The Back Electromotive Force (BEMF) signal is used in sensorless BLDC motor control to
  determine the rotor position and to time the commutation of the motor phases. The expected
  state of the BEMF signal is typically a zero-crossing event, which occurs when the voltage
  induced in the floating (non-energized) phase crosses zero volts.
  In simpler terms, when the BEMF signal crosses zero, it indicates that the rotor has reached
  a specific position relative to the stator magnets, and it's time to switch the motor phases
  to keep the motor running smoothly.
*/

//ISR (Interrupt Service Routine) used to handle Back Electromotive Force (BEMF) detection in a Brushless DC (BLDC) motor
ISR (ANALOG_COM_VECT){ //When an analog comparator event occurs, this routine is executed.
//BEMF DEBOUNCE (this debounce code is to ensure that the BEMF signal is stable before taking further action. It iterates 10 times to check the state of the BEMF signal.)
 for(i = 0; i<10 ; i++); //A loop that runs 10 times to perform debouncing
 {
   if(bldc_step & 1) //Checks if the bldc_step variable is odd. The & 1 operation isolates the least significant bit.
   {
     if(!(ACSR & 0X20)) //If the BEMF signal is not in the expected state (a zero-crossing event), it decrements the loop counter i to recheck, effectively delaying until the signal stabilizes.
     i -= 1;
   }
   else //Similar to the previous step, but for the case when bldc_step is even. It rechecks if the BEMF signal is in the expected state (a zero-crossing event).
   {
     if(ACSR & 0X20)
     i -= 1;
   }
  }
move_bldc(); //Calls the move_bldc() function, which handles the commutation step for the BLDC motor.
bldc_step ++; //Increments the commutation step counter.
bldc_step %= 6; //Ensures that bldc_step wraps around to 0 after reaching 6, maintaining a valid range for the commutation steps (0 to 5).
}  

/*NOTES!:
 WHAT IS THE analog comparator event IN THIS CASE:
 the analog comparator event refers to the detection of (BEMF) signal's zero-crossing
 by the analog comparator. When the BEMF signal crosses zero volts, it triggers the analog comparator,
 which then generates an interrupt. This interrupt is what your code is responding to in the Interrupt
 Service Routine (ISR) you provided.
 In the code, the ISR handles this analog comparator event, ensuring that the BEMF signal is stable 
 before calling the bldc_move() function to advance the commutation step. This process helps maintain
 the correct timing for switching the motor phases, ensuring smooth operation of the BLDC motor.
*/

void move_bldc(){ //declares the bldc_move function
  if (reverse){ //checks the reverse flag to determine the direction of the motor.If reverse is true, it executes the reverse steps.
    switch(bldc_step) {
     case 0:
     CH_BL(); //This function sets the high side of phase C and the low side of phase B in the mosfets.
     BEMF_A_FALLING(); //Configures the analog comparator to detect a falling edge on BEMF signal for phase A.
     break;
     case 1: 
     CH_AL(); //Sets the high side of phase C and the low side of phase A
     BEMF_B_RISING();//Sets up the analog comparator to detect a rising edge on the BEMF signal for phase B
     break;
     case 2: 
     BH_AL(); //Sets the high side of phase B and the low side of phase A.
     BEMF_C_FALLING(); //Sets up the analog comparator to detect a rising edge on the BEMF signal for phase C.
     break;
     case 3: 
     BH_CL(); // Sets the high side of phase B and the low side of phase C.
     BEMF_A_RISING(); //Sets up the analog comparator to detect a rising edge on the BEMF signal for phase A.
     break;
     case 4: 
     AH_CL(); //Sets the high side of phase A and the low side of phase C.
     BEMF_B_FALLING(); //Configures the analog comparator to detect a falling edge on the BEMF signal for phase B.
     break;
     case 5: 
     AH_BL(); //Sets the high side of phase A and the low side of phase B.
     BEMF_C_RISING(); //Sets up the analog comparator to detect a rising edge on the BEMF signal for phase C.
     break;
  }
}
  else{ //If reverse is false, it executes the commutation steps for forward directiON.
    switch(bldc_step){ 
      case 0: 
      AH_BL(); //Sets the high side of phase A and the low side of phase B.
      BEMF_C_RISING(); //Sets up the analog comparator to detect a rising edge on the BEMF signal for phase C.
      break; 
      case 1: 
      AH_CL(); //Sets the high side of phase A and the low side of phase C.
      BEMF_B_FALLING(); //Configures the analog comparator to detect a falling edge on the BEMF signal for phase B.
      break; 
      case 2: 
      BH_CL(); //Sets the high side of phase B and the low side of phase C.
      BEMF_A_RISING(); //Configures the analog comparator to detect a falling edge on the BEMF signal for phase C.
      break; 
      case 3: 
      BH_AL(); //Sets the high side of phase B and the low side of phase A.
      BEMF_C_FALLING(); //Configures the analog comparator to detect a falling edge on the BEMF signal for phase C.
      break; 
      case 4: 
      CH_AL(); //Sets the high side of phase C and the low side of phase A.
      BEMF_B_RISING(); //Sets up the analog comparator to detect a rising edge on the BEMF signal for phase B.
      break; 
      case 5: 
      CH_BL(); //Sets the high side of phase C and the low side of phase B.
      BEMF_A_FALLING(); //Configures the analog comparator to detect a falling edge on the BEMF signal for phase A.
      break;
    }
  }
} 

/*MATLAB CODE FOR GRAPHICAL REPRESENTATION OFCOMMUTATION STEP CYCLES:
% PWM Duty Cycles for each phase in each commutation step (Forward Direction)
pwm_duty_cycle_forward = [
    1 0 -1; % Step 0
    1 -1 0; % Step 1
    0 -1 1; % Step 2
    -1 0 1; % Step 3
    -1 1 0; % Step 4
    0 1 -1; % Step 5
];

% PWM Duty Cycles for each phase in each commutation step (Reverse Direction)
pwm_duty_cycle_reverse = [
    -1 1 0; % Step 0
    -1 0 1; % Step 1
    0 -1 1; % Step 2
    1 -1 0; % Step 3
    1 0 -1; % Step 4
    0 1 -1; % Step 5
];

% Generate the 3-phase PWM signals for forward direction
pwm_signal_forward = zeros(length(t), 3); % Initialize the PWM signal matrix
for step = 1:size(pwm_duty_cycle_forward, 1)
    start_idx = round((step-1)*T/6*fs) + 1;
    end_idx = round(step*T/6*fs);
    for phase = 1:3
        pwm_signal_forward(start_idx:end_idx, phase) = pwm_duty_cycle_forward(step, phase);
    end
end

% Generate the 3-phase PWM signals for reverse direction
pwm_signal_reverse = zeros(length(t), 3); % Initialize the PWM signal matrix
for step = 1:size(pwm_duty_cycle_reverse, 1)
    start_idx = round((step-1)*T/6*fs) + 1;
    end_idx = round(step*T/6*fs);
    for phase = 1:3
        pwm_signal_reverse(start_idx:end_idx, phase) = pwm_duty_cycle_reverse(step, phase);
    end
end

% Plot the 3-phase PWM signals for forward direction
figure;
subplot(2,1,1);
plot(t, pwm_signal_forward(:,1), 'r', 'DisplayName', 'Phase A');
hold on;
plot(t, pwm_signal_forward(:,2), 'g', 'DisplayName', 'Phase B');
plot(t, pwm_signal_forward(:,3), 'b', 'DisplayName', 'Phase C');
title('3-Phase PWM Signals for BLDC Motor steps (Forward)');
xlabel('Time (s)');
ylabel('Amplitude');
legend('show');
grid on;

% Customize the axes for forward direction plot
xticks(0:0.01:T); % Set x-axis ticks every 0.01 seconds
yticks(-1:0.5:1); % Set y-axis ticks every 0.5 units
xlim([0 T]); % Set x-axis limit
ylim([-1.2 1.2]); % Set y-axis limit

% Plot the 3-phase PWM signals for reverse direction
subplot(2,1,2);
plot(t, pwm_signal_reverse(:,1), 'r', 'DisplayName', 'Phase A');
hold on;
plot(t, pwm_signal_reverse(:,2), 'g', 'DisplayName', 'Phase B');
plot(t, pwm_signal_reverse(:,3), 'b', 'DisplayName', 'Phase C');
title('3-Phase PWM Signals for BLDC Motor steps (Reverse)');
xlabel('Time (s)');
ylabel('Amplitude');
legend('show');
grid on;

% Customize the axes for reverse direction plot
xticks(0:0.01:T); % Set x-axis ticks every 0.01 seconds
yticks(-1:0.5:1); % Set y-axis ticks every 0.5 units
xlim([0 T]); % Set x-axis limit
ylim([-1.2 1.2]); % Set y-axis limit

*/

void loop() { 
  SET_PWM_DUTY(PWM_START_DUTY); //This function call sets the initial PWM duty cycle
  i = 5000; //ariable is used to control the delay and gradual decrease in the delay between commutation steps
  while(i > 100) { //to gradually decrease the delay between commutation steps, allowing the motor to start smoothly.
    delayMicroseconds(i); 
    move_bldc(); 
    bldc_step++; // This line increments the bldc_step variable to move through the commutation cycles in (bldc_move).
    bldc_step %= 6; // Ensures that bldc_step cycles back to 0 after reaching 5
    i -= 20; // Decreases the value of i by 20 microseconds with each iteration of the loop, allowing the motor to accelerate smoothly during start-up.
  } 
  
  motor_speed = PWM_START_DUTY; // Sets the initial motor speed to PWM_START_DUTY.
  ACSR |= 0x08; // Enables the analog comparator interrupt, for BEMF detection.
  
  while(1) { // This loop runs continuously, reading the PWM input and adjusting the motor's behavior accordingly.
    int PWM_value = analogRead(PWM_INPUT_PIN); // Reads the PWM value from the PWM_INPUT_PIN and stores it in pwm_value.
    if (PWM_value >= PWM_FORWARD_MIN && PWM_value <= PWM_FORWARD_MAX) { // Checks if pwm_value is within the forward direction range (PWM_FORWARD_MIN to PWM_FORWARD_MAX).
      reverse = false; // Sets reverse to false for forward direction.
      motor_speed = map(PWM_value, PWM_FORWARD_MIN, PWM_FORWARD_MAX, PWM_MIN_DUTY, PWM_MAX_DUTY); // Maps the PWM value to the corresponding motor speed (duty cycle) using the map function.
    } else if (PWM_value >= PWM_REVERSE_MIN && PWM_value <= PWM_REVERSE_MAX) { // Checks if pwm_value is within the reverse direction range (PWM_REVERSE_MIN to PWM_REVERSE_MAX).
      reverse = true; // Sets reverse to true for reverse direction.
      motor_speed = map(PWM_value, PWM_REVERSE_MIN, PWM_REVERSE_MAX, PWM_MIN_DUTY, PWM_MAX_DUTY); // Maps the PWM value to the corresponding motor speed (duty cycle) using the map function.
    } else if (PWM_value >= PWM_DEAD_ZONE_MIN && PWM_value <= PWM_DEAD_ZONE_MAX) { // Checks if pwm_value is within the dead zone range (PWM_DEAD_ZONE_MIN to PWM_DEAD_ZONE_MAX).
      motor_speed = 0; // Sets motor_speed to 0, stopping the motor or putting it into a neutral state.
    } 
    SET_PWM_DUTY(motor_speed); // Calls the SET_PWM_DUTY function to set the PWM duty cycle to the motor_speed value calculated based on the PWM input.
    delay(100); // Introduces a delay, this helps stabilize the motor control by not updating too rapidly.
  } 
}

void BEMF_A_FALLING(){
  //ADCSRB: (Analog-to-Digital Converter Control and Status Register B) it's register is part of the Analog-to-Digital Converter (ADC) control registers
  ADCSRB = (0 << ACME); //Disables the ADC multiplexer and selects AIN1 as the negative input to the comparator.by setting it to 0.
  ACSR = ~0x01; //Configures the ACSR to enable interrupts on a rising edge of the comparator output.
}

void BEMF_A_RISING(){
  ADCSRB = (0 << ACME); // Select AIN1 as comparator negative input 
  ACSR |= 0x03; //Configures the ACSR to enable interrupts on a rising edge of the comparator output.
}

void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN); // Disable the ADC module
  ADCSRB = (1 << ACME); ////Enables the ADC multiplexer and selects the analog channel specified by ADMUX.
  ADMUX = 2; // Select analog channel 2 as comparator negative input
  ACSR |= 0x03; ////Configures the Analog Comparator Control and Status Register (ACSR) to enable interrupts on a falling edge of the comparator output.
}

void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);  // Disable the ADC module
  ADCSRB = (1 << ACME); //Enables the ADC multiplexer and selects the analog channel specified by ADMUX.
  ADMUX = 2; // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01; //Configures the Analog Comparator Control and Status Register (ACSR) to enable interrupts on a rising edge of the comparator output.
}

void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN); // Disable the ADC module
  ADCSRB = (1 << ACME); //Enables the ADC multiplexer and selects the analog channel specified by ADMUX.
  ADMUX = 2; // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01; //Configures the Analog Comparator Control and Status Register (ACSR) to enable interrupts on a falling edge of the comparator output.
}

void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN); // Disable the ADC module
  ADCSRB = (1 << ACME); //
  ADMUX = 3; // Select analog channel 3 as comparator negative input
  ACSR |= 0x03; //Configures the Analog Comparator Control and Status Register (ACSR) to enable interrupts on a rising edge of the comparator output.
}

void AH_BL() {
  PORTB  =  0x04; //Sets PB2 (pin 10) HIGH, which might be one of the high-side switches for phase A.
  PORTD &= ~0x18;  //Clears PD3 (pin 5) and PD4 (pin 4) to LOW, turning off these pins.
  PORTD |=  0x20; //Sets PD5 (pin 3) HIGH, which might be a low-side switch for phase B.
  TCCR1A =  0; //Disables PWM on Timer1, ensuring pins 9 and 10 are not using PWM
  TCCR2A =  0x81; //Enables PWM on Timer2A (pin 11), configuring this timer for phase control.
}

void AH_CL() {
  PORTB  =  0x02; //Sets PB1 (pin 9) HIGH, which might be another high-side switch for phase A.
  PORTD &= ~0x18; //Clears PD3 (pin 5) and PD4 (pin 4) to LOW.
  PORTD |=  0x20; //Sets PD5 (pin 3) HIGH.
  TCCR1A =  0; //Disables PWM on Timer1.  
  TCCR2A =  0x81; //Enables PWM on Timer2A (pin 11).
}

void BH_AL() {
  PORTB  =  0x08; //Sets PB3 (pin 11) HIGH, which might be another high-side switch for phase B.
  PORTD &= ~0x28; //Clears PD3 (pin 5) and PD5 (pin 3) to LOW.
  PORTD |=  0x10; //Sets PD4 (pin 4) HIGH.
  TCCR2A =  0; //Disables PWM on Timer2A.          
  TCCR1A =  0x21; //Enables PWM on Timer1B (pin 10).
}

void BH_CL() {
  PORTB  =  0x02; //Sets PB1 (pin 9) HIGH, which might be a high-side switch for phase B.
  PORTD &= ~0x28; //Clears PD3 (pin 5) and PD5 (pin 3) to LOW.
  PORTD |=  0x10; //Sets PD4 (pin 4) HIGH.
  TCCR2A =  0; //Disables PWM on Timer2A.       
  TCCR1A =  0x21; //Enables PWM on Timer1B (pin 10), configuring this timer for phase control.
}

void CH_AL() {
  PORTB  =  0x08; // Sets PB3 (pin 11) HIGH, which might be a high-side switch for phase C.
  PORTD &= ~0x30; //Clears PD3 (pin 5) and PD4 (pin 4) to LOW.
  PORTD |=  0x08; //Sets PD3 (pin 5) HIGH.
  TCCR2A =  0; //Disables PWM on Timer2A.      
  TCCR1A =  0x81; //Enables PWM on Timer1A (pin 9), configuring this timer for phase control.
}

void CH_BL() {
  PORTB  =  0x04; //Sets PB2 (pin 10) HIGH, which might be a high-side switch for phase C.
  PORTD &= ~0x30; //Clears PD3 (pin 5) and PD4 (pin 4) to LOW.
  PORTD |=  0x08; //Sets PD3 (pin 5) HIGH.
  TCCR2A =  0; //Disables PWM on Timer2A.      
  TCCR1A =  0x81; //Enables PWM on Timer1A (pin 9).
}

/*NOTES:
These functions,control the BLDC motor phases by setting the appropriate pins and configuring the timers for PWM output
These functions control which phases of the BLDC motor are activated and configure the PWM settings to drive the motor:
PORTB and PORTD: Set specific pins to HIGH or LOW to control the phases.
TCCR1A and TCCR2A: Configure Timer1 and Timer2 for PWM output on specific pins.
By cycling through these functions in the bldc_move function, the motor is commutated properly, ensuring smooth operation.
*/

void SET_PWM_DUTY(byte duty) {
  if(duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY; //This prevents the duty cycle from being too low, which might be ineffective for driving the motor.
  if(duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY; //This prevents the duty cycle from exceeding the maximum allowable value, which is typically 255 for an 8-bit PWM signal.
  OCR1A = duty; //Sets the duty cycle for Timer1, Channel A, which controls the PWM output on pin 9.
  OCR1B = duty; //Sets the duty cycle for Timer1, Channel B, which controls the PWM output on pin 10.
  OCR2A = duty; //Sets the duty cycle for Timer2, Channel A, which controls the PWM output on pin 11.
}
/*NOTES:
In the context of Pulse Width Modulation (PWM) and motor control, "duty" refers to the duty cycle of a PWM signal. 
The duty cycle is a measure of how long a signal is active (high) compared to the total period of the signal. 
It's typically expressed as a percentage.
OCR1A:(Output Compare Function) When the value in the counter matches the value in OCR1A, an action is triggered, 
such as toggling a pin, generating a PWM signal, or setting a flag.
*/

void bepper(){
  const int beep_duration = 100; //The duration of each beep in milliseconds (100 ms).
  const int pause_duration = 200; //The duration of the pause between beeps in milliseconds (200 ms). Note: This constant is defined but not used in the function.
  const int num_beeps = 2; //The number of beeps to be emitted (2 beeps).

 for (int j = 0; j < num_beeps; j++){ //Loop runs num_beeps times, which is 2 times in this case.
   SET_PWM_DUTY(PWM_MAX_DUTY); //Sets the PWM duty cycle to the maximum value (PWM_MAX_DUTY), which turns the beeper on at full power.
   delay(beep_duration); //Waits for beep_duration milliseconds (100 ms) while the beeper is on.
   SET_PWM_DUTY(0); //Sets the PWM duty cycle to zero, turning the beeper off.
  }
}







