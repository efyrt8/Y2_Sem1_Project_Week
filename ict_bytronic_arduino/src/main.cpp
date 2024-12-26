#include <Arduino.h>

// SORTING
const int METAL_PEG_DETECT = 47; // 0 = No metal, 1 = metal
const int SORT_AREA_DETECT = 40; // 0 = Empty, 1 = Object Detected
const int SORT_SOLENOID_RETURN = 44; // 0 = Eject, 1 = Extract
// HOOPER DETECT
const int ASSEMBLY_HOPPER_FULL = 48; // 0 = Detect object, 1 = No object detected
// ASSEMBLY DETECT POINT
const int RING_ASSEMBLY = 41; // 0 = Assembled, 1 = Else
const int ASSEMBLY_DETECT = 49; // 0 = Nothing, 1 = Detect object
const int BELT_PEG_DETECT = 51; // 1 = Nothing, 0 = Detect object
// LATCH SWITCH
const int ON_SWITCH = 50; // 0 = Not push, 1 = Push
const int OFF_SWITCH = 43; // 0 = Push, 1 = Not Push
// REJECT POINT
const int REJECT_AREA_DETECT = 42; // 0 = Nothing, 1 = Object Detected
const int REJECT_SOLENOID_RETURN = 52; // 0 = Eject, 1 = Extract
// BELT
const int CHAIN_CONVEYOR = 6;
const int BELT_CONVEYOR = 8;
// SORTING
const int SORT_SOLENOID = 24;
// HOOPER REJECT
const int ROTARY_SOLENOID = 7;
// NON-ASSEMBLY REJECT
const int REJECT_SOLENOID = 23;

// ROBOT ARM SIGNAL
const int robot_arm_signal = 24;

// Variable
int ON_STATE;
int OFF_STATE;
int flag = 0;
int metal_state = 0;
int plastic_count = 0;
const int max_plastic = 5;
int metal_check;
int plastic_check;
int assembly_check;
int previous_assembly_check = 0;
int assembly_flag = -1;
int current_metal_check;
int current_plastic_check;

// Timer reset interval variable
const int sort_solenoid_reset = 300;
const int assembly_hopper_load = 1200;
const int hopper_release_reset = 400;
const int assembly_state_timeout = 1000;
const int reject_solenoid_reset = 300;

// Assembly status array
const int array_size = 10;
int status_array[array_size]; // initialize array to empty

void setup() {
  Serial.begin(9600);
  pinMode(METAL_PEG_DETECT, INPUT);
  pinMode(SORT_AREA_DETECT, INPUT);
  pinMode(ASSEMBLY_HOPPER_FULL, INPUT);
  pinMode(RING_ASSEMBLY, INPUT);
  pinMode(ASSEMBLY_DETECT, INPUT);
  pinMode(BELT_PEG_DETECT, INPUT);
  pinMode(ON_SWITCH, INPUT);
  pinMode(OFF_SWITCH, INPUT);
  pinMode(REJECT_AREA_DETECT, INPUT);
  pinMode(SORT_SOLENOID_RETURN, INPUT);
  pinMode(REJECT_SOLENOID_RETURN, INPUT);

  pinMode(CHAIN_CONVEYOR, OUTPUT);
  pinMode(BELT_CONVEYOR, OUTPUT);
  pinMode(SORT_SOLENOID, OUTPUT);
  pinMode(ROTARY_SOLENOID, OUTPUT);
  pinMode(REJECT_SOLENOID, OUTPUT);
  pinMode(robot_arm_signal, OUTPUT);

  // Initialize status_array to -1
  for (int i = 0; i < array_size; i++){
    status_array[i] = -1;
  }
}

// Timer function for plastic in hopper channel
int timer(unsigned long previous_millis, unsigned long output_type) {
  unsigned long currentMillis = millis();

  if (currentMillis - previous_millis >= output_type) {
    // Reset timer to 0 after reaching the interval
    previous_millis = currentMillis;  
    return 0;
  }
  return 1;
}

// Switch function
int latch_switch() {
  int ON_STATE = digitalRead(ON_SWITCH);
  int OFF_STATE = digitalRead(OFF_SWITCH);

  if (ON_STATE == HIGH && OFF_STATE == HIGH){
    digitalWrite(robot_arm_signal, HIGH);
    return flag = 1;
  } else if (ON_STATE == LOW && OFF_STATE == LOW) {
    digitalWrite(robot_arm_signal, LOW);
    return flag = 0;
  } else {
    return flag = flag;
  }
}

// On or Off conveyor
void conveyor(){
  digitalWrite(CHAIN_CONVEYOR, flag);
  digitalWrite(BELT_CONVEYOR, flag);
}

// Sort metal and plastic
void metal_detect() {
  // Activation time for sort_solenoid
  static unsigned long metal_time = 0;
  static bool sort_solenoid_active = false;

  // Update metal_state if metal is detected
  if (digitalRead(METAL_PEG_DETECT) == HIGH) {
    metal_state = 1; 
  } 
  
  // Activate solenoid when metal_state is 0, sort detect is 1, and plastic count is within limit
  if (metal_state == 0 && digitalRead(SORT_AREA_DETECT) == HIGH && plastic_count <= max_plastic) {
    analogWrite(SORT_SOLENOID, 160); 
    metal_time = millis();
    sort_solenoid_active = true;
  }

  // Check if the solenoid is on and turn it off after the reset interval has passed
  if (sort_solenoid_active) {
    if (millis() - metal_time >= sort_solenoid_reset) {
      analogWrite(SORT_SOLENOID, 0);  
      plastic_count += 1;  
      sort_solenoid_active = false;        
    }
  }

  // Reset metal_state when the sort area is clear
  if (digitalRead(SORT_AREA_DETECT) == LOW) {
    metal_state = 0;
  }
}

// Release plastic to hopper
void hopper_release() {
  static unsigned long rotary_solenoid_time = 0; // Activation time for rotary_solenoid
  static unsigned long hopper_channel_time = 0;  // Time for plastic in hopper channel
  static bool rotary_solenoid_active = false;
  static bool hopper_timer = false;

  // Check if the sort solenoid was recently activated and start the delay timer
  if (!hopper_timer) {
    hopper_channel_time = millis(); 
    hopper_timer = true; 
  }

  // After 1 second, activate the rotary solenoid if the assembly hopper is not full
  if (hopper_timer && timer(hopper_channel_time, assembly_hopper_load) == 0) {
    if (digitalRead(ASSEMBLY_HOPPER_FULL) == LOW) {
      digitalWrite(ROTARY_SOLENOID, HIGH);  
      rotary_solenoid_time = millis();    
      rotary_solenoid_active = true;  
    }   
  }

  // Turn off the rotary solenoid after the release reset interval
  if (rotary_solenoid_active && (millis() - rotary_solenoid_time >= hopper_release_reset)) {
    digitalWrite(ROTARY_SOLENOID, LOW); 
    rotary_solenoid_active = false;
    plastic_count = max(0, plastic_count - 1);  // Reduce plastic count
    hopper_timer = false;
  }
}

// Add assembly_status to array
void add_status_array(int status){
  for (int i = array_size -1; i>0; i--){
    status_array[i] = status_array[i-1];
  }
  status_array[0] = status;
}

// Check assembly
void assembly_screen() {
  static unsigned long assembly_start_time = 0; 
  static bool assembly_timer_active = false;  

  current_metal_check = digitalRead(BELT_PEG_DETECT);
  current_plastic_check = digitalRead(RING_ASSEMBLY);
  assembly_check = digitalRead(ASSEMBLY_DETECT);

  // Start the 1-second timer if assembly_flag is -1 and no timer is active
  if (assembly_flag == -1 && !assembly_timer_active) {
    metal_check = current_metal_check;
    plastic_check = current_plastic_check;
    assembly_start_time = millis();
    assembly_timer_active = true; // Timer start
  }
  // Check if the 1-second timer has elapsed
  if (assembly_timer_active) {
    if (millis() - assembly_start_time >= assembly_state_timeout) {
      // Timeout, reset timer
      assembly_timer_active = false;     
    } 
    else {
      // Perfect assembly
      if (current_metal_check == 1 && current_plastic_check == 0) {
        assembly_flag = 1; 
        assembly_timer_active = false; // Stop the timer
      } 
      // Only metal
      else if (current_metal_check == 1 && current_plastic_check == plastic_check) {
        assembly_flag = 0;
        assembly_timer_active = false; // Stop the timer
      }
      // Only plastic
      else if (previous_assembly_check == 1 && assembly_check == 0 && assembly_flag == -1){
        assembly_flag = 2;
        assembly_timer_active = false;
      }
    }
  }
  // Add flag to array if prev check is 1 and current check is 1
  if (previous_assembly_check == 1 && assembly_check == 0){
    add_status_array(assembly_flag);
    assembly_flag = -1;
  }
  // Update prev assembly check
  previous_assembly_check = assembly_check;
}

// Reject metal or plastic
void reject_object(){
  static unsigned long current_solenoid_time = 0;
  static bool reject_area_flag = false;
  static bool reject_solenoid_active = false;
  int last_valid_index = -1;

  // Check for last element in array (not -1)
  for (int i = array_size -1; i>=0; i--){
    if (status_array[i] != -1){
      last_valid_index = i;
      break;
    }
  }

  // Reject if assembly_status is 0 or 2
  if (last_valid_index != -1){
    if (digitalRead(REJECT_AREA_DETECT) == HIGH){
      // Imperfect assembly
      if (status_array[last_valid_index] == 0 || status_array[last_valid_index] == 2){
        analogWrite(REJECT_SOLENOID, 220);
        current_solenoid_time = millis();
        reject_area_flag = true; // Raise flag, indicating assembly passing through reject area
        reject_solenoid_active = true;
      }
      // Perfect assembly
      else if (status_array[last_valid_index] == 1){
        reject_area_flag = true; // Raise flag, indicating assembly passing through reject area
      }
    }
    // Reset the last non -1 assembly_flag in array to -1 after the object leave the detection zone
    else if(digitalRead(REJECT_AREA_DETECT) == LOW && reject_area_flag){
      status_array[last_valid_index] = -1;
      reject_area_flag = false;
    }
  }

  // Reset solenoid if it is triggered
  if (reject_solenoid_active && (millis() - current_solenoid_time >= reject_solenoid_reset)){
    analogWrite(REJECT_SOLENOID, 0);
    reject_solenoid_active = false;
  }
}

void reset(){
  // Variable
  metal_state = 0;
  plastic_count = 0;
  previous_assembly_check = 0;
  assembly_flag = -1;

  // Array
  for (int i = 0; i < array_size; i++){
    status_array[i] = -1;
  }
}

void loop() {
  while (latch_switch()) {
    conveyor();
    metal_detect();
    if (plastic_count > 0){
      hopper_release();
    }
    assembly_screen();
    reject_object();
  }
  conveyor();
  reset();
}