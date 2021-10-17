/* -- The Kinesthetic Oblate Novelty Button --
*/

#include <BleKeyboard.h>//works with version 0.2.3 of the library
//https://github.com/T-vK/ESP32-BLE-Keyboard/tree/0.2.3
BleKeyboard bleKeyboard("The KNOB", "Pangolin Design Team", 69);

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


//Pin defines
#define ENCODER_A ((uint8_t) 13)
#define ENCODER_B ((uint8_t) 14)
#define ENCODER_BUTTON ((uint8_t) 27)
#define BUTTON_1 ((uint8_t) 32)
#define BUTTON_2 ((uint8_t) 33)
#define BATT_VOLT_PIN ((uint8_t) 1) 


RTC_DATA_ATTR int bootCount =0;
RTC_DATA_ATTR int wakeDelayB =0; 

/*
ESP32 ADC Data:
https://deepbluembedded.com/esp32-adc-tutorial-read-analog-voltage-arduino/
use a 46k, 10k res div on bat voltage pin to scale 4.7v to 0.75v
*/

float batt_volt_read(uint8_t pin, float refV, uint8_t precB){
  float batt_bits = analogRead(pin);
  float voltage = batt_bits * (refV / static_cast <float>(precB));
  return voltage;
}

int batt_chg_percent(uint8_t pin, float refV, uint8_t precB){
  //This method will be super flawed for a Li battery
  float batt_volt_now = batt_volt_read(pin, refV, precB);
  int chg_percent;
  if(batt_volt_now >= 4.1 )
    chg_percent = 100;
  else if (batt_volt_now >= 3.9 && batt_volt_now < 4.1)
    chg_percent = 80;
  else if (batt_volt_now >= 3.8 && batt_volt_now < 3.9)
    chg_percent = 60;
  else if (batt_volt_now >= 3.7 && batt_volt_now < 3.8)
    chg_percent = 40;
  else if (batt_volt_now < 3.7)
    chg_percent = 20;
  return chg_percent;
}


//Global mailbox array
#define MAILBOX_LENGTH 40
uint8_t key_mailbox[MAILBOX_LENGTH];

uint8_t mb_search(uint8_t mailbox[]){
  uint8_t mailbox_length = MAILBOX_LENGTH;//pass this, what were you thinking?
  uint8_t last_used_index =0;
  for(uint8_t i = 0; i < (mailbox_length-1); i++){
    if (mailbox[i] ==0){
      last_used_index=i;
      break;
    }
  }
  return last_used_index;
}

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR enc_ISR() {
  portENTER_CRITICAL(&mux);//disable interrupts

  //-- encoder states are read first since they are fast
  // other button state read after debounce
  int enc_clk_now = digitalRead(ENCODER_A);
  int enc_dt_now = digitalRead(ENCODER_B); 
   
  static int enc_composit ;//byte pack of pinstates
  
  //-- variables for Debouncing signals
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  if(interrupt_time - last_interrupt_time > 0){//debounces interrupts| 0 WORKED VERY WELL
    last_interrupt_time = interrupt_time;
  
    //Build a byte of encoder states
    int button_state_now = (!digitalRead(BUTTON_1));

    enc_composit |= (enc_dt_now << 3);
    enc_composit |= (enc_clk_now << 2);
    
    //Position stored here. 0: no change, 1: CW, -1: CCW
    int enc_position=0;

    switch (enc_composit) {
      case 2: case 4: case 11: case 13:
        enc_position = 1;
        break;
      case 1: case 7: case 8: case 14:
        enc_position = -1;
        break;
      default:
        break;
    }
    enc_composit = enc_composit >> 2;

/* -- Regarding the cases

    0 0 0 0 | 0 | X
    0 0 0 1 | 1 | CCW
    0 0 1 0 | 2 | CW
    0 0 1 1 | 3 | X
    0 1 0 0 | 4 | CW
    0 1 0 1 | 5 | X
    0 1 1 0 | 6 | X
    0 1 1 1 | 7 | CCW
    1 0 0 0 | 8 | CCW
    1 0 0 1 | 9 | X
    1 0 1 0 | 10| X
    1 0 1 1 | 11| CW
    1 1 0 0 | 12| X
    1 1 0 1 | 13| CW
    1 1 1 0 | 14| CCW
    1 1 1 1 | 15| X

*/    

    uint8_t index_now = mb_search(key_mailbox);
    
    if (index_now <= (MAILBOX_LENGTH-1)){//Check to not overflow the mailbox array.
    // If service cannot empty in time, itmes are not added to mailbox
 
      if(button_state_now == 1){ //if FFRW key pressed, encoder encodes for FFRW
        if(enc_position > 0){
          key_mailbox[index_now] = 2;
        }
        if(enc_position < 0){
          key_mailbox[index_now] = 3;
        }   
      }
      else{
        if(enc_position > 0){//if not pressed, encode volume
          key_mailbox[index_now] = 5;
        }
        else if(enc_position < 0){
          key_mailbox[index_now] = 6;
        }
      }
    }  
  }  
    portEXIT_CRITICAL(&mux);//reenable interrupts
    return;    
}

void IRAM_ATTR key_detect(){
  portENTER_CRITICAL(&mux);//disable interrupts  
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();

  if(interrupt_time2 - last_interrupt_time2 > 300){//debounces interrupts
    last_interrupt_time2 = interrupt_time2;
    int encoder_button_state = digitalRead(ENCODER_BUTTON);
    int function_button_state = digitalRead(BUTTON_2);
    uint8_t index_now = mb_search(key_mailbox);
      
    if (index_now <= (MAILBOX_LENGTH-1)){
      if(function_button_state == 0){//function key press.
        key_mailbox[index_now] = 1;
        index_now++;
        }
      if(encoder_button_state == 0){//encoder key press.
        key_mailbox[index_now] = 4;
        index_now++;
        }
     }
  }
  portEXIT_CRITICAL(&mux);//enable interrupts  
}

void setup() {
  //clear the mailbox on boot
  for (int i = 0; i < MAILBOX_LENGTH; i++){
    key_mailbox[i]=0;    
  }

  // make the pushButton pin an input:
  pinMode(BUTTON_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), key_detect, FALLING);
  pinMode(BUTTON_1, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_1), key_detect, FALLING);
  pinMode(ENCODER_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON), key_detect, FALLING);
  pinMode(ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), enc_ISR, CHANGE);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), enc_ISR, CHANGE);

  // initialize control over the keyboard:
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  #endif

  bleKeyboard.begin();

}



void loop() {
  //timing variables for power saving
  static unsigned long last_send_time = 0;
  unsigned long now_send_time;
  unsigned long power_timeout_debug = 30000; //600000;// Ten minutes in ms  

  if(wakeDelayB == 1){//A few things to try to have a more seamless wake
    //bleKeyboard.write(KEY_F22);//this mostly works
    //bleKeyboard.begin();
    bleKeyboard.setBatteryLevel(batt_chg_percent(1, 1.2, 4096));//mostly works
    DEBUG_PRINT("Battery level ");
    DEBUG_PRINT(batt_chg_percent(1, 1.2, 4096));
    DEBUG_PRINTLN("%");
    DEBUG_PRINTLN("Waking up");
    delay(1000);
    wakeDelayB = 0;
  }

  if(bleKeyboard.isConnected() && wakeDelayB ==0){
    uint8_t mailbox_index = mb_search(key_mailbox);
    if(mailbox_index != 0){ 
      DEBUG_PRINT("Send Mailbox size: ");
      DEBUG_PRINTLN(mailbox_index);

      for( int i = 0; i < (MAILBOX_LENGTH-1); i++){//This can be replaced with logic that only iterates over mailbox_index, but to start with, clear the WHOLE mailbox
        switch (key_mailbox[i]){
          case 1: 
            bleKeyboard.write(KEY_F23);
            DEBUG_PRINTLN("Send Function Key");
            break;
          case 2:
            bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
            DEBUG_PRINTLN("Send FF Key");
            break;
          case 3:
            bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
            DEBUG_PRINTLN("Send RW Key");
            break;
          case 4:
            bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
            DEBUG_PRINTLN("Send Play Pause Key");
            break;
          case 5:
            bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
            DEBUG_PRINTLN("Send UP Key");
            break;
          case 6:
            bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
            DEBUG_PRINTLN("Send DOWN Key");
            break;
          default:
            break;
        }
        key_mailbox[i] = 0;
        last_send_time = millis();     
      }
    }

    //--sleep loop
    now_send_time = millis();
    if( now_send_time - last_send_time > power_timeout_debug){
      DEBUG_PRINTLN("Enter sleep mode");
      wakeDelayB = 1;
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 0);//Needs to be the same number as ENCODER_BUTTON
      esp_deep_sleep_start();      
    }

    if (millis() % 60000 == 1){
      static int batt_chg = 69;
      int batt_chg_now = batt_chg_percent(1, 1.2, 4096);
      if (batt_chg_now != batt_chg){
        batt_chg = batt_chg_now;
        bleKeyboard.setBatteryLevel(batt_chg);
        DEBUG_PRINT("Battery Level set to ");
        DEBUG_PRINTLN(batt_chg);
      }
    }

     
  }
  else{
    if (millis() % 1000 == 1){
      DEBUG_PRINTLN("Disconnected"); 
    }   
  }
}
