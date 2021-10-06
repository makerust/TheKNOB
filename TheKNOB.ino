/* -- The Kinesthetic Oblate Novelty Button --
*/

#include <BleKeyboard.h>//works with version 0.2.3 of the library
//https://github.com/T-vK/ESP32-BLE-Keyboard/tree/0.2.3
BleKeyboard bleKeyboard("The KNOB", "Pangolin Design Team", 69);

#define DEBUG

//Pin defines
#define ENCODER_A ((uint8_t) 18)
#define ENCODER_B ((uint8_t) 17)
#define ENCODER_BUTTON ((uint8_t) 16)
#define BUTTON_1 ((uint8_t) 1)
#define BUTTON_2 ((uint8_t) 2) 

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

void IRAM_ATTR enc_ISR() {//remove whole keypresses from ISR and replace with buffer.
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
    //-- This function puts a message into for each distinct key event
 
//      if(button_state_now == 1){ //if FFRW key pressed, encoder encodes for FFRW
//        if(enc_position > 0){
//          key_mailbox[index_now] = 2;
//        }
//        if(enc_position < 0){
//          key_mailbox[index_now] = 3;
//        }   
//      }

      if(enc_position > 0){//if not pressed, encode volume
        key_mailbox[index_now] = 5;
      }
      else if(enc_position < 0){
        key_mailbox[index_now] = 6;
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
  for (int i = 0; i < MAILBOX_LENGTH; i++){
    key_mailbox[i]=0;    
  }
  // make the pushButton pin an input:
  pinMode(BUTTON_2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_2), key_detect, CHANGE);
  pinMode(BUTTON_1, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_1), key_detect, CHANGE);
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
  bleKeyboard.print("Connected");
}



void loop() {  
  if(bleKeyboard.isConnected()){
    uint8_t mailbox_index =mb_search(key_mailbox);
    if(mailbox_index != 0){ 
      #ifdef DEBUG
      Serial.print("Send Mailbox size: ");
      Serial.println(mailbox_index);    
      #endif
      for( int i = 0; i < (MAILBOX_LENGTH-1); i++){//This can be replaced with logic that only iterates over mailbox_index, but to start with, clear the WHOLE mailbox
        switch (key_mailbox[i]){
          case 1: 
            bleKeyboard.write(KEY_F23);
            #ifdef DEBUG
            Serial.println("Send Function Key");
            #endif
            break;
          case 2:
            bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
            #ifdef DEBUG
            Serial.println("Send FF Key");
            #endif
            break;
          case 3:
            bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
            #ifdef DEBUG
            Serial.println("Send RW Key");
            #endif
            break;
          case 4:
            bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
            #ifdef DEBUG
            Serial.println("Send Play Pause Key");
            #endif
            break;
          case 5:
            bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
            #ifdef DEBUG
            Serial.println("Send UP Key");
            #endif
            break;
          case 6:
            bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
            #ifdef DEBUG
            Serial.println("Send DOWN Key");
            #endif
            break;
          default:
            break;
        }
        key_mailbox[i] = 0;     
      }
    }
    else{

    }
     
  }
  else{
    #ifdef DEBUG
    Serial.println("Disconnected");
    delay(1000);
    #endif    
  }
}
