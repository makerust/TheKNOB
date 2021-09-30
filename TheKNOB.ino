/* -- The Kinesthetic Oblate Novelty Button --
*/

#include <BleKeyboard.h>
BleKeyboard bleKeyboard("The KNOB", "Pangolin Design Team", 69);

//#define DEBUG

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
  uint8_t mailbox_length = MAILBOX_LENGTH;
  uint8_t last_used_index =0;
  for(uint8_t i = 0; i < (mailbox_length-1); i++){
    if (mailbox[i] ==0){
      last_used_index=i;
      break;
    }
  }
  return last_used_index;
}

int IRAM_ATTR key_detect(int enc_tick) {//remove whole keypresses from ISR and replace with buffer.
  noInterrupts();//disable interrupts

  int pin_state_now = 0;//byte pack of pinstates
  //-- encoder states are read first since they are fast
  // other button states read after debounce
  //[Button 2 | Button 1 | Encoder button | Encoder B | Encoder A]
  int enc_clk_now = pin_state_now |= (digitalRead(ENCODER_A) << 1);
  int enc_dt_now = pin_state_now |= digitalRead(ENCODER_B);  
  
  //-- variables for Debouncing signals
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  //-- Encoder gets special double debounce
  static unsigned long last_encoder_send_time =0;
  #define ENCODER_FLOP_COOLDOWN 500
  static int last_send = 0; //0 = non encoder, 1 = cw, 2 = ccw

  
  if(interrupt_time - last_interrupt_time > 3){//debounces interrupts

    last_interrupt_time = interrupt_time;
  
    //Build a byte of pin states
    //[Button 2 | Button 1 | Encoder button | Encoder B | Encoder A]
    // pin_state_now |= (!(digitalRead(BUTTON_2) << 4));
    // pin_state_now |= (!(digitalRead(BUTTON_1) << 3));
    // pin_state_now |= (!(digitalRead(ENCODER_BUTTON)) << 2);
    
    //Position stored here. 0: no change, 1: CW, -1: CCW
    int enc_position=0;
    static int enc_clk_prev =0;

    if (enc_clk_prev != enc_clk_now && enc_clk_now == 0)// 
      if(enc_dt_now != enc_clk_now){
        if((interrupt_time - last_encoder_send_time > ENCODER_FLOP_COOLDOWN) || last_send != 2){
          enc_position=1;
          last_send = 1;
          last_encoder_send_time = millis();
        }
      }
      else{
        if((interrupt_time - last_encoder_send_time > ENCODER_FLOP_COOLDOWN) || last_send != 1){
          enc_position =-1;
          last_send=2;
          last_encoder_send_time = millis();
        }
      }
//      else{
//        last_send=0;
//      }
    enc_clk_prev = enc_clk_now;
    
    //this encoder works weird. Trans low on each tick, A leads if cw, b leads if ccw. Reverts to vcc always!
    //This packs a byte where 
    //[ ISR ENC A | ISR ENC B | pin_state ENC A | pin_state ENC B]
    // int encode_state = ( (((enc_tick >> 1) & 1)<<3) |(((enc_tick >> 0) & 1)<<2) |(((pin_state_now >> 1) & 1)<<1) | ((pin_state_now >> 0) & 1));
    // switch (encode_state){
    //   case 5://0101 CW case
    //     if((interrupt_time - last_encoder_send_time > ENCODER_FLOP_COOLDOWN) || last_send != 2){
    //       //various iterations would cause some amount of false (opposite)
    //       //reads, e.g. rotating the encoder CW would produce mostly CW
    //       //read state with occasional CCW reads. This made usage frustrating
    //       //The tests requires a timeout between changing directions or a
    //       //same direction read before registering a new encoder position
    //       enc_position = 1;
    //       last_encoder_send_time = millis();
    //       last_send = 1;
    //     }
    //     break;
    //   case 11: //1011 CCW case I would expect this case to be 1010, but case 10 didn't work
    //     if((interrupt_time - last_encoder_send_time > ENCODER_FLOP_COOLDOWN) || last_send != 1){
    //       enc_position = (-1);
    //       last_encoder_send_time = millis();
    //       last_send = 2;
    //     }
    //     break;
    //   default:
    //     enc_position =0;
    //     break;
    // }

     



    uint8_t index_now = mb_search(key_mailbox);
    
    if (index_now <= (MAILBOX_LENGTH-1)){//Check to not overflow the mailbox array.
    // If service cannot empty in time, itmes are not added to mailbox
    //-- This function puts a message into for each distinct key event
    //[1: Function key press | 2: encoder CW with mod | 3: encoder CCW with mod| 4:]
      // if((pin_state_now & (1 << 4)) !=  0){//function key press.
      //    key_mailbox[index_now] = 1;
      // }
      if((pin_state_now & (1 << 3)) !=  0){ //if FFRW key pressed, encoder encodes for FFRW
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
      // if((pin_state_now & (1<<2)) != 0){//encoder button
      //   key_mailbox[index_now] = 4;
      // }
    }  
  }  
    interrupts();
    return 0;    
}

void IRAM_ATTR key_detectO(){
  noInterrupts();//disable interrupts

//  int encoder_button_state = digitalRead(ENCODER_BUTTON);
//  int function_button_state = digitalRead(BUTTON_2);
  
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
  interrupts();  
}

//Consistent Encoder decoding has required tracking
//which Pin triggered the ISR which created this hacky crap
void IRAM_ATTR key_detectA() {key_detect(1);}
void IRAM_ATTR key_detectB() {key_detect(2);}
//void IRAM_ATTR key_detectO() {key_detect(0);}


void setup() {
  for (int i = 0; i < MAILBOX_LENGTH; i++){
    key_mailbox[i]=0;
  }
  // make the pushButton pin an input:
  pinMode(BUTTON_2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_2), key_detectO, CHANGE);
  pinMode(BUTTON_1, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_1), key_detectO, CHANGE);
  pinMode(ENCODER_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON), key_detectO, FALLING);
  pinMode(ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), key_detectA, CHANGE);
  pinMode(ENCODER_B, INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_B), key_detectB, FALLING);

  
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
      #ifdef DEBUG
      Serial.print("Mailbox size: ");
      Serial.println(mailbox_index); 
      #endif
    }
    #ifdef DEBUG
    delay(300);
    #endif
     
  }
  else{
    #ifdef DEBUG
    Serial.println("Disconnected");
    delay(1000);
    #endif    
  }
}
