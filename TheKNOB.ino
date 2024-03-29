/* -- The Kinesthetic Novelty Oblate Button --
*/

#include <driver/dac.h>
#include <BleKeyboard.h>//works with version 0.2.3 of the library
//https://github.com/T-vK/ESP32-BLE-Keyboard/tree/0.2.3
BleKeyboard bleKeyboard("The KNOB", "Pangolin Design Team", 69);
#include <Adafruit_NeoPixel.h>
#include "mailbox.hpp"


#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


//Pin defines
constexpr auto BATT_VOLT_PIN = 34;
constexpr auto CHG_STAT = 35;
constexpr auto USB_VOLT_PIN = 32;
constexpr auto BUTTON_1 = 33;
constexpr auto BUTTON_2 = 26;
constexpr auto ENCODER_BUTTON = 27;
constexpr auto ENCODER_B = 14;
constexpr auto ENCODER_A = 13;
constexpr auto LED_PIN = 4;


#define LED_COUNT 3


enum class Action : uint8_t{
  no_action = 0,
  fast_forward = 2,
  reverse = 3,
  volume_up = 5,
  volume_down = 6,
  function_press = 1,
  encoder_press = 4
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int wakeDelayB = 0;

/*
  ESP32 ADC Data:
  uses a 47k, 10k res div on bat voltage pin to scale 4.7v to 0.75v
  0-4095 bit depth
  ref 1085mV
  vi = (vo * (Rb + Rt))/ Rb
  scaling ratio = 5.7
  batt volt: 3.80v res div 640mV
  ADC 4095
*/

constexpr float adc_volt_div_correction = 5.7;
constexpr float refV = 1.085;
constexpr float precB = 4095;
constexpr float usbVoltThresh = 0.6;


float volt_read(uint8_t pin, float refV, float precB) {
  float batt_bits = analogRead(pin);
  float voltage = (batt_bits * refV) / precB;
  return voltage;
}

int batt_chg_percent(uint8_t pin, float refV, float precB, float resDivRatio) {
  //This method will be super flawed for a Li battery
  float batt_volt_roll = 0;
  int batt_buff_size = 30;
  for(int i =0; i <= (batt_buff_size-1); i++){
    batt_volt_roll += (resDivRatio * volt_read(pin, refV, precB));
    //delay(3);
  }

  float batt_volt_now = batt_volt_roll/batt_buff_size;
  //float batt_volt_now = resDivRatio * volt_read(pin, refV, precB);

  if (batt_volt_now >= 4.1 )
    return 100;
  else if (batt_volt_now >= 3.9)
    return 80;
  else if (batt_volt_now >= 3.8)
    return 60;
  else if (batt_volt_now >= 3.7)
    return 40;
  else
    return 20;
}

//Global mailbox array
mailbox<Action, 40> key_mailbox;

//This is used as a mutex in ESP code to handle interrupts
struct critical_section{
  static portMUX_TYPE mux;
  critical_section(){
    portENTER_CRITICAL(&mux);//disable interrupts
  }
  ~critical_section(){
    portEXIT_CRITICAL(&mux);//enable interrupts
  }
};
portMUX_TYPE critical_section::mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR enc_ISR() {
  auto c = critical_section{};

  //-- encoder states are read first since they are fast
  // other button state read after debounce
  int enc_clk_now = digitalRead(ENCODER_A);
  int enc_dt_now = digitalRead(ENCODER_B);

  static int enc_composit ;//byte pack of pinstates

  //-- variables for Debouncing signals
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 0) { //debounces interrupts| 0 WORKED VERY WELL
    last_interrupt_time = interrupt_time;

    //Build a byte of encoder states
    int button_state_now = (!digitalRead(BUTTON_1));

    enc_composit |= (enc_dt_now << 3);
    enc_composit |= (enc_clk_now << 2);

    //Position stored here. 0: no change, 1: CW, -1: CCW
    int enc_position = 0;

    switch (enc_composit) {
      case 2:
        enc_position = 1;
        break;
      case 1:
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

    //in practice, the mailbox is largely unnecessary as it never gets filled
    //past the first position
    if (!key_mailbox.is_full()) { //Check to not overflow the mailbox array.
      // If service cannot empty in time, itmes are not added to mailbox

      if (button_state_now == 1) { //if FFRW key pressed, encoder encodes for FFRW
        if (enc_position > 0) {
          key_mailbox.push_back(Action::fast_forward);
        }
        if (enc_position < 0) {
          key_mailbox.push_back(Action::reverse);
        }
      }
      else {
        if (enc_position > 0) { //if not pressed, encode volume
          key_mailbox.push_back(Action::volume_up);
        }
        else if (enc_position < 0) {
          key_mailbox.push_back(Action::volume_down);
        }
      }
    }
  }
  return;
}

void IRAM_ATTR key_detect() {
  auto c = critical_section{};
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();

  if (interrupt_time2 - last_interrupt_time2 > 300) { //debounces interrupts
    last_interrupt_time2 = interrupt_time2;
    int encoder_button_state = digitalRead(ENCODER_BUTTON);
    int function_button_state = digitalRead(BUTTON_2);

    if (!key_mailbox.is_full()) {
      if (function_button_state == 0) { //function key press.
        key_mailbox.push_back(Action::function_press);
      }
      if (encoder_button_state == 0) { //encoder key press.
        key_mailbox.push_back(Action::encoder_press);
      }
    }
  }
}


#define LED_BRIGHTNESS   5
uint32_t red = strip.ColorHSV(0, 255, LED_BRIGHTNESS);
uint32_t green = strip.ColorHSV(21845, 255, LED_BRIGHTNESS);
uint32_t blue = strip.ColorHSV(43690, 255, LED_BRIGHTNESS);
uint32_t white = strip.ColorHSV(0, 0, LED_BRIGHTNESS);
uint32_t orange = strip.ColorHSV(5461, 255, LED_BRIGHTNESS);


bool blink(bool led_stat, uint32_t color, uint32_t bkgd = 0) {
  unsigned long time_millis = millis();
  static unsigned long set_time_millis;
  if (led_stat == 0 && time_millis % 2750 == 1) {
    for (int i = 0;  i < (LED_COUNT); i++)
    {
      strip.setPixelColor(i, color);
    }
    strip.show();
    set_time_millis = time_millis;
    led_stat = 1;
  }
  if (led_stat == 1 && ((time_millis - set_time_millis) > 250)) {
    for (int j = 0; j < (LED_COUNT); j++)
    {
      strip.setPixelColor(j, bkgd);
    }
    strip.show();
    led_stat = 0;
  }
  return led_stat;
}

bool light(bool led_stat, uint32_t color) {
  for (int i = 0;  i < (LED_COUNT); i++)
  {
    strip.setPixelColor(i, color);
  }
  strip.show();
  led_stat = 1;

  return led_stat;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, white);
  }
  strip.show();

  key_mailbox.fill(Action::no_action);


  dac_output_disable(DAC_CHANNEL_1);
  dac_output_disable(DAC_CHANNEL_2);
  // Disable DAC1
  REG_CLR_BIT(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC);
  REG_SET_BIT(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC_XPD_FORCE);
  // Disable DAC2
  REG_CLR_BIT(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC);
  REG_SET_BIT(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC_XPD_FORCE);
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
  pinMode(BATT_VOLT_PIN, INPUT);


  // initialize control over the keyboard:
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
#endif

  bleKeyboard.begin();
  //strip.show();
  delay(200);

  analogSetAttenuation(ADC_0db);
  analogSetWidth(12);

  bleKeyboard.setBatteryLevel(batt_chg_percent(BATT_VOLT_PIN, refV, precB, 5.7));

}



void loop() {

  static uint32_t led_bkgd = 0;
  //timing variables for power saving
  static unsigned long last_send_time = 0;
#ifdef DEBUG
  unsigned long power_timeout_debug = 3000000;
#else
  unsigned long power_timeout_debug = 300000; //Five minutes in ms
#endif
  static bool led_on = 0;
  static bool usbPower = 0;

  if (wakeDelayB == 1) { //A few things to try to have a more seamless wake

    strip.setPixelColor(0, 0, 0, 12);
    strip.show();
    bleKeyboard.setBatteryLevel(batt_chg_percent(1, 1.2, 4096, 5.7));//mostly works
    DEBUG_PRINT("Battery level ");
    DEBUG_PRINT(batt_chg_percent(BATT_VOLT_PIN, 1.1, 4095, 5.7));
    DEBUG_PRINTLN("%");
    DEBUG_PRINTLN("Waking up");
    delay(1000);
    wakeDelayB = 0;
    usbPower = 0;

  }

  usbPower = (volt_read(USB_VOLT_PIN, refV, precB) > usbVoltThresh);

  if (bleKeyboard.isConnected() && wakeDelayB == 0) {

    if (usbPower) {
      led_on = light(led_on, green);
    }
    else {
      led_on = blink(led_on, green, led_bkgd);
    }

    auto count = key_mailbox.count();
    if (count != 0) {
      DEBUG_PRINT("Send Mailbox size: ");
      DEBUG_PRINTLN(count);

      for (auto i = count; i > 0; i--) {
        switch (key_mailbox.pop_front()) {
          case Action::function_press:
            bleKeyboard.write(KEY_F8);
            DEBUG_PRINTLN("Send Function Key");
            break;
          case Action::fast_forward:
            bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
            DEBUG_PRINTLN("Send FF Key");
            break;
          case Action::reverse:
            bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
            DEBUG_PRINTLN("Send RW Key");
            break;
          case Action::encoder_press:
            bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
            DEBUG_PRINTLN("Send Play Pause Key");
            break;
          case Action::volume_up:
            bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
            DEBUG_PRINTLN("Send UP Key");
            break;
          case Action::volume_down:
            bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
            DEBUG_PRINTLN("Send DOWN Key");
            break;
          default:
            break;
        }
        last_send_time = millis();
      }
    }
  }

  else {
    if (usbPower) {
      led_on = light(led_on, red);
    }
    else {
      led_on = blink(led_on, red, led_bkgd);
    }
    if (millis() % 1000 == 1) {
      DEBUG_PRINTLN("Disconnected");
      DEBUG_PRINT("Battery voltage :");
      DEBUG_PRINTLN(5.7 * volt_read(BATT_VOLT_PIN, refV, precB));
      DEBUG_PRINT("ADC raw read :");
      DEBUG_PRINTLN(analogRead(BATT_VOLT_PIN));
      DEBUG_PRINT("USB Power? ");
      DEBUG_PRINTLN(usbPower);
      DEBUG_PRINT("USB voltage: ");
      DEBUG_PRINTLN(5.7 * volt_read(USB_VOLT_PIN, refV, precB));
    }
  }

  //--sleep loop
  if ( (millis()) - last_send_time > power_timeout_debug) {
    strip.setPixelColor(0, blue);
    strip.show();
    DEBUG_PRINTLN("Enter sleep mode");
    wakeDelayB = 1;
    delay(200);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
    usbPower = 0;
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 0);//Needs to be the same number as ENCODER_BUTTON
    esp_deep_sleep_start();
  }

  //-- battery charge update
  if (millis() % 600 == 1) {//60000
    static int batt_chg = 69;
    int batt_chg_now =  batt_chg_percent(BATT_VOLT_PIN, refV, precB, 5.7);
    if (batt_chg_now != batt_chg or batt_chg == 69) {
      batt_chg = batt_chg_now;
      bleKeyboard.setBatteryLevel(batt_chg);
      DEBUG_PRINT("Battery Level set to ");
      DEBUG_PRINTLN(batt_chg);
      if (batt_chg <= 20){
        led_bkgd = orange;
        DEBUG_PRINTLN("Bkgd color changed to orange.");
      }
      else{
        led_bkgd = 0;
      }
    }
  }
}
