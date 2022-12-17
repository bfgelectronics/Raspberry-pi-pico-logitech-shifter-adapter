#include "Adafruit_TinyUSB.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define Y_AXIS 26
#define X_AXIS 27
#define R_BUTTON 22


#define GAMEPAD_REPORT(...) \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
    HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD), \
    HID_COLLECTION(HID_COLLECTION_APPLICATION), /* Report ID if any */ \
    __VA_ARGS__ \
 /* 8 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN    ( 0x81                                   ) ,\
    HID_LOGICAL_MAX    ( 0x7f                                   ) ,\
    HID_REPORT_COUNT   ( 6                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit DPad/Hat Button Map  */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
    HID_LOGICAL_MIN    ( 1                                      ) ,\
    HID_LOGICAL_MAX    ( 8                                      ) ,\
    HID_PHYSICAL_MIN   ( 0                                      ) ,\
    HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
    HID_REPORT_COUNT   ( 1                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 10 buttons settup */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 10                                     ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 32                                     ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    HID_COLLECTION_END

#define ADC_MAX_VAL 4096

#define XAXIS_LEFT_THRESH (int)(0.3 * ADC_MAX_VAL)
#define XAXIS_RIGHT_THRESH (int)(0.6 * ADC_MAX_VAL)
#define YAXIS_UP_THRESH (int)(0.7 * ADC_MAX_VAL)
#define YAXIS_DOWN_THRESH (int)(0.3 * ADC_MAX_VAL)

enum gear { N = 0,
            G1 = 1,
            G2 = 2,
            G3 = 3,
            G4 = 4,
            G5 = 5,
            G6 = 6,
            R = 7 };

uint8_t const desc_hid_report[] = {
  GAMEPAD_REPORT()
};

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);

hid_gamepad_report_t gp;

int y_axis_val = 0;
int x_axis_val = 0;

int last_gear = 0;

void shift_gear(int current_gear) {
  if (last_gear != current_gear) {
    Serial.println(current_gear);
     gp.buttons = (0b1 << current_gear);
     usb_hid.sendReport(0, &gp, sizeof(gp));
    last_gear = current_gear;
    delay(100);
  }
}

void read_gear() {
  adc_select_input(0);
  x_axis_val = adc_read();
  adc_select_input(1);
  y_axis_val = adc_read();

  if (digitalRead(R_BUTTON))
    return shift_gear(gear::R);

  if (y_axis_val < YAXIS_DOWN_THRESH) {
    if (x_axis_val < XAXIS_LEFT_THRESH)
      return shift_gear(gear::G2);

    if (x_axis_val > XAXIS_LEFT_THRESH && x_axis_val < XAXIS_RIGHT_THRESH)
      return shift_gear(gear::G4);

    if (x_axis_val > XAXIS_RIGHT_THRESH)
      return shift_gear(gear::G6);
  }

  if (y_axis_val > YAXIS_DOWN_THRESH && y_axis_val < YAXIS_UP_THRESH)
    return shift_gear(gear::N);


  if (y_axis_val > YAXIS_UP_THRESH) {
    if (x_axis_val < XAXIS_LEFT_THRESH)
      return shift_gear(gear::G1);

    if (x_axis_val > XAXIS_LEFT_THRESH && x_axis_val < XAXIS_RIGHT_THRESH)
      return shift_gear(gear::G3);

    if (x_axis_val > XAXIS_RIGHT_THRESH)
      return shift_gear(gear::G5);
  }
}

void setup() {
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  Serial.begin(115200);

  
  pinMode(R_BUTTON, INPUT);

  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);

  usb_hid.begin();
  delay(5000);

  gp.buttons = (0b1);
  usb_hid.sendReport(0, &gp, sizeof(gp));
}

void loop() {
  read_gear();
}