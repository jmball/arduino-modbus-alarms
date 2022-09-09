#define SYSTEM_NAME "G44"

#define MAC {0xA8, 0x61, 0x0A, 0xAE, 0x75, 0x00}

#define ADC_FLOW_IP { 192, 168, 1, 23 }
#define ADC_PRESSURE_IP { 192, 168, 1, 22 }
#define ADC_TEMPERATURE_IPS {{ 192, 168, 0, 29 }, { 192, 168, 0, 29 }, { 192, 168, 0, 29 }, { 192, 168, 0, 29 }, { 192, 168, 0, 29 }}
#define ADC_FLOW_PORT 502
#define ADC_PRESSURE_PORT 502
#define ADC_TEMPERATURE_PORT 2000

#define SOFTWARE_STATUS_IP { 192, 168, 1, 100 }
#define SOFTWARE_STATUS_PORT 6000

// modbus ID's
#define ADC_FLOW_ID 1
#define ADC_PRESSURE_ID 1
#define ADC_TEMPERATURE_ID 1

// pins 4, 10, 50, 51, 52 cannot be used with ethernet shield on the MEGA
#define LO_FLOW_ALARM_PIN 2
#define HI_ROUHGING_PRESSURE_ALARM_PIN 3
#define HI_TEMPERATURE_ALARM_PIN 5 
#define TURBO_50_PIN 6
#define COMMS_ERROR_ALARM_PIN 7
#define SOFTWARE_ERROR_ALARM_PIN 8

// Threshold ADC readings
#define ADC_FLOW_MIN 1500 // 0.15 lpm
#define ADC_ROUGHING_PRESSURE_MAX 7429 // 10 mbar
#define ADC_TEMPERATURE_MAX 600 // 600C
#define ADC_TURBO_50 5000 // 50% speed

#define FLOW_CHANNELS 6
#define TEMPERATURE_CHANNELS 8
#define ADC_ROUGHING_PRESSURE_CH 2
#define ADC_TURBO_50_CH 1
