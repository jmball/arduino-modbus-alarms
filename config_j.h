#define MAC {0xA8, 0x61, 0x0A, 0xAE, 0x74, 0x8A}

#define ADC_FLOW_IP { 172, 16, 0, 128 }
#define ADC_PRESSURE_IP { 172, 16, 0, 108 }
#define ADC_TEMPERATURE_IP { 172, 16, 0, 129 }

#define SOFTWARE_STATUS_IP { 172, 16, 0, 112 }
#define SOFTWARE_STATUS_PORT 6000

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
#define ADC_ROUGHING_PRESSURE_MAX 6143 // 1 mbar
#define ADC_TEMPERATURE_MAX 6000 // 600C
#define ADC_TURBO_50 5000 // 50% speed

#define FLOW_CHANNELS 6
#define TEMPERATURE_CHANNELS 8
#define ADC_ROUGHING_PRESSURE_CH 2
#define ADC_TURBO_50_CH 1
