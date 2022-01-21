/* 
  Check for alarm conditions of Modbus instruments over ethernet.
*/

#include <config_j.h>

#include <ArduinoModbus.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <Ethernet.h>
#include <SPI.h>

// MAC address
byte mac[] = MAC;

// modbus instrument IPs
byte adc_flow_ip[] = ADC_FLOW_IP;
byte adc_pressure_ip[] = ADC_PRESSURE_IP;
byte adc_temperature_ip[] = ADC_TEMPERATURE_IP;

// modbus instrument id's
int adc_flow_id = ADC_FLOW_ID;
int adc_pressure_id = ADC_PRESSURE_ID;
int adc_temperature_id = ADC_TEMPERATURE_ID;

// alarm pins
int lo_flow_alarm_pin = LO_FLOW_ALARM_PIN;
int hi_roughing_pressure_alarm_pin = HI_ROUHGING_PRESSURE_ALARM_PIN;
int hi_temperature_alarm_pin = HI_TEMPERATURE_ALARM_PIN;
int turbo_50_pin = TURBO_50_PIN;
int comms_error_pin = COMMS_ERROR_ALARM_PIN;
int software_error_pin = SOFTWARE_ERROR_ALARM_PIN;

// adc alarm levels
int adc_flow_min = ADC_FLOW_MIN;
int adc_roughing_pressure_max = ADC_ROUGHING_PRESSURE_MAX;
int adc_turbo_50 = ADC_TURBO_50;
int adc_temperature_max = ADC_TEMPERATURE_MAX;

// alarm channel counts
int flow_channels = FLOW_CHANNELS;
int temperature_channels = TEMPERATURE_CHANNELS;

// bytes available to read after modbus request
int b;

// modbus read value
long r;

// local tcp modbus client
EthernetClient client;
ModbusTCPClient modbusTCPClient(client);

void setup() { 
  // keep checking for DHCP address until its available
  while (Ethernet.begin(mac) == 0) {
    delay(1);
  }

  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // init alarm pins
  pinMode(lo_flow_alarm_pin, OUTPUT);
  pinMode(hi_roughing_pressure_alarm_pin, OUTPUT);
  pinMode(turbo_50_pin, OUTPUT);
  pinMode(hi_temperature_alarm_pin, OUTPUT);
  pinMode(comms_error_pin, OUTPUT);
  pinMode(software_error_pin, OUTPUT);

  // assume no error states at start, PLC inputs are active LOW
  digitalWrite(lo_flow_alarm_pin, HIGH);
  digitalWrite(hi_roughing_pressure_alarm_pin, HIGH);
  digitalWrite(turbo_50_pin, HIGH);
  digitalWrite(hi_temperature_alarm_pin, HIGH);
}

void loop() {
  // maintain an IP lease from the DHCP server
  Ethernet.maintain();

  // check temperature adc for alarm conditions
  if (modbusTCPClient.begin(adc_temperature_ip, 502)) {
      Serial.println("Modbus TCP Client connected to thermocouple ADC!");

      // request to read and get number of values available
      modbusTCPClient.requestFrom(adc_temperature_id, INPUT_REGISTERS, 0x00, temperature_channels);
      b = modbusTCPClient.available();
      
      // read values and compare to alarm conditions
      if (b > 0) {
        // reset max temp exceeded channel count
        adc_temperature_max_channels = 0;

        // read all values and check if any exceed max temp
        for (int i = 0; i < b; i++) {
          r = modbusTCPClient.read();
          Serial.print("channel ")
          Serial.print(i)
          Serial.print(": ")
          Serial.println(r)

          // if value exceeds max temperature increment channel count
          if (r > adc_temperature_max) {
            adc_temperature_max_channels++;
          }
        }

        // if any channels exceed max temp raise alarm
        if (adc_temperature_max_channels > 0) {
          digitalWrite(hi_temperature_alarm_pin, LOW);
          Serial.print("Maximum temperature exceeded on ");
          Serial.print(adc_temperature_max_channels);
          Serial.println(" channels");
        } else {
          digitalWrite(hi_temperature_alarm_pin, HIGH);
        }
        
      } else {
        Serial.println("No values to read");
      }

    // close connection and move on
    modbusTCPClient.stop()

  } else {
    Serial.println("Modbus TCP Client failed to connect to temperature ADC!");
    digitalWrite(comms_error_pin, LOW);
  }
}






  
