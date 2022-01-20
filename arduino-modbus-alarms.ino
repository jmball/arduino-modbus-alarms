/* 
  Check for alarm conditions of Modbus instruments over ethernet.
*/

#include <ArduinoModbus.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <Ethernet.h>
#include <SPI.h>

// MAC address
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x89, 0x87};

// modbus instrument IPs
byte adc_flow_ip[] = { 10, 0, 0, 177 };
byte adc_pressure_ip[] = { 10, 0, 0, 177 };
byte adc_temperature_ip[] = { 10, 0, 0, 177 };

// modbus instrument id's
int adc_flow_id = 1;
int adc_pressure_id = 1;
int adc_temperature_id = 1;

// alarm pins
int low_flow_pin = 1;
int high_roughing_pressure_pin = 2;
int turbo_50_pin = 3;
int high_temp_pin = 4;
int comms_error_pin = 5;
int software_error_pin = 6;

// adc alarm levels
int flow_min = 2048; // 0.15 lpm
int roughing_pressure_max = 2048; // 1 mbar
int tubro_50 = 2048; // 50%
int temp_max = 2048; // 600C

// alarm channel counts
int low_flow_channels = 0;
int temp_max_channels = 0;

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

  // init alarm pins, PLC inputs are active LOW
  pinMode(low_flow_pin, OUTPUT);
  pinMode(high_roughing_pressure_pin, OUTPUT);
  pinMode(turbo_50_pin, OUTPUT);
  pinMode(high_temp_pin, OUTPUT);
  pinMode(comms_error_pin, OUTPUT);

  // assume no error states at start
  digitalWrite(low_flow_pin, HIGH);
  digitalWrite(high_roughing_pressure_pin, HIGH);
  digitalWrite(turbo_50_pin, HIGH);
  digitalWrite(high_temp_pin, HIGH);
}

void loop() {
  // maintain an IP lease from the DHCP server
  Ethernet.maintain();

  // check temperature adc for alarm conditions
  if (modbusTCPClient.begin(adc_temperature_ip, 502)) {
      Serial.println("Modbus TCP Client connected to thermocouple ADC!");

      // request to read and get number of values available
      modbusTCPClient.requestFrom(adc_temperature_id, INPUT_REGISTERS, 0x00, 0x06);
      b = modbusTCPClient.available();
      
      // read values and compare to alarm conditions
      if (b > 0) {
        // reset max temp exceeded channel count
        temp_max_channels = 0;

        // read all values and check if any exceed max temp
        for (int i = 0; i < b; i++) {
          r = modbusTCPClient.read();

          // if value exceeds max temperature increment channel count
          if (r > temp_max) {
            temp_max_channels++;
          }
        }

        // if any channels exceed max temp raise alarm
        if (temp_max_channels > 0) {
          digitalWrite(high_temp_pin, LOW);
          Serial.print("Maximum temperature exceeded on ");
          Serial.print(temp_max_channels);
          Serial.println(" channels");
        } else {
          digitalWrite(high_temp_pin, HIGH);
        }
        
      } else {
        Serial.println("No values to read");
      }

    // close connection and move on
    modbusTCPClient.stop()

  } else {
    Serial.println("Modbus TCP Client failed to connect to flow ADC!");
    digitalWrite(comms_error_pin, LOW);
  }
}






  
