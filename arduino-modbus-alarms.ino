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

// bytes available to read after modbus request
int b;

// modbus read value
byte r[] = {0x00, 0x00}

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

  // check flow adc for alarm conditions
  if (modbusTCPClient.begin(adc_flow_ip, 502)) {
      Serial.println("Modbus TCP Client connected to flow ADC!");

      modbusTCPClient.requestFrom(COILS, 0x0, 0x6);

      b = modbusTCPClient.available()
      if (b > 0) {
        for (int i = 0; i == b; i++) {
          r[i % 2] = modbusTCPClient.read();

          if (i % 2 == 1) {
            ;
          }
        }
      }

    // close connection and move on
    modbusTCPClient.stop()

  } else {
    Serial.println("Modbus TCP Client failed to connect to flow ADC!");
    digitalWrite(comms_error_pin, LOW);
  }
}






  
