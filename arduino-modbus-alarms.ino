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

// local tcp modbus client
EthernetClient client;
ModbusTCPClient modbusTCPClient(client);

void setup() { 
  // keep checking for DHCP address until its available
  while (Ethernet.begin(mac) == 0) {
    delay(1);
  }

  // init alarm pins, PLC inputs are active LOW
  pinMode(low_flow_pin, OUTPUT);
  pinMode(high_roughing_pressure_pin, OUTPUT);
  pinMode(turbo_50_pin, OUTPUT);
  pinMode(high_temp_pin, OUTPUT);
  pinMode(comms_error_pin, OUTPUT);

  // assume no error states at restart
  digitalWrite(low_flow_pin, HIGH);
  digitalWrite(high_roughing_pressure_pin, HIGH);
  digitalWrite(turbo_50_pin, HIGH);
  digitalWrite(high_temp_pin, HIGH);
  digitalWrite(comms_error_pin, HIGH);
}

void loop() {
  // maintain an IP lease from the DHCP server
  Ethernet.maintain();

}
