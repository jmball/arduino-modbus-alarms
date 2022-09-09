/* 
  Check for alarm conditions of Modbus instruments over ethernet.
*/

#include "config_wolfson.h"

#include <ArduinoModbus.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <Ethernet.h>
#include <SPI.h>
#include <TimeLib.h>

char system_name = SYSTEM_NAME;

// MAC address
byte mac[] = MAC;

// instrument IPs and ports
byte adc_flow_ip[] = ADC_FLOW_IP;
byte adc_pressure_ip[] = ADC_PRESSURE_IP;
byte adc_temperature_ips[] = ADC_TEMPERATURE_IPS;
byte adc_temperature_ip[4];
int num_temperature_adcs = sizeof(adc_temperature_ips) / sizeof(adc_temperature_ips[0]);
int adc_flow_port = ADC_FLOW_PORT;
int adc_pressure_port = ADC_PRESSURE_PORT;
int adc_temperature_port = ADC_TEMPERATURE_PORT;

// instrument id's
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
float adc_temperature_max = ADC_TEMPERATURE_MAX;

// flow and thermocouple channel counts
int flow_channels = FLOW_CHANNELS;
int temperature_channels = TEMPERATURE_CHANNELS;
int adc_temperature_max_channel_count = 0;
int adc_flow_min_channel_count = 0;

// channels to read on pressure ADC
int adc_roughing_pressure_ch = ADC_ROUGHING_PRESSURE_CH;
int adc_turbo_50_ch = ADC_TURBO_50_CH;

// bytes available to read after modbus multiple channel request
int b;

// modbus read values
long r;
long turbo_speed;
long roughing_pressure;
float temperature;

// comms error loop count, if 3 loops report comms errors raise alarm
int comms_loop_errors = 0;
int max_comms_loop_errors = 3;
int comms_this_loop_errors;

// software IP and port
byte software_status_ip[] = SOFTWARE_STATUS_IP;
int software_status_port = SOFTWARE_STATUS_PORT;
int connect_status;

// local tcp modbus client
EthernetClient client;
ModbusTCPClient modbusTCPClient(client);

void setup() {
  // init alarm pins
  pinMode(lo_flow_alarm_pin, OUTPUT);
  pinMode(hi_roughing_pressure_alarm_pin, OUTPUT);
  pinMode(turbo_50_pin, OUTPUT);
  pinMode(hi_temperature_alarm_pin, OUTPUT);
  pinMode(comms_error_pin, OUTPUT);
  pinMode(software_error_pin, OUTPUT);

  // assume error state on start, PLC inputs are active LOW
  digitalWrite(lo_flow_alarm_pin, LOW);
  digitalWrite(hi_roughing_pressure_alarm_pin, LOW);
  digitalWrite(turbo_50_pin, LOW);
  digitalWrite(hi_temperature_alarm_pin, HIGH);
  digitalWrite(comms_error_pin, LOW);
  digitalWrite(software_error_pin, LOW);
  
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Serial connected!");

  // keep checking for DHCP address until its available
  while (Ethernet.begin(mac) == 0) {
    delay(1);
  }

  Serial.print("Ethernet connected on IP address: ");
  Serial.println(Ethernet.localIP());  
}

void loop() {
  // maintain an IP lease from the DHCP server
  Ethernet.maintain();

  // reset comms error count for new loop
  comms_this_loop_errors = 0;

  // check for temeprature alarm conditions
  if (system_name == "WOLFSON") {
    temperature_check_wolfson();
  }
  else if (system_name == "G44") {
    temperature_check_g44();
  }
  else {
    digitalWrite(hi_temperature_alarm_pin, LOW);
    Serial.println("Invalid system name. Check config header file.");
    Serial.println("Cannot retrieve temperature information so assuming max temp exceeded to be safe.");
  }

  // check flow adc for alarm conditions
  if (modbusTCPClient.begin(adc_flow_ip, adc_flow_port)) {
    Serial.println("Modbus TCP Client connected to flow ADC!");

    // request to read and get number of values available
    modbusTCPClient.requestFrom(adc_flow_id, INPUT_REGISTERS, 0x00, flow_channels);
    b = modbusTCPClient.available();
    
    // read values and compare to alarm conditions
    if (b > 0) {
      // reset min flow channel count
      adc_flow_min_channel_count = 0;

      // read all values and check if any exceed max temp
      for (int i = 0; i < b; i++) {
        r = modbusTCPClient.read();
        Serial.print("channel ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(r / 10000.0, 4);

        // if value below min flow increment channel count
        if (r < adc_flow_min) {
        adc_flow_min_channel_count++;
        }
      }

      // if any channels are below min flow raise alarm
      if (adc_flow_min_channel_count > 0) {
        digitalWrite(lo_flow_alarm_pin, LOW);
        Serial.print("Flow below minimum on ");
        Serial.print(adc_flow_min_channel_count);
        Serial.println(" channels");
      } else {
        digitalWrite(lo_flow_alarm_pin, HIGH);
      }
    } else {
    Serial.println("No values to read");
    }

    // close connection and move on
    modbusTCPClient.stop();
  } else {
    Serial.println("Modbus TCP Client failed to connect to flow ADC!");
    comms_this_loop_errors++;
  }

  // check pressure adc for alarm conditions
  if (modbusTCPClient.begin(adc_pressure_ip, adc_pressure_port)) {
    Serial.println("Modbus TCP Client connected to pressure ADC!");

    // read turbo speed
    modbusTCPClient.requestFrom(adc_pressure_id, INPUT_REGISTERS, adc_turbo_50_ch, 1);
    b = modbusTCPClient.available();

    if (b > 0) {
      turbo_speed = modbusTCPClient.read();
      Serial.print("Turbo speed: ");
      Serial.println(turbo_speed / 100.0, 2);

      // check threshold condition
      if (turbo_speed > adc_turbo_50) {
        digitalWrite(turbo_50_pin, LOW);
      } else {
        digitalWrite(turbo_50_pin, HIGH);  
      }
    } else {
    Serial.println("No turbo speed value to read");
    }

    // read roughing pressure
    modbusTCPClient.requestFrom(adc_pressure_id, INPUT_REGISTERS, adc_roughing_pressure_ch, 1);
    b = modbusTCPClient.available();
    
    if (b > 0) {
      roughing_pressure = modbusTCPClient.read();
      Serial.print("Roughing pressure voltage: ");
      Serial.println(roughing_pressure / 1000.0, 2);

      // check threshold condition
      if (roughing_pressure > adc_roughing_pressure_max) {
        digitalWrite(hi_roughing_pressure_alarm_pin, LOW);
      } else {
        digitalWrite(hi_roughing_pressure_alarm_pin, HIGH);  
      }
    } else {
    Serial.println("No roughing pressure value to read");
    }

    // close connection and move on
    modbusTCPClient.stop();
  } else {
    Serial.println("Modbus TCP Client failed to connect to pressure ADC!");
    comms_this_loop_errors++;
  }

  // check if software is running
  connect_status = client.connect(software_status_ip, software_status_port);
  Serial.print("Software status code: ");
  Serial.println(connect_status);
  if (connect_status) {
    // the connection was successful so software must be running
    digitalWrite(software_error_pin, HIGH);
    Serial.println("TCP Client connected to software!");

    // connection isn't needed anymore so close it
    client.stop();
  } else {
    // the connection was unsuccessful so software probably isn't running
    digitalWrite(software_error_pin, LOW);
    Serial.println("TCP Client failed to connect to software!");
  }

  if (comms_this_loop_errors > 0) {
    // had comms errors during this loop so increment counter
    comms_loop_errors++;
  } else {
    // no comms errors this loop so reset counter
    comms_loop_errors = 0;
  }

  // report comms error
  if (comms_loop_errors > max_comms_loop_errors) {
    digitalWrite(comms_error_pin, LOW);
    Serial.println("Comms Error!");
  } else {
    digitalWrite(comms_error_pin, HIGH);
  }

  delay(5000);
}


void temperature_check_wolfson() {
  // reset max temp exceeded channel count
  adc_temperature_max_channel_count = 0;
  
  // loop over all temperature adcs
  for (int i = 0; i < num_temperature_adcs; i++) {
    
    // get temperature adc IP address
    for (int j = 0; j < 4; j++) {
      adc_temperature_ip[j] = adc_temperature_ips[i][j];
    }
       
    // check temperature adc for alarm conditions
    if (modbusTCPClient.begin(adc_temperature_ip, adc_temperature_port)) {
      Serial.println("Modbus TCP Client connected to thermocouple ADC!");
  
      // request to read and get number of values available
      modbusTCPClient.requestFrom(adc_temperature_id, INPUT_REGISTERS, 0x00, temperature_channels);
      b = modbusTCPClient.available();
      
      // read values and compare to alarm conditions
      if (b > 0) {
        // read all values and check if any exceed max temp
        for (int i = 0; i < b; i++) {
          r = modbusTCPClient.read();
          temperature = r / 10.0;
          Serial.print("channel ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(temperature, 2);
  
          // if value exceeds max temperature increment channel count
          if (temperature > adc_temperature_max) {
            adc_temperature_max_channel_count++;
          }
        }
      } else {
        // no data to read from instrument so must be comms error
        Serial.println("Modbus TCP Client failed to read from thermocouple ADC!");
        comms_this_loop_errors++;
      }
  
      // close connection and move on
      modbusTCPClient.stop();
    } else {
      Serial.println("Modbus TCP Client failed to connect to thermocouple ADC!");
      comms_this_loop_errors++;
    }
  }

  // if any channels exceed max temp raise alarm
  if (adc_temperature_max_channel_count > 0) {
    digitalWrite(hi_temperature_alarm_pin, LOW);
    Serial.print("Maximum temperature exceeded on ");
    Serial.print(adc_temperature_max_channel_count);
    Serial.println(" channels");
  } else {
    digitalWrite(hi_temperature_alarm_pin, HIGH);
  }
}

void temperature_check_g44() {
  // reset max temp exceeded channel count
  adc_temperature_max_channel_count = 0;

  // comms setup for TCP temperature adc's
  char TERMCHAR = 0x15;
  char m;
  int buf_arr_len = 8;
  char buf_arr[buf_arr_len];
  int timeout = 15;
  
  // loop over all temperature adcs
  for (int i = 0; i < num_temperature_adcs; i++) {
    
    // get temperature adc IP address
    for (int j = 0; j < 4; j++) {
      adc_temperature_ip[j] = adc_temperature_ips[i][j];
    }

    // connect to temperature adc
    if (client.connect(adc_temperature_ip, adc_temperature_port)) {
      
      // request current temperature
      client.write("*G110");
      client.write(TERMCHAR);

      // read response bytes into the buffer
      int i = 0;
      time_t t0 = now();
      int read_err = 0;
      while (true) {
        // check error conditions on read
        if ((now() - t0) > timeout) {
          read_err = 1;
          break;
        }
        else if (i > buf_arr_len - 1) {
          read_err = 1;
          break;
        }
    
        // read a byte into the buffer
        m = client.read();
        if (m == TERMCHAR){
          // end of message reached
          break;  
        }
        buf_arr[i] = m;
        i++;
      }

      if (read_err == 0) {
        temperature = atof(buf_arr);

        // if value exceeds max temperature increment channel count
        if (temperature > adc_temperature_max) {
          adc_temperature_max_channel_count++;
        }
      } else {
        comms_this_loop_errors++;
      }
      
      // reset read buffer
      for (int i = 0; i < buf_arr_len; i++) {
        buf_arr[i] = '\0';
      }

      client.stop();
    } else {
      Serial.println("Failed to connect to thermocouple ADC!");
      comms_this_loop_errors++;
    }
  
  // if any channels exceed max temp raise alarm
  if (adc_temperature_max_channel_count > 0) {
    digitalWrite(hi_temperature_alarm_pin, LOW);
    Serial.print("Maximum temperature exceeded on ");
    Serial.print(adc_temperature_max_channel_count);
    Serial.println(" channels");
  } else {
    digitalWrite(hi_temperature_alarm_pin, HIGH);
  }
}





  
