/*
 * Description: Heartbeat Pulse Sensor monitoring with ESP32 and AskSensors IoT
 *  Author: https://asksensors.com, 2020
 *  github: https://github.com/asksensors
 */
 
#include <WiFi.h>
#include <PubSubClient.h>

//TODO: ESP32 MQTT user config
const char* ssid = ".................."; // Wifi SSID
const char* password = ".................."; // Wifi Password
const char* username = "................."; // my AskSensors username
const char* pubTopic = "publish/..../....."; // publish/username/apiKeyIn

const unsigned int writeInterval = 2000;   // write interval (in ms)
//AskSensors MQTT config
const char* mqtt_server = "mqtt.asksensors.com";
unsigned int mqtt_port = 1883;

// KY039 defines
#define TAB_LENGTH        4
#define RISE_THRESHOLD  5
#define CALIB_OFFSET  0 // change this offset after calibration

WiFiClient askClient;
PubSubClient client(askClient);

void setup() {
  Serial.begin(115200);
  Serial.println("*****************************************************");
  Serial.println("********** Program Start : ESP32 publishes KY039 Heartbeat data to AskSensors over MQTT");
  Serial.print("********** connecting to WIFI : ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("->WiFi connected");
  Serial.println("->IP address: ");
  Serial.println(WiFi.localIP());
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
}

void loop() {

  float analog_Tab[TAB_LENGTH], analog_sum;
  float last, analog_average, start;
  float KY039_data, first, second, third, before;
  bool rising;
  int rise_count, n_reads;
  long int last_beat, now, ptr;
  // WiFi connection 
  if (!client.connected()) 
    reconnect();
  client.loop();
  // Init variables
  for (int i = 0; i < TAB_LENGTH; i++)  analog_Tab[i] = 0;
  analog_sum = 0;
  ptr = 0;
   while(1) {
     // calculate an average of the sensor during a 20 ms period to eliminate the 50 Hz noise caused by electric light
     n_reads = 0;
     start = millis();
     analog_average = 0.;
     do {
       analog_average += analogRead(A0);
       n_reads++;
       now = millis();
     }
     while (now < start + 20);  
     analog_average /= n_reads;  // we got an average
     // Add the newest measurement to an array and subtract the oldest measurement from the array
     // to maintain a sum of last measurements
     analog_sum -= analog_Tab[ptr];
     analog_sum += analog_average;
     analog_Tab[ptr] = analog_average;
     last = analog_sum / TAB_LENGTH;
     // now last holds the average of the values in the array
     // check for a rising curve (= a heart beat)
     if (last > before) {
       rise_count++;
       if (!rising && rise_count > RISE_THRESHOLD) {
        // we have detected a rising curve, which implies a heartbeat.
        // Record the time since last beat, keep track of the two previous times (first, second, third) to get a weighed average.
        // The rising flag prevents us from detecting the same rise more than once.
        rising = true;
        first = millis() - last_beat;
        last_beat = millis();
        // Calculate the weighed average of heartbeat rate according to the three last beats
        KY039_data = 60000. / (0.4 * first + 0.3 * second + 0.3 * third)+ CALIB_OFFSET;
        Serial.print(KY039_data);
        Serial.println(" BPM\n"); // Unit
        third = second;
        second = first;
        Serial.println("********** Publish MQTT data to ASKSENSORS");
        char mqtt_payload[30] = "";
        snprintf (mqtt_payload, 30, "m1=%f", KY039_data);
        Serial.print("Publish message: ");
        Serial.println(mqtt_payload);
        client.publish(pubTopic, mqtt_payload);
        Serial.println("> MQTT data published");
        Serial.println("********** End ");
        Serial.println("*****************************************************");
        delay(writeInterval);// delay
       }
     }
     else
     {
       // Ok, the curve is falling
       rising = false;
       rise_count = 0;
     }
     before = last;
     ptr++;
     ptr %= TAB_LENGTH;
   }
}

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("********** Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", username, "")) {  
      Serial.println("-> MQTT client connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("-> try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
