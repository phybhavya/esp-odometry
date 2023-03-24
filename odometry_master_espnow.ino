/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include<stdlib.h>
#include<stdio.h>
#include<math.h>  
/*
initialising the odometry and the base
*/

#define radius 50
#define EncoderCounts 1200

double pi = M_PI;

int i;
int dir[3]={17, 26, 14}; //insert direction pins
int pwm[3]={5, 27, 12}; //insert pwm pins
bool directions[3];//={true, false, true}; //random - we'll change accc to wheel
float speedd=100.0;
int vel[3];
int wheel_angles[3]={0, 120, 240}; //try with {90, 330, 210} - assuming forward to be 0 degree (top, bottom_left, bottom_right)
bool dirs[3] = {false, false, false};
double Set[2] = {-1000,1000};
volatile double counter[2] = {0,0};                                  //variable to store counts of the encoder
double angle[2], prevAngle[2]={0,0};                                        //angle turned by encoder
double dist[2] = {0,0};                                        //distance travelled in the current iteration
double Coordinate[2] = {0,0};                                              //coordinates of the bot
int enc_pin[4] = {2,15,22,23};                              //18 19 for x axis, 20 21 for y axis
double error[2]= {0,0};
int enc_angle[2] = {0, 45};

int point=0, no_of_coord = 3;
int pathX[3] = {0, -1000, 0};
int pathY[3] = {2000, 2000, 0};
/*
Complete the initilization
*/
// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
typedef struct message{
  float x;
  float y;
  float vx;
  float vy;
  float error_x;
  float error_y;
  
}message;
message odometry;
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void odometry1(int o)
{
  angle[o] = counter[o]*360/EncoderCounts;                                                          //calculating angle
  dist[o] = ((angle[o]-prevAngle[o])*pi*radius/180)/cos(enc_angle[o] * pi / 180);                   //calculating distance moved
  prevAngle[o] = angle[o];
  Coordinate[o] = Coordinate[o] + dist[o];                                                          //current X and Y coordinates
  error[o] = Set[o] - Coordinate[o];

}

double vel_x, vel_y;

void holonomic()
{ 
  float ang = atan(error[1]/error[0]);
  double distance = sqrt(error[0]*error[0] + error[1]*error[1]);
  speedd = map(distance, -2000, 2000, -100, 100);
  vel_x = speedd*cos(ang);
  vel_y = speedd*sin(ang);
  
  for(i=0;i<3;i++)
  {
    vel[i]=vel_x*cos(wheel_angles[i] * M_PI/180) + vel_y*sin(wheel_angles[i] * M_PI/180);
    if(vel[i]<0)
    {
      vel[i]*=-1;
      directions[i]=!dirs[i];
    }
    else
      directions[i] = dirs[i];
  }

  for(i=0;i<3;i++)
  {
    digitalWrite(dir[i], directions[i]);
    analogWrite(pwm[i], vel[i]);
  }
}
// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}



// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    for(i=0;i<3;i++){
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);}

  for(i=0; i<4; i++)
    pinMode(enc_pin[i], INPUT_PULLUP);                                      //for encoders
  
  attachInterrupt(digitalPinToInterrupt(2), ai2, RISING);
  attachInterrupt(digitalPinToInterrupt(15), ai3, RISING);
  attachInterrupt(digitalPinToInterrupt(22), ai4, RISING);
  attachInterrupt(digitalPinToInterrupt(23), ai5, RISING);

  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
   odometry1(0);
  odometry1(1);
  Serial.println(counter[0]);
  Serial.println(counter[1]);
  holonomic();
  const uint8_t *peer_addr = slave.peer_addr;
  odometry.x = Coordinate[0];
  odometry.y = Coordinate[1];
  odometry.vx = vel_x;
  odometry.vy = vel_y;
  odometry.error_x = error[0];
  odometry.error_y = error[1];
   esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &odometry, sizeof(odometry));
    if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again

}
    void ai2() {
  if(digitalRead(15)==LOW) {
  counter[0]++;
  }else{
  counter[0]--;
  }
  }
   
  void ai3() {
  if(digitalRead(2)==LOW) {
  counter[0]--;
  }else{
  counter[0]++;
  }
  }

    void ai4() {
  if(digitalRead(23)==LOW) {
  counter[1]++;
  }else{
  counter[1]--;
  }
  }
   
  void ai5() {
  if(digitalRead(22)==LOW) {
  counter[1]--;
  }else{
  counter[1]++;}}
