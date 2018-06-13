#include <SoftwareSerial.h>
SoftwareSerial mySerial(D1, D2); // RX, TX
#include <ESP8266WiFi.h>
#include <Stream.h>
const char* ssid     = "FaryLink_2954A4";      // SSID
const char* password = "";      // Password
const char* host = "192.168.4.1";  // IP serveur - Server IP
const int   port = 80;            // Port serveur - Server Port
const int   watchdog = 5000;        // Fréquence du watchdog - Watchdog frequency
unsigned long previousMillis = millis(); 
#define DEBUG true
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  mySerial.print("Connecting to ");
  mySerial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    mySerial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  mySerial.println("");
  mySerial.println("WiFi connected");  
  mySerial.println("IP address: ");
  mySerial.println(WiFi.localIP());
  
  //sendData("AT+RST\r\n",2000,DEBUG); // reset module
  //sendData("AT+CWMODE=3\r\n",1000,DEBUG); // configure as access point

  //sendData("AT+CIFSR\r\n",1000,DEBUG); // get ip address
  //sendData("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
  //sendData("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80
  
}

void loop() {
  
  unsigned long currentMillis = millis();

  if ( currentMillis - previousMillis > watchdog ) {
    previousMillis = currentMillis;
    WiFiClient client;
    client.setTimeout(10000);
  
    if (!client.connect(host, port)) {
      Serial.println("connection failed");
      mySerial.println("connection failed");
      delay(500);
      return;
    }

    String url = "192.168.4.1";
    url += String(millis());
    url += "&ip=";
    url += WiFi.localIP().toString();
    
    // Envoi la requete au serveur - This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > watchdog) {
        Serial.println(">>> Client Timeout !");
        mySerial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }
  
    // Read all the lines of the reply from server and print them to Serial
    while(client.available()){
      String line = client.readString();
      Serial.println(line);
      mySerial.println(line);
    }
  }
}

/*
String sendData(String command, const int timeout, boolean debug)
            {
                String response = "";
                Serial1.print(command);
                long int time = millis();
                while( (time+timeout) > millis())
                {
                   while(Serial1.available())
                      {
                         char c = Serial1.read(); // read the next character.
                         response+=c;
                      }  
                }
                
                if(debug)
                     {
                     Serial.print(response); //displays the esp response messages in arduino Serial monitor
                     }
                return response;
            }
*/
