#include <WiFi.h>

// Network Credentials
const char* ssid = "PhysicsBox-3";
const char* password = "12345678"; // Password must be over 7 characters

WiFiServer server(80); //Web Server port 80 for HTTP
String header;         // store HTTP request

String FileList, FileName;
int state = 0;
char C; // placeholder character to read from sd card/UNO

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Set Serial Monitor baud rate
  while(!Serial);

  WiFi.softAP(ssid, password); // Set access point
  IPAddress IP = WiFi.softAPIP(); // IP Address
//  Serial.print("IP address: ");
//  Serial.println(IP);
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  WiFiClient client = server.available(); // Listens for new clients
  if(client){ // New Client Connected
    while(client.connected()){
      if(client.available()){ // Read http request
        char c = client.read();
        header += c;
//        Serial.print(state);
        switch(state){
          case 0:
            if (c == '\n'){
              state = 1;
            }
            break;
          case 1:
            if (c == '\r'){
              state = 2; 
            }
            break;
          case 2:
            if (c == '\n'){
              state = 3; 
            }
            break;
          case 3: // Double line detected
            if(header.indexOf("GET / ") >= 0){ // Empty GET request, load Page
              Serial.print("L");
              // Response Code
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();
  
              // HTML Webpage
              client.println("<!DOCTYPE html><html><head>");
              client.println("<title>Page Title</title>");
              client.println("</head><body>");

//              Serial.print("reading char");
              while(C != '\n'){
                while (Serial.available() > 0) { // Wait to receive file from UNO
                  C = Serial.read();
//                  Serial.print(C);
                  client.print(C); // Relay it to client
                }
              }

              client.println("<ul>");
//              FileName = "test";
//              client.println("<li><a href=\"");
//              client.print(FileName);
//              client.print("\">");
//              client.print(FileName);
//              client.print("</a></li>");
//              
//              client.println("<li><a href=\"url\">link text</a></li>");
              
              client.println("</ul></body></html>");
              client.println(); // Final one to end response
            }
            else{// Assume file to be sent
//              Serial.print("F");
//              Serial.println("file GET");
              // Send Files
              FileName = header.substring(header.indexOf("GET") + 4, header.indexOf("HTTP") - 1); // Needs index adjustments
  //                Serial.print("Files");
    
              Serial.print("F" + FileName + "\n");
//                  Serial.print('\n'); // println instead?
              // Download Appropriate File
              
              // Response Code
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/csv");
              client.println("Connection: close");
              // client.println("Content-Disposition: attachment; filename=\"");
              // client.print(FileName);
              // client.print(".csv\"");
              client.println();
              while(C != '\0'){
//                Serial.print("receiving");
                while (Serial.available() > 0) { // Wait to receive file from UNO
                  C = Serial.read();
//                  Serial.write(C);
                  client.print(C); // Relay it to client
                }
              }
//              Serial.println("next");
//              while (Serial.available() > 0) { // Wait to receive file from UNO
//                client.print(Serial.read()); // Relay it to client
//              }
              client.println(); // Final one to end response
          }
          state = 0;
          C = ' ';
//          Serial.println(header);
          header = "";
//          Serial.println("client disconnected");
          flushSerial();
          client.stop();  // Close connection
          break;
        }
      }
    }
  }
}

void flushSerial(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}
