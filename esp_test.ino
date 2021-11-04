#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <string>
#include <stdexcept>
#include <map>
#include <functional>

ESP8266WebServer server;
char *s = "<head><meta http-equiv='refresh' content='1'></head><h1>Stacja pogodowa by Damian Wojcik & Gajda Wojciech</h1><br><h3>Temperatura:   20.00 &#8451;</h3><h3>Wilgotnosc:     50.0 %%</h3><h3>Cisnienie:    101300 Pa</h3>  ";

//default
double temp = 20.00;
double hum = 50.0;
double pres = 101300.0;

constexpr uint buffer_size {40};
std::string rx_buffer_;
uint bytes_received {0};

std::map<char , std::function<void()>> callbacks_;

void stronaGlowna()
{
   server.send(200, "text/html", s);
}

double extractNumberFromMessage()
{
  return std::stod(rx_buffer_.substr(2 , bytes_received - 1 ));// < + sensor char + > + '\0'
}

void parsePressure()
{
   pres = extractNumberFromMessage() / 4096;
}

void parseTemperature()
{
   temp = extractNumberFromMessage() / 10;
}

void parseHumidity()
{
  hum = extractNumberFromMessage() / 10;
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  rx_buffer_.resize(buffer_size);

  callbacks_['t'] = parseTemperature;
  callbacks_['p'] = parsePressure;
  callbacks_['h'] = parseHumidity;
  
  IPAddress localIp(192,168,0,1);
  IPAddress gateway(192,168,0,1);
  IPAddress subnet(255,255,255,0);
  
  WiFi.softAPConfig(localIp, gateway, subnet);
  WiFi.softAP("Stacja pogodowa", "12341234");
  server.on("/", stronaGlowna);
  server.begin();
}

void checkMessage(bool statement)
{
  if(!statement)
  {
    throw std::logic_error{"Bad message!"};
  }
}

void tryReadDataFromUart()
{
  bytes_received = Serial.available();

  checkMessage(bytes_received < buffer_size && bytes_received != 0);
  
  Serial.readBytes(rx_buffer_.data() , bytes_received);

  Serial.write( (rx_buffer_ + std::to_string(bytes_received)).data() );

  checkMessage(rx_buffer_[0] == '<' );
  checkMessage(rx_buffer_[bytes_received - 1] == '>' );
}

void loop() 
{
    server.handleClient();
    
    sprintf(s,"<head><meta http-equiv='refresh' content='1'>"
    "</head><h1>Stacja pogodowa by Damian Wojcik & Gajda Wojciech</h1>"
    "<br><h3>Temperatura:   %5.2lf &#8451;</h3><h3>Wilgotnosc:"     
    "%4.1lf %%</h3><h3>Cisnienie:    %6lf hPa</h3>" ,
    temp,hum,pres);

    try
    {
      delay(50);
      tryReadDataFromUart();

      callbacks_.at(rx_buffer_[1])();
    }
    catch(...)//unexpected error , do nothing
    {
      Serial.flush();//clear stream
      //and do nothing
    }
}
