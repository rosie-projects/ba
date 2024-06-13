#include <TinyGPS.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

ros::NodeHandle nh;
sensor_msgs::NavSatFix GPS;  // variavel
ros::Publisher pub_GPS("GPS", &GPS);

float lat,lon,alt;
TinyGPS gps; // create gps object

void setup(){
//Serial.begin(57600); // connect serial
//Serial.println("The GPS Received Signal:");
Serial3.begin(9600); // connect gps sensor

  nh.initNode();
  nh.advertise(pub_GPS);
}
 
void loop(){
    
    PublicarGPS();
    
    while(Serial3.available()){ // check for gps data
    if(gps.encode(Serial3.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
    alt = gps.f_altitude();

    Serial.print("Position: ");
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);
    
    Serial.print(",");
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.print(lon,6); 
    
    Serial.print(",");
    
    
    Serial.print(" Altitude: ");
    Serial.println(alt);
   }
  }
}

void PublicarGPS(){
      GPS.latitude = lat;
      GPS.longitude = lon;
      GPS.altitude = alt;
      pub_GPS.publish(&GPS);
      nh.spinOnce();
}
