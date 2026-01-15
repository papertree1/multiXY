# Descripció del projecte
- Un dispositiu MIDI que detecta un objecte en un pla bidimensional i n'extreu la posició per a convertir-la en missatges MIDI.  
- De funcionament similar a un pad XY, però amb la possibilitat d'enviar missatges MIDI sense estar tocant el dispositiu i de tenir més d'un objecte, per tant, més d'una senyal.  

# Objectius
-     Detecció precisa de la posició d'un objecte en un pla.
-     Quan s'aconsegueixi, detecció precisa i diferenciada de més d'un objecte en un pla.
-     Resolució de <1cm.
-     Latència de <150ms. (?)
-     Comunicació a temps real amb un patch de Max/MSP.

# Referències i material
## Radiodrum
- https://en.wikipedia.org/wiki/Radiodrum
- https://www.ece.uvic.ca/~peterd/radiodrum.html
- https://quod.lib.umich.edu/i/icmc/bbp2372.2006.078
## VL530X
- https://www.l33t.uk/ebay-tutorials/getting-started-with-the-vl53l0x/
- https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
  
# Memòria

## 19/09/25
- Plantejament del projecte
- Primeres idees: sensors d'efecte Hall, magnetòmetres, detecció de radio-freqüències
- La detecció de radio-freqüències, sobretot dispositius com el Radio Drum, semblen els més fiables i precisos.
## 26/09/25
- Recerca del Radio Drum i del seu funcionament
- Recerca d'altres mètodes per a detectar la posició d'un objecte en un pla bidimensional
## 03/10/25
- 🇵🇸
## 10/10/25
- Proves amb sensors d'ultrasons HC-SR204 i Arduino
- El problema amb sensors d'aquests és que tenen un feix molt prim (de 15º d'obertura). Per tant, per tenir una superfície de, diguem, 15cm, necessitaríem posar-los a uns 80'5 cm.
 ![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/CamScanner_11-13-2025_17.30_1.jpg]]
 ```java
 void loop() {
    // Measure distance for the first sensor
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    
    long duration1 = pulseIn(echoPin1, HIGH);
    int distance1 = duration1 / 58;
    
    // Measure distance for the second sensor
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    
    long duration2 = pulseIn(echoPin2, HIGH);
    int distance2 = duration2 / 58;
    x = ((distance1*distance1)-(distance2*distance2)+(COSTAT*COSTAT))/(2*COSTAT);
    y = sqrt((distance1*distance1)-(x*x));
    
    // Print the distances to the serial monitor
    Serial.print("Distance 1: ");
    Serial.print(distance1);
    Serial.print(" cm, Distance 2: ");
    Serial.print(distance2);
    Serial.print(" cm.........");
    Serial.print("Position (x,y): ");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
    
    // Wait for a short time before measuring again
    delay(100);
}
```
## 17/10/25
- Proves amb el sensor VL53L0X. La idea és moure dos sensors amb servos i calcular les diferències de temps entre la detecció de l'objecte en els diferents sensors. 
    - D'aquesta manera, podríem comparar els càlculs de posició amb les dues mesures i tenir un resultat més fiable.
    - També ens permetria tenir més d'un objecte sense tenir zones mortes, en què un objecte n'eclipsa un altre.
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/CamScanner_11-13-2025_17.30_2.jpg]]
- Setup bàsic i proves amb dos sensors i una ESP32, sense servos, per a entendre el funcionament dels sensors i fer-me una idea de la seva sensibilitat i fiabilitat.
    - Els sensors VL53L0X es comuniquen per I2C amb la ESP32, el que ens permet tenir-ne més d'un connectat als mateixos pins de la ESP32:
```java
    digitalWrite(pin_sensor_A, HIGH);
    if (!sensor_A.init()) {
        Serial.println(F("Failed to boot 1rst VL53L0X"));
        while(1);
    } else {
        sensor_A.setAddress(0x30); //Els dos sensors fan servir diferents adreces d'I2C
        sensor_A.setSignalRateLimit(0.1);
        sensor_A.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor_A.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        sensor_A.setMeasurementTimingBudget(40000);
        sensor_A.startContinuous(45);
    }
    delay(50);
    
    digitalWrite(pin_sensor_B, HIGH);
    delay(50);
    
    if (!sensor_B.init()) {
        Serial.println(F("Failed to boot 2nd VL53L0X"));
        while(1);
    } else {
        sensor_B.setAddress(0x31); //Els dos sensors fan servir diferents adreces d'I2C
        sensor_B.setSignalRateLimit(0.1);
        sensor_B.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor_B.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        sensor_B.setMeasurementTimingBudget(40000);
        sensor_B.startContinuous(45);
    }
```


- Aquesta prova bàsica m'ha ajudat a veure com treballar amb dos sensors, i veig viable fer la lectura "simultània" dels dos sensors per a augmentar la precisió de les lectures.
## 24/10/2025
- Més proves amb el mateix setup
- Elaboració d'un programa de Processing que dibuixi la posició que llegeixen els sensors, que es comunica a través del Serial

```java
import processing.serial.*;

Serial myPort;
float x, y;

void setup(){
  size(220, 220);
  String portName = Serial.list()[2];
  myPort = new Serial(this, portName, 115200);
}

void draw(){
if ( myPort.available() > 0) 
  {  // If data is available,
    background(0,0,0);
    String val = myPort.readStringUntil('\n');
    x = float(val.substring(0, val.indexOf(',')));
    y = float(val.substring(val.indexOf(',')+2,val.length()-1));
    println(x, y);
    fill(255,0,0);
    circle(x, y, 50);
    
  } 
}

```

![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/VID_20251023_191106364.mp4]]
- Els sensors, amb aquest setup, tenen molt de soroll.
- Fem el càlcul de la posició amb trigonometria bàsica:
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/CamScanner_11-13-2025_17.30_3.jpg]]
- Els resultats no son gaire fiables, ja que sumem el soroll dels dos sensors (els multipliquem, de fet)
## 31/10/2025
- Primeres proves amb servos.
- Reducció del setup a només un servo i un sensor, per simplicitat.
- Amb una bona lectura d'un sensor podem obtenir la posició d'un objecte.
- ![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/VID_20251106_190028353.mp4]]
- Fent servir la llibreria `ESP32Servo` i `VL53L0X`.

```java
void loop() {
    for(int i=0; i<ROTATION; i++){
        // Set servo angle to rotate in opposite directions
        i<ROTATION/2 ? theta1 = i : theta1 = ROTATION-i;
        //i<ROTATION/2 ? theta2 = ROTATION-i : theta2 = i;
        
        theta1 += 50; // Offset per la posició inicial
        
        servo_A.write(theta1);
        // READ POSITION
        if((millis()- startTime1) > mInterval)
        {
            reading_B = sensor_B.readRangeContinuousMillimeters();
            // TEST WITH ONE SERVO AND ONE SENSOR
            // TODO: FER DOS READINGS AMB DUES X, Y
            // I AFEGIR A UN ARRAY I CROSS-REFERENCE
            if(reading_B <= LIMIT_95){
                xReading = abs(sin(theta1 * PI/180) * reading_B);
                yReading = abs(cos(theta1 * PI/180) * reading_B);
            } else {
                x = -1;
                y = -1;
            }
        
            Serial.print(x);
            Serial.print(", ");
            Serial.println(y);
            
            startTime1 = millis();
        }
    }
    delay(200);
}
```

- Les mesures funcionen, però el soroll + imprecisions al càlcul (assumint que el feix del sensor és linear, quan realment és de 25º) fan que els càlculs encara no siguin fiables
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/VID_20251106_190529295.mp4]]
## 07/11/2025
- Més proves amb el mateix setup
- Intent d'smoothing de les mesures fent la mitjana entre la lectura anterior i l'actual.

```java
void loop() {

    for(int i=0; i<ROTATION; i++){
        // Set servo angle to rotate in opposite directions
        i<ROTATION/2 ? theta1 = i : theta1 = ROTATION-i;
        i<ROTATION/2 ? theta2 = ROTATION-i : theta2 = i;
        
        theta1 -= 50;
        theta2 -= 50;
        servo_A.write(theta1);
        //servo_B.write(theta2);
        
        // READ POSITION
        if((millis()- startTime1) > mInterval)
        {
            reading_B = sensor_B.readRangeContinuousMillimeters();
            if(reading_B <= LIMIT_95){
                xReading = abs(sin(theta1 * PI/180) * reading_B);
                yReading = abs(cos(theta1 * PI/180) * reading_B);
            } else {
                x = -1;
                y = -1;
            }
            
            xNow = xReading;
            yNow = yReading;
            
            x = (xReading+xAnt)/2;
            y = (yReading+yAnt)/2;
            
            xAnt = x;
            yAnt = y;
            
            Serial.print(x);
            Serial.print(", ");
            Serial.println(y);
            
            startTime1 = millis();
        }
    }
    delay(200);
}
```

- Desastre
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/IMG_20251113_171246846.jpg]]
## 14/11/2025
- Seguim provant amb els servos, tenint en compte dos aprenentatges: 
    - No els hi agraden els valors negatius
    - No els hi agraden els valors per sobre de 360
## 21/11/2025
- Gravació.
## 28/11/2025
- Bolo.
## 05/12/2025
- Proves amb el sensor `HMC5883L` per detecció de camp magnètic:
```java
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>


HMC5883L_Simple Compas;

void setup()
{

    Serial.begin(9600);
    Wire.begin();
    
    Compas.SetDeclination(1, 59, 'E');
    Compas.SetScale(COMPASS_SCALE_810);
    
    Serial.println("Hi");
    
    Compas.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
    
    Serial.println("Bye");
}

void loop(){
    float heading = Compas.GetHeadingDegrees();
    Serial.print("Heading: \t");
    Serial.println( heading );
    delay(1000);
}
```
- No he aconseguit que funcionés, no sembla detectar variacions en el camp magnètic, ja sigui rotant el sensor en qualsevol dimensió o aproximant un imant.
- Proves amb el sensor d'efecte Hall `HW-477`
```java
const int sensorPin = A5; // Pin where the OUT pin is connected
float sensorValue = 0; // Variable to store sensor value

void setup() 
    Serial.begin(9600); // Start serial communication
    pinMode(sensorPin, INPUT); // Set sensor pin as input
}

void loop() {
    sensorValue = analogRead(sensorPin); // Read the sensor value
    Serial.println(sensorValue); // Print the sensor value to the Serial Monitor
    delay(50); // Wait for half a second before the next read
}
```
- Les lectures són precises i fiables, i el codi senzill, però el rang és molt curt (<2cm)
## 12/12/2025
- Recuperació del model amb un servo i un VL530X.
- Anàlisi de les lectures amb el codi actual:
- Lectures d'un sol cicle (moviment de 0º -> 90º -> 0º):
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/dist_vs_angle.png]]

- Com es pot observar, els valors de l'anada i de la tornada son completament diferents. El que és pitjor, l'objecte, en aquesta prova, estava a uns 45º.
- No podem extreure cap patró d'aquesta conducta, excepte que caldrà trobar la manera d'incloure el feix de 25º del sensor dins dels càlculs, d'alguna manera.
- Comparant-lo amb un gràfic de fa unes setmanes (d'una prova diferent, però amb comparació d'anada vs tornada):
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/dist_vs_angle_1.png]]
## 19/12/2025
- Més mesures i més anàlisis de les mesures
- Mesura de la lectura del sensor amb cap objecte davant:
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/blank.png]]
## Vacances de Nadal
- Replantejament de la metodologia: en comptes d'algún sensor, es farà la detecció amb una càmera i anàlisi d'imatge amb Processing
- Per la càmera, utilitzo una [ESP32-CAM](https://www.tiendatec.es/electronica/placas-de-desarrollo/2072-esp32-cam-placa-esp32-con-camara-ov2640-wifi-bt-8472496025850.html), un mòdul ESP32 amb un sensor de càmera
- La idea és capturar una imatge, enviar-la a Processing d'alguna manera, analitzar-la per trobar on hi ha un objecte, i enviar un missatge MIDI CC.
- El setup és el següent
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/Pasted_image_20260114203024.png]]
- L'objecte té un imant a la base, i a sota de la superfície hi ha un altre imant enganxat a un paper de color
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/VID_20260114_203153947.mp4]]
- La construcció de la caixa és molt rudimentària, però el resultat final compleix la funció:
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/IMG_20260114_105852.jpg]]
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/IMG_20260115_100738583.jpg]]
- La programació de l'ESP32-CAM necessita la [llibreria d'expressif](https://github.com/espressif/esp32-camera/tree/master), mal documentada i confosa
- Per fer la foto, adapto el codi d'[aquesta guia](https://randomnerdtutorials.com/esp32-cam-take-photo-save-microsd-card/), però no la guardo a una targeta SD.
- Per la comunicació amb Processing, enviaré la foto pel port Serial.
- Provo diferents mètodes (enviar pixel per pixel, comprimir la informació d'alguna manera...), i diferents modes de la càmera (JPEG, RGB565), però cap metodologia serveix, ja que la manera en què la llibreria emmagatzema una foto està poc documentada.
- Finalment, la càmera fa la foto i l'emmagatzema en format JPEG a un buffer intern, la codifico en base64 i l'envio pel Serial.

```cpp
#include "esp_camera.h"
#include "Arduino.h"

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <base64.h>

// Pin definition for CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200); //higher rates eat characters
  
  // CAMERA CONFIG
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  /* <!-- Sensor settings throw an error. The sensor can't handle them? --!> */
  sensor_t * s = esp_camera_sensor_get();
  
/*
    s->set_brightness(s, 1); // -2 to 2
    s->set_contrast(s, 2); // -2 to 2
    s->set_saturation(s, 1); // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1); // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1); // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0); // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1); // 0 = disable , 1 = enable
    s->set_aec2(s, 0); // 0 = disable , 1 = enable
    s->set_ae_level(s, 0); // -2 to 2
    s->set_aec_value(s, 300); // 0 to 1200
    s->set_gain_ctrl(s, 1); // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0); // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
    s->set_bpc(s, 0); // 0 = disable , 1 = enable
    s->set_wpc(s, 1); // 0 = disable , 1 = enable
    s->set_raw_gma(s, 0); // 0 = disable , 1 = enable
    s->set_lenc(s, 0); // 0 = disable , 1 = enable
    s->set_hmirror(s, 0); // 0 = disable , 1 = enable
    s->set_vflip(s, 1); // 0 = disable , 1 = enable
    s->set_dcw(s, 0); // 0 = disable , 1 = enable
    s->set_colorbar(s, 0); // 0 = disable , 1 = enable
*/
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA; // 320x240 
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  pinMode(4, OUTPUT);
}

void loop() {
  takePicture(); //~500ms per frame
}

void takePicture(){
  camera_fb_t* fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Turns on the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  digitalWrite(4, 20);
  
  String encoded = base64::encode((uint8_t*)fb->buf, fb->len);
  //Serial.write(encoded.c_str(), encoded.length()); //Alternative
  Serial.println(encoded);

  esp_camera_fb_return(fb);

  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  digitalWrite(4, LOW); 
}
```

- A Processing, llegeixo del Serial i analitzo pixel a pixel si predomina un color principal (rgb). En cas que sí, n'extrec la posició.
- Un cop s'han analitzat tots els píxels, es fa una mitjana de la posició per cada color i s'envia un missatge MIDI CC. 
- Els missatges s'envien per canals aparellats, enviant la posició X pel primer i la Y pel segon. D'aquesta manera, els canals queden així:

| Canal | Informació |
| ----- | ---------- |
| 20    | Red X      |
| 21    | Red Y      |
| 22    | Green X    |
| 23    | Green Y    |
| 24    | Blue X     |
| 25    | Blue Y     |

```java
import themidibus.*;

import processing.serial.*;
import java.awt.*;
import java.awt.Image;
import java.awt.Toolkit;
import java.util.Base64;
import java.io.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;



processing.serial.Serial myPort;
MidiBus myBus;

PImage feed = null;

String val = "";

int chan = 0;
final int THRESH = 30;

void setup() 
  {
  size(320, 240);
  String portName = "COM3";
  myPort = new processing.serial.Serial(this, portName, 115200);
  MidiBus.list();
  myBus = new MidiBus(this, -1, 5);
  
  //noLoop();
  }

void draw(){
  //background(0);
  if(myPort.available() > 0){
  
    val = myPort.readStringUntil('\n');
    if(val != null && val.startsWith("/")){
      //println(val);
      val = val.trim(); //Remove \r at the end of String
      
      // Remove the "data:image/jpeg;base64," prefix (if exists)
      if (val.startsWith("data:image/jpeg;base64,"))
        {
        val = val.substring("data:image/jpeg;base64,".length());
        }
        
      try 
        {
        feed = DecodePImageFromBase64(val); // adds "no" delay (~5ms)
        image(feed, 0, 0);
        checkForColors(feed);
        }
      catch (IOException e) 
        {
        println(e);
        }
    }
  } /*else {
        try 
        {
        feed = DecodePImageFromBase64(valTest);
        image(feed, 0, 0);
        checkForColors(feed);
        noLoop();
        }
      catch (IOException e) 
        {
        println(e);
        }  
    }*/
}

public void checkForColors(PImage image){
  image.loadPixels();
  int[] pixs = image.pixels;
  int[][] placesRed = new int[320*240][2];
  int[][] placesGreen = new int[320*240][2];
  int[][] placesBlue = new int[320*240][2];
          /*
            [
             [172, 146],
             [179, 146],
             [180, 150]
            ]
          */
  
  int jRed = 0;
  int jGreen = 0;
  int jBlue = 0;
  for(int i=0; i<pixs.length; i++){
    //println(pix);
    int x = i%width;
    int y = i/width;
    color pix = pixs[i];
    int r = (pix>>16) &0xFF;
    int g = (pix>>8)  &0xFF;
    int b = (pix)     &0xFF;
    
    if(r > g && r > b){
      if(r-g > THRESH && r-b > 15){
        placesRed[jRed][0] = x;
        placesRed[jRed][1] = y;
        jRed++;
        
      } 
    } else if(g > r && g > b){
      if(g-r > THRESH && g-b > 15){
        placesGreen[jGreen][0] = x;
        placesGreen[jGreen][1] = y;
        jGreen++;
        
      }
    } else if(b > r && b > g){
      if(b-r > THRESH && b-g > 15){
        placesBlue[jBlue][0] = x;
        placesBlue[jBlue][1] = y;
        jBlue++;
      
      }
    }
    
    
  }
  //println(j);
  
  averageCoordinates averageRed = calculateAverage(placesRed, jRed);
  averageCoordinates averageGreen = calculateAverage(placesGreen, jGreen);
  averageCoordinates averageBlue = calculateAverage(placesBlue, jBlue);
  
  if(averageRed.getX() != -1)
    sendMidi(averageRed, 1, 20);
  
  if(averageGreen.getX() != -1)
    sendMidi(averageGreen, 1, 22);
  if(averageBlue.getX() != -1)
    sendMidi(averageBlue, 1, 24);
    
  
  fill(#ff0000);
  circle(averageRed.getTrueX(), averageRed.getTrueY(), 10);
  fill(#00ff00);
  circle(averageGreen.getTrueX(), averageGreen.getTrueY(), 10);
  fill(#0000ff);
  circle(averageBlue.getTrueX(), averageBlue.getTrueY(), 10);
}

public PImage DecodePImageFromBase64(String i_Image64) throws IOException{
  PImage result = null;
  println(i_Image64);
  
  byte[] decodedBytes = Base64.getDecoder().decode(i_Image64); //IllegalArgumentException: Input byte array has incorrect ending byte at 6372 OR Illegal base64 character d. String is correct JPG
  //byte[] decodedBytes = javax.xml.bind.DatatypeConverter.parseBase64Binary(i_Image64);
  //println(decodedBytes);

  ByteArrayInputStream in = new ByteArrayInputStream(decodedBytes);
  BufferedImage bImageFromConvert = ImageIO.read(in);
  BufferedImage convertedImg = new BufferedImage(bImageFromConvert.getWidth(), bImageFromConvert.getHeight(), BufferedImage.TYPE_INT_ARGB);
  convertedImg.getGraphics().drawImage(bImageFromConvert, 0, 0, null);
  result = new PImage(convertedImg); //Depracated but works

  return result;
} 

final class averageCoordinates{
  private final float midiX;
  private final float midiY;
  private final float trueX;
  private final float trueY;
  
  public averageCoordinates(float _x, float _y, float _mX, float _mY){
    midiX = _x;
    midiY = _y;
    trueX = _mX;
    trueY = _mY;
  }
  
  public float getX(){
    return midiX;
  }
  public float getY(){
    return midiY;
  }
  public float getTrueX(){
    return trueX;
  }
  public float getTrueY(){
    return trueY;
  }
}

public averageCoordinates calculateAverage(int[][] coordinates, int num){
  int sumX = 0;
  int sumY = 0;
  float midiX = 0;
  float midiY = 0;
  float trueX = 0;
  float trueY = 0;
  
  if(num > 0){
    for(int i=0; i<num; i++){
      sumX += coordinates[i][0];
      sumY += coordinates[i][1];
    }
    midiX = sumX / num;
    midiY = sumY / num;
    
    trueX = midiX;
    trueY = midiY;
    
    midiX = map(midiX, 0, 320, 127, 0);
    midiY = map(midiY, 0, 240, 127, 0);
    return new averageCoordinates(midiX, midiY, trueX, trueY);
  } else {
    return new averageCoordinates(-1, -1, -1, -1);
  }
}

public void sendMidi(averageCoordinates coordinates, int chan, int cc){
  myBus.sendControllerChange(chan, cc,(int) coordinates.getX());
  myBus.sendControllerChange(chan, cc+1,(int) coordinates.getY());
}
```

- A Max, rebo els missatges MIDI i recreo la posició de les tres peces utilitzant l'objecte `pictslider`
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/Pasted_image_20260114204259.png]]
- Afegint comunicació amb Ableton Live (ajudant-me d'[aquest tutorial](https://www.youtube.com/watch?v=Wy3SYumK5Vg), he transformat el patch de Max en un patch de Max For Live, el que ofereix la possibilitat de mapejar qualsevol de les coordenades de qualsevol dels punts a qualsevol paràmetre dins de Live.
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/Pasted_image_20260115100258.png]]
- El resultat final és un dispositiu que envia missatges MIDI, per tant, les seves aplicacions no estàn limitades als patchos de Max que he dissenyat. També se li pot donar ús dins del VCV Rack, altres DAWs o programes que acceptin MIDI.
![[https://github.com/papertree1/multiXY/blob/c9f9bedf0d03823562d33267b557cb63bc7b1882/Attachments/VID_20260115_102055318.mp4]]
# Conclusions
Aquest LabSo ha sigut, per mi, una lliçó enorme en paciència i en saber buscar maneres alternatives. La gran majoria de la feina ha sigut recerca de diferents eines, proves i "fracassos" (en el sentit de no aconseguir de les eines allò que buscava). De totes maneres, m'ha servit moltíssim per a trobar metodologies de recerca de diferents sensors i maneres, per a aprendre a fer mesures que m'aportin informació rellevant sobre el sensor que estava provant, i per trobar un mètode de treball quan res funcionava.
La decisió final de fer servir una càmera en comptes de les altres opcions de sensors va ser encertada pel projecte en un marc de LabSo, però si hagués tingut més temps m'hagués agradat trobar una manera més "mecànica" que no depengués tant del software. A més, la implementació que tinc amb la càmera és poc pràctica com a dispositiu MIDI, ja que és enorme (al final fa 15x15x20cm). Les implementacions que intentava amb sensors com el VL530X eren més elegants, en la meva opinió, i s'adherien més a la idea incial del projecte.

Quant als objectius proposats a principi del projecte, els he complert la majoria:
-     Detecció precisa de la posició d'un objecte en un pla.
-     Quan s'aconsegueixi, detecció precisa i diferenciada de més d'un objecte en un pla.
-     Resolució de <1cm.
-     Latència de <150ms. (?)
-     Comunicació a temps real amb un patch de Max/MSP.
L'únic que no he pogut assolir és el de la latència. Actualment, aquesta implementació té una latència d'uns 450-500ms, que es nota moltíssim a l'hora de fer servir el dispositiu i és el defecte més gran del disseny, ja que elimina completament les possibilitats de fer-lo servir en directe de manera precisa.
