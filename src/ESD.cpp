#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <RA8875.h>
#include "fonts/akashi_36.c"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

/*
  NOTES:
  1. I have not tested the smoothing, but it should work
*/

//pins
#define rotaryButton 2  //define rotary encoder

//debug tools
#define debugMode true     //use to enable all debug commands
#define overideSensors (debugMode && true)  //overide "X" if sensors are not connected
#define disableAudio (debugMode && true)  //disable teensy onboard audio

//SD card
#define fileDir "flights"
#define fileName "testFlight"
char fileNameChar[sizeof(fileName) - 1 + sizeof(fileDir) -1 + 8];
int fileNum = 0;
long int sdTime = 0;
bool logging = false;

#define RA8875_CS 10 //see below...
#define RA8875_RESET 9//any pin or nothing!

//define different Colors
#define TFT_BLACK   0x0000
#define TFT_GREY    0x8410
#define TFT_WHITE   0xFFFF
#define TFT_RED     0xF800
#define TFT_ORANGE  0xFA60
#define TFT_YELLOW  0xFFE0
#define TFT_LIME    0x07FF
#define TFT_GREEN   0x07E0
#define TFT_CYAN    0x07FF
#define TFT_AQUA    0x04FF
#define TFT_BLUE    0x001F
#define TFT_MAGENTA 0xF81F
#define TFT_PINK    0xF8FF

#define RPMMAX 3000
#define RED2 2800
#define YELLOW2 2500
#define GREEN -1
#define YELLOW1 -1

int TXRXtime1 = 0;
int TXRXtime2 = 0;

#define degred2 240 - ((270* RED2)/ RPMMAX)
#define degyellow2 240 - ((270*YELLOW2)/ RPMMAX)
#define deggreen 240 - ((270*GREEN) / RPMMAX)
#define degyellow1 240 - ((270*YELLOW1) / RPMMAX)

String wrap = ";";
bool READING1 = false;
bool READING2 = false;
char str1[30]; //Initialized variable to store recieved data
char str2[30];
int charnum1 =0;
int charnum2 =0;
String storesave1 = "";
String storesave2 = "";
String Serial1Reading = "";
String Serial2Reading = "";
String storesave = "";

float voltage = 0;
float voltageprev = 0;
int voltyprev = 425;

#define numReadings 10
float current = 0;
float currentprev = 0;
float currentReadings[numReadings];
int currentIndex =0;
float currentTotal = 0;
long int currentPreMillis = 0;

float rpm = 0;
float rpmprev = 0;
float rpmdegprev = 240;
int perthrot = 0;
int perthrotprev = 0;
float powerdegprev = 0;
float power = 0;
float powerprev = 0;
int v9yprev = 100;

//int d1 = 90;
#define v1 500
#define v2 500
#define v3 500
//int v4 = 500;
//int v5 = 500;
int rev1 = 1, rev2 = 1;


int batPer = 100;

int rQ = 0;
int rK = 0;
int rQi = 0;
int rKi = 0;

int contemp = 0;
int mottemp = 0;
int battemp = 0;
int contempprev = 0;
int mottempprev = 0;
int battempprev = 0;
int contempprevy = 425;
int mottempprevy = 425;
int battempprevy = 425;

double bat = 0;

#define batCAP 50 // Battery Max. Capacity in Ah
#define celln 28 // number of cells in battery

float AMPHR = 0;
float KWHR = 0;

bool singleSave = true;
int count1 = 0;
int pressedTime = 0;
int betweenpressedTime = 0;

int countaux = 0;
float auxavg = -1;
float auxavgprev = 0;
double cellvoltage = 0;

int getdata = 1;
int getdataprev = 1;
int cancount = 0;
bool canON = false;
bool canONprev = false;

int scrolltime = 0;

float aux = 12;
unsigned int eeAddress = 0;
bool buttonON = false;
bool arrowTOG = false;
bool resetlight = false;
int initialT = 0;
int ATB = 0;
int ATBprev = 0;
long int seconds = 0;
long int secondsSum = 0;
long int newSeconds = 0;
bool newTime = false;
int hours = 0;
int minutes = 0;
RA8875 tft = RA8875(RA8875_CS, RA8875_RESET); //Teensy3/arduino's


// function calling dial display, with centers, radius of display, added thickness of colored regions, and degree limits (mindeg, maxdeg, y1, g, y2, r2) for colored regions

void circledisplay(int centerx, int centery, int radius, int thickness, int mindeg, int maxdeg, int y1 , int g , int y2 , int r2) {
  for (int i = mindeg; i < maxdeg; i++) {

    // this part of the loop creates the dial of desired thickness by creating two triangles that fill in the region of a quadrilateral between four points: (r,theta), (r+thickness, theta), (r, theta+delta(theta)), (r+thickness, theta+delta(theta)) [here,  delta(theta) = 1 degree]

    int x_cur = radius * cos(i * PI / 180.0) + centerx;
    int y_cur = radius * sin(-i * PI / 180.0) + centery;

    int x_cur_thick = (radius + thickness) * cos(i * PI / 180.0) + centerx;
    int y_cur_thick = (radius + thickness) * sin(-i * PI / 180.0) + centery;

    int x_next = radius * cos((i + 2) * PI / 180.0) + centerx;
    int y_next = radius * sin(-(i + 2) * PI / 180.0) + centery;

    int x_next_thick = (radius + thickness) * cos((i + 2) * PI / 180.0) + centerx;
    int y_next_thick = (radius + thickness) * sin(-(i + 2) * PI / 180.0) + centery;


    // coloring the quadrilaterals that make up the static part of the dial, depending on their degree limits

    if ((i <= y1) && (i >= mindeg)) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_RED);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_RED);
    }

    if ((i <= g) && (i > y1)) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_YELLOW);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_YELLOW);
    }

    if ((i <= y2) && (i > g)) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_GREEN);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_GREEN);
    }

    if ((i <= r2) && (i > y2)) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_YELLOW);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_YELLOW);
    }

    if ((i <= maxdeg) && (i > r2)) {
      tft.fillTriangle(x_cur, y_cur, x_cur_thick, y_cur_thick, x_next, y_next, TFT_RED);
      tft.fillTriangle(x_cur_thick, y_cur_thick, x_next, y_next, x_next_thick, y_next_thick, TFT_RED);
    }

    // drawing white border lines at each step to create a continuous curve border on inside and out

    tft.drawLine(x_cur, y_cur, x_next, y_next, TFT_WHITE);
    tft.drawLine(x_cur_thick, y_cur_thick, x_next_thick, y_next_thick, TFT_WHITE);
  }

  // closing the border on both ends with straight lines

  tft.drawLine(radius * cos(mindeg * PI / 180.0) + centerx, radius * sin(-mindeg * PI / 180.0) + centery, (radius + thickness)*cos(mindeg * PI / 180.0) + centerx, (radius + thickness)*sin(-mindeg * PI / 180.0) + centery, TFT_WHITE);
  tft.drawLine(radius * cos((maxdeg + 1)*PI / 180.0) + centerx, radius * sin(-(maxdeg + 1)*PI / 180.0) + centery, (radius + thickness)*cos((maxdeg + 1)*PI / 180.0) + centerx, (radius + thickness)*sin(-(maxdeg + 1)*PI / 180.0) + centery, TFT_WHITE);
}





// battery function called, effectively two rectanges with a deleted line between top and middle parts
void battery(int centerx , int centery , int H1 , int W1 , int H2 , int W2) {
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  tft.drawRect(x, y, W1, H1, TFT_WHITE);
  tft.drawRect(x + ((W1 - W2) / 2), y - H2, W2, H2, TFT_WHITE);
  tft.drawLine(x + ((W1 - W2) / 2) + 1, y, x + ((W1 + W2) / 2) - 2, y, TFT_BLACK);
  tft.drawLine(x + ((W1 - W2) / 2) + 1, y - 1, x + ((W1 + W2) / 2) - 2, y - 1, TFT_BLACK);
}

void linedisp(int lxi, int lyi, int heig, int ticks, int ST, int LT) {
  tft.drawLine(lxi, lyi, lxi, lyi + heig, TFT_WHITE);
  tft.drawLine(lxi, lyi, lxi - LT, lyi, TFT_WHITE);
  tft.drawLine(lxi, lyi + heig, lxi - LT, lyi + heig, TFT_WHITE);
  int diff = heig / (ticks + 1);
  for (int i = 1; i <= ticks; i++) {
    tft.drawLine(lxi, lyi + (i * diff), lxi - ST, lyi + (i * diff), TFT_WHITE);
  }
}



// function to fill battery

void fillbat(int centerx /* center x */, int centery /* center y*/, int H1 /* height of middle part*/, int W1 /* width of middle part*/, int H2 /* height of top part */, int W2 /* width of top part */, double P /* percent of battery full */, uint16_t /* color to fill with */color) {

  int HM = int(P * (H1 + H2));
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  float prop = (float)H1 / (H1 + H2);

  // two cases, depending on whether or not fill region reaches top part

  if (P <= prop) {
    tft.fillRect(x + 2, y + (H1 - HM) + 2, W1 - 4, HM - 4, color);
  }

  if (P > prop) {
    tft.fillRect(x + 2, y + 2, W1 - 4, H1 - 4, color);
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - (HM - H1) + 2, W2 - 4, HM - H1, color);
  }

}

// function to fill inverse of battery,  used to fill remainder of battery black and mitigate flicker whenever there is a change in the amount of battery that is full (P is percent battery full)

void invfillbat(int centerx, int centery, int H1, int W1, int H2, int W2, double P, uint16_t color) {

  int HM = int(P * (H1 + H2));
  int x = centerx - (W1 / 2);
  int y = centery - (H1 / 2);
  float prop = (float)H1 / (H1 + H2);

  if (P <= prop) {
    tft.fillRect(x + 2, y + 2, W1 - 4, (H1 - HM) - 4, color);
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - (H2) + 2, W2 - 4, H2, color);
  }

  if (P > prop) {
    tft.fillRect(x + (W1 - W2) / 2 + 2, y - H2 + 2, W2 - 4, H2 - HM + H1, color);
  }

}

// function calling the pointer of the dial, which is drawn as a triangle with points: (radius-len, deg+(thick/2)), (radius-len, deg-(thick/2)), and (radius, deg)
void dialcirc(int16_t centerx, int16_t centery, int16_t radius, int16_t len, int16_t deg, int16_t thick, uint16_t colorfill, uint16_t colorborder) {
  int x1 = (radius - len) * cos((deg - (thick / 2)) * PI / 180.0) + centerx;
  int y1 = (radius - len) * sin(-(deg - (thick / 2)) * PI / 180.0) + centery;
  int x2 = (radius - len) * cos((deg + (thick / 2)) * PI / 180.0) + centerx;
  int y2 = (radius - len) * sin(-(deg + (thick / 2)) * PI / 180.0) + centery;
  int x3 = (radius) * cos((deg) * PI / 180.0) + centerx;
  int y3 = (radius) * sin(-(deg) * PI / 180.0) + centery;
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, colorfill);
  tft.drawLine(x1, y1, x2, y2, colorborder);
  tft.drawLine(x2, y2, x3, y3, colorborder);
  tft.drawLine(x3, y3, x1, y1, colorborder);
}

void countdown (int16_t P) {
  for (int i = 0; i < 6; i++) {
    tft.drawLine(647 + P, 428 + i, 747, 428 + i, TFT_GREEN);
    tft.drawLine(655, 428 + i, 655 + (P - 8), 428 + i, TFT_BLACK);
  }
}

// bar display indicator with global coordinates (x,y)
void vertdialbar (int16_t x, int16_t y, int16_t len, int16_t thick, uint16_t colorfill, uint16_t colorborder) {
  int x1 = x;
  int y1 = y;
  int x2 = x + (len);
  int y2 = y + (thick / 2);
  int x3 = x + (len);
  int y3 = y - (thick / 2);
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, colorfill);
  tft.drawLine(x1, y1, x2, y2, colorborder);
  tft.drawLine(x2, y2, x3, y3, colorborder);
  tft.drawLine(x3, y3, x1, y1, colorborder);
}

// this function determines what the color of the dial indicator/triangle should be
void dialcirccolor (int16_t centerx, int16_t centery, int16_t radius, int16_t len, int16_t deg, int16_t thick, uint16_t colord, int mindeg, int maxdeg, int y1, int g, int y2, int r2) {
  if ((deg >= mindeg) && (deg <= y1)) {
    colord = TFT_RED;
  }
  if ((deg > y1) && (deg <= g)) {
    colord = TFT_YELLOW;
  }
  if ((deg > g) && (deg <= y2)) {
    colord = TFT_GREEN;
  }
  if ((deg > y2) && (deg <= r2)) {
    colord = TFT_YELLOW;
  }
  if ((deg > r2) && (deg <= maxdeg)) {
    colord = TFT_RED;
  }
  dialcirc(centerx, centery, radius, len, deg, thick, colord, TFT_WHITE);
}

// ditto the above funciton for a vertical bar
void vertdialbarcolor (int16_t x, int y, int16_t topy, int16_t boty, int16_t len, int16_t thick, uint16_t colord, int p1, int p2, int p3, int p4) {
  if ((y >= topy) && (y <= (topy + p1))) {
    colord = TFT_RED;
  }
  if ((y > (topy + p1)) && (y <= (topy + p1 + p2))) {
    colord = TFT_YELLOW;
  }
  if ((y > (topy + p1 + p2)) && (y <= (topy + p1 + p2 + p3))) {
    colord = TFT_GREEN;
  }
  if ((y > (topy + p1 + p2 + p3)) && (y <= (topy + p1 + p2 + p3 + p4))) {
    colord = TFT_YELLOW;
  }
  if ((y > (topy + p1 + p2 + p3 + p4) )&& (y <= boty)) {
    colord = TFT_RED;
  }
  vertdialbar(x, y, len, thick, colord, TFT_WHITE);
}

//function that will display a value with [dec] number of decimals, and [dig] number of digits.
//"per" is a variable input to display as the number, and [x] and [y] is the leftmost position of the
//greatest digit. [FontSize] is the font of the digit. The function will ensure no flicker by
//opting to display a new value onto of the old value with a black backround, effectlively "stamping"
//itself onto the previous value. [str] is an optional string that can be attached somewhere near the [per]
//of size [stringFontSize] at relative position [xadj] away from [x] and [yadj] away from [y]. [color] will
//be the visible color of both text, and [Zeros] will enable 0's instead of empty space for extra digits.
void LText(double per, int x, int y, float FontSize, int dec, int dig, String str,int xadj,int yadj,float stringFontSize,int color, bool Zeros)
{
  tft.setFontScale(FontSize);
  tft.setTextColor(color,TFT_BLACK);
  tft.setCursor(x,y);
  for(int i = dig -1; i > 0; i--)
  {
    if(per/pow(10,i) < 1)
    {
      if(Zeros)
      {
        tft.print(0);
      }
      else
      {
        tft.print(" ");
      }
    }
  }
  tft.print(per,dec);
  tft.setFontScale(stringFontSize);
  tft.setCursor(x+xadj, y+yadj);
  tft.print(str);
}

//-------------------SD Save-------------------
//prints current data readings to SD card
void SDSave()
{
  String dataString = "";
  dataString += String(millis()-sdTime);
  dataString += ",";
  dataString += String(seconds);
  dataString += ",";
  dataString += String(hours);
  dataString += ",";
  dataString += String(minutes);
  dataString += ",";
  dataString += String(voltage);
  dataString += ",";
  dataString += String(aux);
  dataString += ",";
  dataString += String(current);
  dataString += ",";
  dataString += String(rpm);
  dataString += ",";
  dataString += String(perthrot);
  dataString += ",";
  dataString += String(contemp);
  dataString += ",";
  dataString += String(mottemp);
  dataString += ",";
  dataString += String(battemp);
  dataString += ",";
  dataString += String(KWHR);
  dataString += ",";
  dataString += String(AMPHR);
  dataString += ",";
  File dataFile = SD.open(fileNameChar,FILE_WRITE);
  if(dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    Serial.println("error opening .csv");
  }
}

//function to create a new SD File
void SDNew()
{
  sdTime = millis();
  //create a directory to hold the data
  char FILEDirChar[sizeof(fileDir)-1];
  for (int i = 0; i < sizeof(fileDir); i++)
  {
    FILEDirChar[i] = fileDir[i];
  }
  if(!SD.exists(FILEDirChar))
  {
    SD.mkdir(FILEDirChar);
  }

  //find the nex available fileName for use
  while(fileNum < 999)
  {
    for(int i = 0; i < 3;i++)
    {
      fileNameChar[sizeof(fileNameChar) - 7 + (2-i)] = (char)((int)('0') + ((fileNum/(int)pow(10,i))%10));
    }
    if(!SD.exists(fileNameChar))
      {
        Serial.print("logging as:");
        Serial.println(fileNameChar);
        break;
      }
    fileNum++;
  }
  File dataFile = SD.open(fileNameChar,FILE_WRITE);
  String nameString = "";
  nameString += "Active Time(ms)";
  nameString += ",";
  nameString += "Flight Time(ms)";
  nameString += ",";
  nameString += "Hours";
  nameString += ",";
  nameString += "Minutes";
  nameString += ",";
  nameString += "Voltage";
  nameString += ",";
  nameString += "Aux";
  nameString += ",";
  nameString += "Current";
  nameString += ",";
  nameString += "RPM";
  nameString += ",";
  nameString += "% Throttle";
  nameString += ",";
  nameString += "ContTemp";
  nameString += ",";
  nameString += "MotTemp";
  nameString += ",";
  nameString += "BatTemp";
  nameString += ",";
  nameString += "kWh";
  nameString += ",";
  nameString += "Ah";
  while(!dataFile);
  dataFile.println(nameString);
  dataFile.close();
}


void vertbardisplay(int topx, int topy, int botx, int boty, int p1, int p2, int p3, int p4) {
  tft.fillRect(topx, topy, botx - topx, p1, TFT_RED);
  tft.fillRect(topx, topy + p1, botx - topx, p2, TFT_YELLOW);
  tft.fillRect(topx, topy + p1 + p2, botx - topx, p3, TFT_GREEN);
  tft.fillRect(topx, topy + p1 + p2 + p3, botx - topx, p4, TFT_YELLOW);
  tft.fillRect(topx, topy + p1 + p2 + p3 + p4, botx - topx, boty - (topy + p1 + p2 + p3 + p4), TFT_RED);
  tft.drawLine(topx, topy, botx, topy, TFT_WHITE);
  tft.drawLine(topx, topy, topx, boty, TFT_WHITE);
  tft.drawLine(topx, boty, botx, boty, TFT_WHITE);
  tft.drawLine(botx, topy, botx, boty, TFT_WHITE);
}


// function calling a bar display, with TL and BR coordinates (topx, topy) and (botx, boty), respectively
// p1, p2, p3, and p4 are the respective pixel heights of the red, yellow, green, and yellow regions
// the final red region is calculated from the remainder of the height minus the sum of other region heights
//function to draw the basic outline of the visuals
void visual() {

  tft.drawRect(650, 426, 100, 10, TFT_WHITE);

  if (arrowTOG) {
    tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_WHITE);
    tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_BLACK);
  }
  else {
    tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_WHITE);
    tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_BLACK);
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK); // static labels
  tft.setFontScale(1);
  tft.setCursor(35, 230);
  tft.println("Motor");
  tft.setCursor(135, 230);
  tft.println("Contrl");
  tft.setCursor(260, 230);
  tft.println("Bat");
  tft.setCursor(520, 395);
  tft.println("Aux");
  tft.setCursor(660, 385);
  tft.println("RESET");
  tft.setCursor(655, 440);
  if(logging == false){
  tft.println("SD OFF");
  }
  if(logging == true){
  tft.println("SD ON");
  }
  tft.setFontScale(2);
  tft.setCursor(490, 235);
  tft.println("Main");
  tft.setCursor(620, 235);
  tft.println("Current");

  tft.setFontScale(0.75);
  tft.setCursor(103, 463);
  tft.println("C");
  tft.drawCircle(100, 463, 2, TFT_WHITE);
  tft.setCursor(209, 463);
  tft.println("C");
  tft.drawCircle(206, 463, 2, TFT_WHITE);
  tft.setCursor(311, 463);
  tft.println("C");
  tft.drawCircle(308, 463, 2, TFT_WHITE);
  tft.setCursor(345, 255);
  tft.println("Hours");
  tft.setCursor(405, 255);
  tft.println("Minutes");
  tft.setCursor(340, 340);
  tft.println("Energy Consumed");
  tft.setCursor(20, 5);
  tft.println("Throttle");

  tft.setFontScale(2);
  tft.setCursor(390,270);
  tft.println(":");
  tft.setCursor(185, 175);
  tft.println("RPM");
  tft.setCursor(595, 175);
  tft.println("KW");
  tft.setFontScale(1);
  
  circledisplay(200, 118, 95, 15, -30, 240, degred2, degyellow2, deggreen, degyellow1); // calling all static display symbols

  circledisplay(600, 118, 95, 15, -30, 240, 0, 20, 270, 270);
  vertbardisplay(60, 270, 89, 445, 15, 23, 130, 9);
  vertbardisplay(166, 270, 195, 445, 15, 23, 130, 9);
  vertbardisplay(268, 270, 297, 445, 15, 23, 130, 9);
  linedisp(60, 30, 130, 3, 10, 20);
  battery(400, 105, 150, 100, 7, 50);

  tft.drawLine(0, 230, 330, 230, TFT_GREY); // segmenting display into five parts (top, flight time, energy consumed, temperatures, and battery information)
  tft.drawLine(330, 230, 330, 480, TFT_GREY);
  tft.drawLine(470, 230, 470, 480, TFT_GREY);
  tft.drawLine(470, 230, 800, 230, TFT_GREY);
  tft.drawLine(330, 250, 470, 250, TFT_GREY);
  tft.drawLine(330, 325, 470, 325, TFT_GREY);
}




void setup() {

  tft.begin(RA8875_800x480);
  tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
  Serial.begin(9600); // For debug
  Serial1.begin(115200);
  Serial2.begin(115200);

  can1.begin();
  can1.setBaudRate(125000);

  EEPROM.get(eeAddress, secondsSum);
  seconds = secondsSum;
  eeAddress+= sizeof(secondsSum);
  EEPROM.get(eeAddress,KWHR);
  eeAddress+= sizeof(KWHR);
  EEPROM.get(eeAddress,AMPHR);

  visual();

  pinMode(8, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);

  if(!disableAudio)
  {
    SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
    SPI.setSCK(14);  // Audio shield has SCK on pin 14
  }
  

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
 
  //define inital values of rotary encoder
  rQ = digitalRead(3);
  rK = digitalRead(4);
  rQi = rQ;
  rKi = rK;

}


void loop() {
  //------------------------------------------------------------------------
  //these 2 functions takes data from Serial1 line and stores the data from it into
  //a string called "store save". Serial Readings must begin with ";", and end with "/"
  
  while (Serial1.available()) {
    str1[charnum1] = Serial1.read();
    if (str1[0] != ';') {
      charnum1 = 0;
      break;
    }
    if (charnum1 > 0) {
      if(charnum1 == 1)
      {
        storesave1 = "";
      }
      storesave1 += String(str1[charnum1]);
    }
    if(charnum1 > 100 || str1[charnum1] == '/')
    {
      charnum1 = 0;
      Serial1Reading = storesave1; 
      break;
    }
  }
  
  if ((Serial2.available()) && (READING2 == false)) {
    TXRXtime2 = millis() + 10;
    READING2 = true;
  }

  if ((TXRXtime2 < millis()) && (READING2 == true)) {

    int i = 0;
    storesave2 = ",";
    while (Serial2.available() && i < 30) {
      str1[i++] = Serial2.read();
      if (String(str1[0]) != ";") {
        break;
      }
      if (i > 0) {
        storesave2 += String(str1[i]);
      }
    }
    str1[i++] = '\0';
    READING2 = false;
  }
  
  //log to SD Card if logging enabled
  if(logging && millis()%100 <= 5)
  {
    SDSave();
  } 

  //read data from canbus
  if (can1.read(msg)) {
    
    if (msg.id == 346095618) {

      //voltage = (msg.buf[0] + (256 * msg.buf[1])) / 57.45;
      //implement smoothing to keep track of current by taking a certain number of samples in a second and updating the average every second
      currentTotal -= currentReadings[currentIndex];
      currentReadings[currentIndex] = (msg.buf[2] + (256 * msg.buf[3])) / 10;
      currentTotal += currentReadings[currentIndex];
      currentIndex++;
      if(currentIndex >= numReadings)
      {
        currentIndex = 0;
      }
      current = currentTotal/numReadings;
      currentPreMillis = millis();

      rpm = (msg.buf[4] + (256 * msg.buf[5]) + (65536 * msg.buf[6])) * 10;

    }

    if (msg.id == 346095619) {
      contemp = msg.buf[4];
      mottemp = msg.buf[5];
      battemp = msg.buf[6];

    }

    if (msg.id == 346095620) {

      perthrot = (msg.buf[2] + (256 * msg.buf[3])) / 10;

    }
    
    if (msg.id == 346095621) {

      voltage = ((msg.buf[6] + (256 * msg.buf[7])) / 10)-1; // reads ~1V high hence -1 offset

    }

    getdata++;
    getdata = getdata % 2;

  }
  else if (debugMode)
  {
    contemp = (millis()/100)%100;
    mottemp = ((millis()/200)+33)%100;
    battemp = ((millis()/300)+66)%100;
    perthrot = ((millis()/500)+80)%101;

    if(voltage > 118)
    {
      voltage = 118;
      rev1 = -1;
    }
    if(voltage < 84)
    {
      voltage = 84;
      rev1 = 1;
    }
    voltage += (.03)*rev1;
    
    if(current > 200)
    {
      current = 200;
      rev2 = -1;
    }

    if(current < 0)
    {
      current = 0;
      rev2 = 1;
    }
    current += (.02)*rev2;
    rpm = (millis()/10)%3000;
  }
  
  //determines whether or not to display statistics on screen by checking if data is updating
  if (getdataprev != getdata) {
    cancount = 0;
    canON = true;
  }
  else
  {
    cancount++;
  }

  if (cancount > 600) {
    canON = overideSensors; 
  }
  
  // will update flight time cummilative seconds ONLY if RPM > 500
  if(rpm>500)
  {
    if(newTime)
    {
      if(debugMode) newSeconds =  millis()/10;
      else
      {
        newSeconds = millis()/1000;
      }
      newTime = false;
    }

    if(debugMode) seconds = secondsSum + (millis()/10 - newSeconds);
    else {
      seconds = secondsSum + (millis()/1000 - newSeconds);
    }
  } else if (!newTime)
  {
    if(debugMode) secondsSum = secondsSum + (millis()/10 - newSeconds);
    else{
      secondsSum = secondsSum + (millis()/1000 - newSeconds);
    }
    newTime = true;
  }
  hours = (seconds/3600 )%100;
  minutes = (seconds/60) % 60;

  int v9y = map(perthrot, 100, 0, 30, 160);
  int rpmdeg = map((int) rpm, 0, RPMMAX, 240, -30);

  float power = ((voltage * current) / 1000);
  int powerdeg = map(power, 0, 20, 240, -30);

  int mottempy = map(mottemp, 99, 0, 270, 445);
  int battempy = map(battemp, 99, 0, 270, 445);
  int contempy = map(contemp, 99, 0, 270, 445);

  //---------KWHR / AMPHR definitions-------
    ATB = millis();
    //function to determine the current KWHR
    if (power > 0){
      KWHR = KWHR + ((ATB - ATBprev) * 0.00000027777777) * (power);
      
    } 
    //ensure KWHR stays positive
    if (KWHR < 0){
      KWHR = 0;
    }
    
    
    //function to determine the current AMPHR
    if (current > 0){
    AMPHR = AMPHR + ((ATB - ATBprev) * 0.000000277777) * (current);
    }
    
    if (AMPHR < 0){
      AMPHR = 0;
    }
    //----------------------------
    
    //determine battery percentage from AMPHR
    bat = (float)map((batCAP - AMPHR), 0, batCAP, 0, 100) / 100;
    batPer = int(100 * bat);


    if(overideSensors)
    {
      if (aux > 95)
      {
        aux = 0;
      }
      float newaux = random(-50,50)/10;
      if(aux + newaux > 0)
      {
        aux += newaux;
      }
      
    }
    else
    {
      aux = analogRead(2);
      aux = map(aux, 0, 1023, 0, 3.293);
      aux = 1.04167 * (19.06 + 5.495) / 5.49 * aux;
    }

    if (countaux < 10) {
      auxavg = auxavg + aux;
      countaux = countaux + 1;
    }

    else {
      auxavgprev = auxavg;
      countaux = 0;
      auxavg = 0;
    }
    
    //display non-can bus data at all times, display rest of values when Can bus is connected

    LText(minutes,412,270,2,0,2,"",0,0,0,TFT_WHITE,true);
    LText(hours,340,270,2,0,2,"",0,0,0,TFT_WHITE,true);
    LText(batPer,334,180,3,0,3,"%",100,29,1,TFT_WHITE,false);
    LText(auxavgprev/10,500,430,1,1,2,"V",68,15,.75,TFT_WHITE,false);
    LText(((float)((int)(10*KWHR)))/10,340,360,2,1,2,"kWh",100,28,.75,TFT_WHITE,false);
    LText(((float)((int)(10*AMPHR)))/10,340,410,2,1,2,"Ah",100,28,.75,TFT_WHITE,false);

    if (canON) {
      LText(perthrot,18,170,1,0,3,"%",50,14,.75,TFT_WHITE,false);
      LText(((float)((int)(10*power)))/10,550,80,2.75,1,2,"",0,0,0,TFT_WHITE,false);
      LText((int)rpm,135,85,3,0,4,"",0,0,0,TFT_WHITE,true);
      LText((int)(current),646,280,3,0,3,"A",98,30,1,TFT_WHITE,false);
      LText(cellvoltage,500,345,1,1,2,"V/ CELL",68,15,.75,TFT_WHITE,false);
      LText((int)voltage,480,280,3,0,3,"V",98,29,1,TFT_WHITE,false);
      LText(mottemp,60,450,1,0,2,"",0,0,0,TFT_WHITE,false);
      LText(contemp,166,450,1,0,2,"",0,0,0,TFT_WHITE,false);
      LText(battemp,268,450,1,0,2,"",0,0,0,TFT_WHITE,false);

      //reset visuals when canOn resets
      if (canONprev != canON) {
        tft.fillRect(0, 0, 800, 480, TFT_BLACK);
        visual();

      }

      if (rpmdegprev != rpmdeg) {
        dialcirc(200, 118, 93, 22, rpmdegprev, 30, TFT_BLACK, TFT_BLACK);
        dialcirccolor(200, 118, 93, 20, rpmdeg, 15, TFT_BLUE, -30, 240, 0, 20, 270, 270);
      }
      
      vertdialbar(91, v1, 20, 20, TFT_BLACK, TFT_BLACK);
      vertdialbar(197, v2, 20, 20, TFT_BLACK, TFT_BLACK);
      vertdialbar(299, v3, 20, 20, TFT_BLACK, TFT_BLACK);

      if (v9yprev != v9y) {
        vertdialbar(62, v9yprev, 10, 10, TFT_BLACK, TFT_BLACK);
      }

      if ((int)(10*powerdegprev) != (int)(10*powerdeg)) {
        dialcirc(600, 118, 93, 22, powerdegprev, 30, TFT_BLACK, TFT_BLACK);
        dialcirccolor(600, 118, 93, 20, powerdeg, 15, TFT_BLUE, -30, 240, degred2, degyellow2, deggreen, degyellow1);
      }

      if (mottemp != mottempprev) {
        vertdialbar(91, mottempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }

      if (battemp != battempprev) {
        vertdialbar(299, battempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }

      if (contemp != contempprev) {
        vertdialbar(197, contempprevy, 20, 20, TFT_BLACK, TFT_BLACK);
      }


      
      
      vertdialbar(62, v9y, 10, 10, TFT_WHITE, TFT_WHITE);     

      cellvoltage = voltage / celln ;
      
      vertdialbarcolor(91, mottempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);
      vertdialbarcolor(197, contempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);
      vertdialbarcolor(299, battempy, 270, 445, 20, 20, TFT_BLUE, 15, 23, 130, 9);

    }
    else //activates red "X" if values not updating (off?)
    {
      for (int i = 0; i <= 10; i++) {
        tft.drawLine(i, 0, 320 + i, 480, TFT_RED);
        tft.drawLine(i, 480, 320 + i, 0, TFT_RED);
        tft.drawLine(470 + i, 0, 800 + i, 380, TFT_RED);
        tft.drawLine(470 + i, 380, 800 + i, 0, TFT_RED);
      }
    }

 

    fillbat(400, 105, 150, 100, 7, 50, bat, TFT_GREEN);
    invfillbat(400, 105, 150, 100, 7, 50, bat, TFT_BLACK);
    
    if (count1 > 10) {

      int countper = map(count1 - 10, 1, 20, 5, 100);
      countdown(countper);

    }
    else {
      if (pressedTime == 0) {
        tft.fillRect(652, 428, 96, 6, TFT_GREEN);
      }
      else {
        tft.fillRect(652, 428, 96, 6, TFT_BLACK);
      }
    }

    tft.setFontScale(1);

    if (resetlight == true) {
      if (pressedTime > millis()) {
        tft.setCursor(660, 385);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("RESET");
      }
      else {
        tft.fillRect(650, 390, 100, 30, TFT_BLACK);
        tft.setCursor(660, 385);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println("RESET");
        resetlight = false;
      }
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    canONprev = canON;

// Save time to EEPROM on Powerdown (when input voltage falls below 12)
  if (aux <= 12.0 && singleSave && !debugMode) {

    eeAddress = 0;
    EEPROM.put(eeAddress, seconds);
    eeAddress+= sizeof(seconds);
    EEPROM.put(eeAddress,KWHR);
    eeAddress+= sizeof(KWHR);
    EEPROM.put(eeAddress,AMPHR);

    Serial.println("Saving for powerdown...");
    singleSave = false;

  }
  
  if(aux > 12.0) singleSave = true;

  
  buttonON = !digitalRead(rotaryButton);
  
  if (buttonON && (pressedTime == 0) && (millis() > betweenpressedTime)) {

    count1 = count1 + 1;
    betweenpressedTime = millis() + 50;

    if (count1 == 30) {
      tft.setFontScale(1);
      //if reset is selected
      if (arrowTOG == true) {
        if(!debugMode) //clear currently stored time
        {
          secondsSum = 0;
          seconds = 0;
          KWHR = 0;
          AMPHR = 0;
          eeAddress = 0;
          for (int i = 0 ; i < EEPROM.length() ; i++) 
          {
            EEPROM.write(i, 0);
          }
          Serial.println("Reseting...");
        }
        else //if debug mode is active, store current value when reset it selected
        {
          eeAddress = 0;
          Serial.println("Saving for powerdown...");
          EEPROM.put(eeAddress,seconds);
          eeAddress+= sizeof(seconds);
          EEPROM.put(eeAddress,KWHR);
          eeAddress+= sizeof(KWHR);
          EEPROM.put(eeAddress,AMPHR);

        }

        tft.fillRect(650, 390, 100, 30, TFT_BLACK);
        resetlight = true;

      }
      //if SD card is selected
      if (arrowTOG == false) {
        logging = !logging;
        static boolean logfailed;
        if(logging)
        {
          if (!SD.begin(BUILTIN_SDCARD)) {
            Serial.println("Card failed, or not present");
            logging = false;
            logfailed = true;
          }
          else
          {
            logging = true;
            logfailed = false;
            Serial.println("card initialized.");
            //intialize SD Card name
            String FILENAME = String(fileDir) + "/" + String(fileName) + "000.csv";
            for(int i = 0; i < FILENAME.length();i++)
            {
              fileNameChar[i] = FILENAME[i];
            }
            SDNew(); //Set a new directory (if neccessary)
          }
        }
        else
        {
          logfailed = false;
        }
        tft.fillRect(650, 440, 100, 30, TFT_BLACK);
        tft.setCursor(655, 440);

        if(logfailed)
        {
          tft.println("FAILED");
        }
        else if (logging) { 
          
          tft.println("SD ON");
        }
        else {
          tft.println("SD OFF");
        }
      }

      count1 = 0;
      pressedTime = millis() + 500;

    }

  }

  if (millis() - 25 > betweenpressedTime) {
    count1 = 0;
  }

  if (pressedTime < millis()) {
    pressedTime = 0;
  }

  getdataprev = getdata;

  rQ = digitalRead(3);
  rK = digitalRead(4);
  //function for determining which option is selected for rotary encoder (option can't change while button is pressed)
  if (((rQ != rQi) || (rK != rKi)) && (scrolltime <= millis()) && !buttonON) {
    arrowTOG = !arrowTOG;
    tft.fillRect(600,390,40,80,TFT_BLACK);
    scrolltime = millis() + 200;
  }
  if(arrowTOG)
  {
    tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_WHITE);
    tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_BLACK);
  }
  else
  {
    tft.fillTriangle(615, 445, 615, 465, 630, 455, TFT_WHITE);
    tft.fillTriangle(615, 395, 615, 415, 630, 405, TFT_BLACK);
  }
  
  rQi = rQ;
  rKi = rK;

  //set current values as old values
  contempprevy = contempy;
  mottempprevy = mottempy;
  battempprevy = battempy;
  contempprev = contemp;
  mottempprev = mottemp;
  battempprev = battemp;
  
  powerprev = power;
  powerdegprev = powerdeg;
  perthrotprev = perthrot;
  rpmprev = rpm;
  rpmdegprev = rpmdeg;
  v9yprev = v9y;
  ATBprev = ATB;
}
