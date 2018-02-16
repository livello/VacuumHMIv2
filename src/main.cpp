// UTouch_ButtonTest (C)2010-2014 Henning Karlsen

// web: http://www.henningkarlsen.com/electronics

//

// This program is a quick demo of how create and use buttons.

//

// This program requires the UTFT library.

//

// It is assumed that the display module is connected to an

// appropriate shield or that you know how to change the pin

// numbers in the setup.

//


#include "Arduino.h"
#include "HardwareSerial.h"

#include <UTFT.h>

#include <UTouch.h>

#define dX 8

#define dY 6

#define TICK 50

#define ACTIVE_PROC_REPAINT_DELAY 500

void printMs(int column, int row);
void drawButton(int column, int row);
void tick();
void  activateProc(int proc_num, boolean enable);
void drawAdjButtons();
void clickAdjButton(int i);


UTFT myGLCD(ILI9327_8, 38, 39, 40, 41);

UTouch  myTouch( 6, 5, 4, 3, 2);



extern uint8_t BigFont[];



int x, y;

char stCurrent[20] = "";

int stCurrentLen = 0;

int selectedButton = 0, lastSelectedButton = -1;

char stLast[20] = "";

const char buttonLabel[][ 6 ][ 7 ] = {"HAXPEB", "CTOL", "BAKYYM", "XOLOD", "BO3DYX"};

const char adjLabel[][ 3 ][ 5 ] = {"+500", "+100", "-500"};

const int ADJ_DELTA[6] = {500, 100, -500};



volatile long int PROC_DURATION_MS[6] = {30000, 30000, 30000, 30000, 500, 0}; //Длительность процессов

const int RELAY[6] = {8, 9, 10, 11, 12, 13};

long int procTimeLeft[6] = {0, 0, 0, 0, 0, 0};

unsigned long last_tick = 0, last_repaint = 0;

boolean procActive[6] = {false, false, false, false, false, false};

boolean repainted = false;

volatile boolean duration_changed = false;



void drawButton(int column, int row)

{

  if (selectedButton == column + row * 2)

    myGLCD.setColor(VGA_YELLOW);

  else

    myGLCD.setColor(VGA_BLACK);

  myGLCD.fillRoundRect (column * 160 + dX / 2, 80 * row + dY / 2, column * 160 + 160 - dX / 2, 80 * row + 80 - dY / 2);

  if (procActive[column + row * 2])

    myGLCD.setColor(VGA_RED);

  else

    myGLCD.setColor(VGA_BLUE);

  myGLCD.fillRoundRect (column * 160 + dX, 80 * row + dY, column * 160 + 160 - dX, 80 * row + 80 - dY);

  myGLCD.setColor(255, 255, 255);

  myGLCD.print(buttonLabel[0][column + row * 2], column * 160 + dX + 10, 80 * row + dY);

  printMs(column, row);

  //  myGLCD.printNumI(column, column * 200 + dX + 10, 80 * row + dY);

}

void printMs(int column, int row)

{ if (procActive[column + row * 2])

    myGLCD.printNumI(procTimeLeft[column + row * 2], column * 160 + dX + 40, 80 * row + dY + 20);

  else

    myGLCD.printNumI(PROC_DURATION_MS[column + row * 2], column * 160 + dX + 40, 80 * row + dY + 20);

}

void tick()

{

  for (int i = 0; i < 6; i++) {

    if (procTimeLeft[i] > 0) {

      procTimeLeft[i] = procTimeLeft[i] - (millis() - last_tick);

    }

  }

  last_tick = millis();

  for (int i = 0; i < 6; i++) {

    if (procTimeLeft[i] <= 0 && procActive[i]) {

      activateProc(i, false);

    }

    if (procActive[i] && millis() > last_repaint) {

      repainted = true;

      printMs(i % 2, (int)(i / 2));

    }

  }

  if (millis() > last_repaint && duration_changed) {

    if (PROC_DURATION_MS[selectedButton] < 0)

      PROC_DURATION_MS[selectedButton] = -PROC_DURATION_MS[selectedButton];

    printMs(selectedButton % 2, (int)(selectedButton / 2));

    repainted = true;

    duration_changed = false;

  }

  if (repainted) {

    last_repaint  = millis() + ACTIVE_PROC_REPAINT_DELAY;

    repainted = false;

  }



}

void  activateProc(int proc_num, boolean enable)

{

  if (procActive[proc_num] != enable) {

    if (enable) {

      digitalWrite(RELAY[proc_num], LOW);

      procTimeLeft[proc_num] = PROC_DURATION_MS[proc_num];

    }

    else

    {

      digitalWrite(RELAY[proc_num], HIGH);

      procTimeLeft[proc_num] = 0;

    }

    procActive[proc_num] = enable;

    drawButton(proc_num % 2, (int)(proc_num / 2));

  }



}

void loop()

{

  while (true)

  {

    if (myTouch.dataAvailable())

    {

      myTouch.read();

      if (myTouch.getX() == -1 || myTouch.getY() == -1)

        continue;

      if (myTouch.getX() <= 320)

      {

        selectedButton = -1;

        drawButton(x, y);

        x = (int)(myTouch.getX() / 160);

        y = (int)(myTouch.getY() / 80);

        selectedButton = x + y * 2;

        if (lastSelectedButton != selectedButton)

        {

          lastSelectedButton = selectedButton;

          drawButton(lastSelectedButton % 2, (int)(lastSelectedButton / 2));

        }

        else

        {

          activateProc(selectedButton, !procActive[selectedButton]);

        }

        lastSelectedButton = selectedButton;        

      }

      else

      {

        clickAdjButton((int)(myTouch.getY() / 80));

      }

    }

    tick();

    delay(TICK);

  }



}

void drawAdjButtons()

{

  for(int i=0;i<3;i++)

  {

  myGLCD.setColor(VGA_NAVY);

  myGLCD.fillRoundRect (320+dX, 0+dY+i*80,400-dX, 80-dY+i*80);

  myGLCD.setColor(VGA_WHITE);

  myGLCD.print(adjLabel[0][i], 320+dX/2, 0+dY+i*80);

  } 

}


void clickAdjButton(int i)

{

  if(selectedButton>=0)

 {

     PROC_DURATION_MS[selectedButton]+=ADJ_DELTA[i];

     duration_changed = true;

 } 

}

void setup()

{

  for (int i = 0; i < 6; i++) {

    pinMode(RELAY[i], OUTPUT);

    digitalWrite(RELAY[i], HIGH);

  }

  

  Serial.begin(115200);

  myGLCD.InitLCD(LANDSCAPE);

  myGLCD.clrScr();

  myTouch.InitTouch(1);

  myTouch.setPrecision(PREC_HI);



  myGLCD.setFont(BigFont);

  myGLCD.setBackColor(0, 0, 255);

  myGLCD.setColor(VGA_BLUE);



  for (int i = 0; i < 2; i++)

    for (int t = 0; t < 3; t++)

      drawButton(i, t);

  last_tick = millis();  

  drawAdjButtons();

}


