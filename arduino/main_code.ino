#include <Servo.h>
#include <AFMotor.h>

AF_Stepper motorx(48, 1);
AF_Stepper motory(48, 2);
Servo servo;

#define penup 115
#define pendown 160
#define Spin 9

struct point {
  float x;
  float y;
  float z;
};

// position of drawing head
struct point actuatorPos;

float StepInc = 1;
int StepDelay = 1;
int LineDelay = 0;

float StepsPerMillimeterX = 250.0;
float StepsPerMillimeterY = 250.0;

//dimensions of drawing area in millimetres
float Xmin = 0;
float Xmax = 40;
float Ymin = 0;
float Ymax = 40;
float Zmin = 0;
float Zmax = 1;

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax;


void setup() {
  Serial.begin(9600);
  servo.attach(Spin);
  servo.write(penup);
  delay(100);
  motorx.setSpeed(600);
  motory.setSpeed(600);
  Serial.println("The CNC is ready");
}

void loop() {
  delay(100);
  char line[512];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (true) {

    while (Serial.available() > 0) {
      c = Serial.read();
      if ((c == '\n') || (c == '\r')) {  // reached end of line
        if (lineIndex > 0) {             // execute if line is ended
          line[lineIndex] = '\0';
          processIncomingLine(line, lineIndex);
          lineIndex = 0;
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");
      } else {
        if ((lineIsComment) || (lineSemiColon)) {  // discard comments and unwanted characters
          if (c == ')') {
            lineIsComment = false;
          }
        } else {
          if (c <= ' ') {
          } else if (c == '/') {
          } else if (c == '(') {
            lineIsComment = true;
          } else if (c == ';') {
            lineSemiColon = true;
          } else if (lineIndex >= 512 - 1) {
            Serial.println("error");
            lineIsComment = false;
            lineSemiColon = false;
          } else if (c >= 'a' && c <= 'z') { 
            line[lineIndex++] = c - 'a' + 'A';
          } else {
            line[lineIndex++] = c;
          }
        }
      }
    }
  }
}

void processIncomingLine(char* line, int charNB) {
  int currentIndex = 0;
  char buffer[64];
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;

  while (currentIndex < charNB) {
    switch (line[currentIndex++]) { // select commeands
      case 'U':
        penUp();
        break;
      case 'D':
        penDown();
        break;
      case 'G':
        buffer[0] = line[currentIndex++];  // assume G command has 2 digits
        buffer[1] = '\0';

        switch (atoi(buffer)) {
          case 0:                
          case 1:
            char* indexX = strchr(line + currentIndex, 'X'); 
            char* indexY = strchr(line + currentIndex, 'Y');
            if (indexY <= 0) {
              newPos.x = atof(indexX + 1);
              newPos.y = actuatorPos.y;
            } else if (indexX <= 0) {
              newPos.y = atof(indexY + 1);
              newPos.x = actuatorPos.x;
            } else {
              newPos.y = atof(indexY + 1);
              indexY = '\0';
              newPos.x = atof(indexX + 1);
            }
            drawLine(newPos.x, newPos.y);
            //Serial.println("ok");
            actuatorPos.x = newPos.x;
            actuatorPos.y = newPos.y;
            break;
        }
        break;
      case 'M':
        buffer[0] = line[currentIndex++];   // assume M command has 3 digits
        buffer[1] = line[currentIndex++];
        buffer[2] = line[currentIndex++];
        buffer[3] = '\0';
        switch (atoi(buffer)) {
          case 300:
            {
              char* indexS = strchr(line + currentIndex, 'S');
              float Spos = atof(indexS + 1);
              //Serial.println("ok");
              if (Spos == 30) {
                penDown();
              }
              if (Spos == 50) {
                penUp();
              }
              break;
            }
          case 114:
            Serial.print("Absolute position : X = ");
            Serial.print(actuatorPos.x);
            Serial.print("  -  Y = ");
            Serial.println(actuatorPos.y);
            break;
          default:
            Serial.print("Command not recognized : M");
            Serial.println(buffer);
        }
    }
  }
}

//lines drawing function
void drawLine(float x1, float y1) {

  y1 = y1 * -1;

  // transform within limits
  if (x1 >= Xmax) {
    x1 = Xmax;
  }
  if (x1 <= Xmin) {
    x1 = Xmin;
  }
  if (y1 >= Ymax) {
    y1 = Ymax;
  }
  if (y1 <= Ymin) {
    y1 = Ymin;
  }

  // convert to steps
  x1 = (int)(x1 * StepsPerMillimeterX / 2);
  y1 = (int)(y1 * StepsPerMillimeterY / 2);
  float x0 = Xpos;
  float y0 = Ypos;

  long dx = abs(x1 - x0);
  long dy = abs(y1 - y0);
  int sx = x0 < x1 ? StepInc : -StepInc;
  int sy = y0 < y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i = 0; i < dx; ++i) {
      motorx.onestep(sx, MICROSTEP);
      over += dy;
      if (over >= dx) {
        over -= dx;
        motory.onestep(sy, MICROSTEP);
      }
      delay(StepDelay);
    }
  } else {
    for (i = 0; i < dy; ++i) {
      motory.onestep(sy, MICROSTEP);
      over += dx;
      if (over >= dy) {
        over -= dy;
        motorx.onestep(sx, MICROSTEP);
      }
      delay(StepDelay);
    }
  }

  delay(LineDelay);
  
  Xpos = x1;
  Ypos = y1;
}


void penUp() {
  servo.write(penup);
  delay(50);
  Zpos = Zmax;
}

void penDown() {
  servo.write(pendown);
  delay(50);
  Zpos = Zmin;
}