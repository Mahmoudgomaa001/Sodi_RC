#include "HUSKYLENS.h"


#include <Servo.h>

Servo myservo;  // create servo object to control a servo

#define IN2 6
#define IN1 7

#define ENA 5

#define Trig_Front 4
#define Echo_Front 3

#define Trig_Right 10
#define Echo_Right 9

#define Trig_Left 13
#define Echo_Left 12

long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;

HUSKYLENS huskylens;

const int ID0 = 0;  //not learned results. Grey result on HUSKYLENS screen
const int ID1 = 1;  //first learned results. colored result on HUSKYLENS screen
const int ID2 = 2;  //second learned results. colored result on HUSKYLENS screen

#define Red_Color_ID 2
#define Green_Color_ID 3
#define Red_Block_Width_Upper_Threshold 65
#define Red_Block_Width_Lower_Threshold 50
#define Green_Block_Width_Upper_Threshold 70
#define Green_Block_Width_Lower_Threshold 50

int currentColorID = 0;
int currentBlockWidth = 0;

#define Trig_Right 10


bool blueExist = false;
int blueCount = 0;
int lastBlueTime = 0;
int stopp = 0;

void printResult(HUSKYLENSResult result);

void setup() {
  Serial.begin(115200);

  myservo.attach(11);  // attaches the servo on pin 9 to the servo object

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENA, OUTPUT);

  pinMode(Trig_Front, OUTPUT);
  pinMode(Trig_Right, OUTPUT);
  pinMode(Trig_Left, OUTPUT);

  pinMode(Echo_Front, INPUT);
  pinMode(Echo_Right, INPUT);
  pinMode(Echo_Left, INPUT);

  Wire.begin();
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
}

void loop() {

  delay(10);
  // avoidance();
  avoidance4();
  // Forward();
  // delay(3000);
  //a7la msa
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) {
    if (blueExist == true) {
      Serial.println(F("No Thing on screen"));
      blueExist = false;
    }

  } else {

    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);

      if (huskylens.count(ID1) == 0) {
        blueExist = false;
      } else if (blueExist == false && millis() - lastBlueTime > 1000) {
        lastBlueTime = millis();
        blueCount++;
        blueExist = true;
        if (blueCount == 12) stopp = 1;
        Serial.print("Blue Found, count is :");
        Serial.println(blueCount);
      }
    }
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {  //result is a block
    currentColorID = result.ID;
    currentBlockWidth = result.width;
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {  //result is an arrow
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {  //result is unknown.
    Serial.println("Object unknown!");
  }
}
void resetBlockTurning() {

  currentBlockWidth = 0;
  currentColorID = 0;
}

void avoidance() {

  if (stopp == 1) {
    Stop();
    return;
  }
  SonarSensor(Trig_Front, Echo_Front);
  FrontSensor = distance;

  SonarSensor(Trig_Left, Echo_Left);
  LeftSensor = distance;

  SonarSensor(Trig_Right, Echo_Right);
  RightSensor = distance;

  Serial.print("Front Sensor: ");
  Serial.println(FrontSensor);
  Serial.print("Right Sensor: ");
  Serial.println(RightSensor);
  Serial.print("Left Sensor: ");
  Serial.println(LeftSensor);
  if (FrontSensor <= 30) {

    Backward();
    delay(450);
    if (RightSensor < LeftSensor) {
      Left();
      delay(300);

    } else {
      Right();
      delay(300);
    }
  } else if (RightSensor <= 30 || (currentColorID == Red_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Red_Block_Width_Upper_Threshold && currentBlockWidth > Red_Block_Width_Lower_Threshold))) {
    if ((currentColorID == Red_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Red_Block_Width_Upper_Threshold && currentBlockWidth > Red_Block_Width_Lower_Threshold))) {
      Serial.print("Gomaa Hack Red-Left: ");
      Serial.print("currentColorID: ");
      Serial.print(currentColorID);
      Serial.print(", currentBlockWidth : ");
      Serial.println(currentBlockWidth);
      resetBlockTurning();
    }
    Left();
    delay(300);

  } else if (LeftSensor <= 30 || (currentColorID == Green_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Green_Block_Width_Upper_Threshold && currentBlockWidth > Green_Block_Width_Lower_Threshold))) {
    if ((currentColorID == Green_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Green_Block_Width_Upper_Threshold && currentBlockWidth > Green_Block_Width_Lower_Threshold))) {
      Serial.print("Gomaa Hack Green-Right: ");
      Serial.print("currentColorID: ");
      Serial.print(currentColorID);
      Serial.print(", currentBlockWidth : ");
      Serial.println(currentBlockWidth);
      resetBlockTurning();
    }
    Right();
    delay(300);

  } else if (RightSensor >= 35) {
    Right();
    delay(300);
  } else {
    Forward();
    delay(300);
  }

  delay(5);
}

///// الليمت يسطا

#define Front_Limit 90
#define Right_Limit 50
#define Left_Limit 40

#define Front_Trun 190
#define Right_Trun_Upper 55
#define Right_Trun_Lower 45

#define Right_Limit_Upper 50
#define Right_Limit_Lower 40



//avoidance2
void avoidance2() {

  // if (stopp == 1) {
  //   Stop();
  //   return;
  // }


  SonarSensor(Trig_Front, Echo_Front);
  // if (distance < 255) {
  //   FrontSensor = distance;
  // }
  FrontSensor = distance;

  SonarSensor(Trig_Left, Echo_Left);
  // if (distance < 255) {

  //   LeftSensor = distance;
  // }
  LeftSensor = distance;

  SonarSensor(Trig_Right, Echo_Right);
  // if (distance < 255) {
  //   RightSensor = distance;
  // }
  RightSensor = distance;

  Serial.print("Front Sensor: ");
  Serial.println(FrontSensor);
  Serial.print("Right Sensor: ");
  Serial.println(RightSensor);
  Serial.print("Left Sensor: ");
  Serial.println(LeftSensor);

  if (FrontSensor >= Front_Limit) {

    // Backward();
    // delay(450);

    if (RightSensor >= Right_Limit_Upper) {
      Right();
      Serial.println("First If Right :");
      // delay(300);

    } else if (RightSensor < Right_Limit_Lower) {
      Serial.println("First If Left :");
      Left();
      // delay(300);
    } else if ((currentColorID == Red_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Red_Block_Width_Upper_Threshold && currentBlockWidth > Red_Block_Width_Lower_Threshold))) {
      Serial.print("Gomaa Hack Red-Left: ");
      Serial.print("currentColorID: ");
      Serial.print(currentColorID);
      Serial.print(", currentBlockWidth : ");
      Serial.println(currentBlockWidth);
      resetBlockTurning();
      Left();
      delay(100);
    }
    //  else if ((currentColorID == Green_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Green_Block_Width_Upper_Threshold && currentBlockWidth > Green_Block_Width_Lower_Threshold))) {
    //   Serial.print("Gomaa Hack Green-Right: ");
    //   Serial.print("currentColorID: ");
    //   Serial.print(currentColorID);
    //   Serial.print(", currentBlockWidth : ");
    //   Serial.println(currentBlockWidth);
    //   resetBlockTurning();
    //   Right();
    //   delay(100);
    // }



    else {

      Forward();
      // delay(100);
      Serial.println("First If Forward :");
    }
    // Serial.println("First If ");
  }

  ///////////// الدوران يسطا
  else if (FrontSensor < Front_Limit && FrontSensor > 50) {

    // Serial.println("Second If ");
    //  && RightSensor > Right_Trun_Upper && RightSensor <= Right_Trun_Lower

    Serial.println("Second If  Trun Left :");
    Left();
    // delay(300);

  }
  // else if (FrontSensor < 50 && FrontSensor > 30) {
  //   if (currentColorID == Red_Color_ID && (currentBlockWidth != 0 && currentBlockWidth < Red_Block_Width_Upper_Threshold && currentBlockWidth > Red_Block_Width_Lower_Threshold)) {
  //     Serial.print("Gomaa Hack Red-Left: ");
  //     Serial.print("currentColorID: ");
  //     Serial.print(currentColorID);
  //     Serial.print(", currentBlockWidth : ");
  //     Serial.println(currentBlockWidth);
  //     resetBlockTurning();
  //     Right();
  //     delay(200);
  //   }

  //   // Serial.println("Second If ");
  //   //  && RightSensor > Right_Trun_Upper && RightSensor <= Right_Trun_Lower

  //   Serial.println("Second If  Trun Left :");
  //   Left();
  //   // delay(300);

  // }
  else if (currentColorID == Red_Color_ID && FrontSensor > 25) {
    Serial.println("RED BLOCK FORWARD");
    Forward();


  } else if (currentColorID == Red_Color_ID && FrontSensor <= 25) {
    Serial.println("RED BLOCK Right");
    Right();

  } else if (FrontSensor < 10) {
    Serial.println("LESS THAN 10 LEFT");
    Left();

  }
  ////////////
  else {
    Serial.println("Else stop ");
    // // Serial.println("First If Stop :");
    Stop();
    // Forward();
    // delay(450);

    // Left
  }
}  //avoidness2
const int numReadings = 2;  // Number of readings to average





int calculateAverage(int trigPin, int echoPin) {
  // Calculate the average of the readings
  // Read sensor values
  int total = 0;
  for (int i = 0; i < numReadings; i++) {
    SonarSensor(trigPin, trigPin);
    total += distance;
  }
  int average = total / numReadings;
  FrontSensor = average;

  return average;
}




unsigned long stuckTimer = 0;      // Variable to store the time when the robot gets stuck
unsigned long stuckTimeout = 500;  // Timeout period in milliseconds (adjust as needed)
bool isStuck = false;              // Flag to indicate if the robot is stuck
int lowSensorThreshold = 20;       // Threshold value to determine a low sensor reading
int verylowSensorThreshold = 10;
#define Front_Limit 90
#define Right_Limit 60
#define Left_Limit 40

void avoidance4() {
  // Read sensor values
  SonarSensor(Trig_Front, Echo_Front);
  FrontSensor = distance;
  SonarSensor(Trig_Left, Echo_Left);
  LeftSensor = distance;
  SonarSensor(Trig_Right, Echo_Right);
  RightSensor = distance;


  // Calculate the averages

  // FrontSensor=calculateAverage(Trig_Front, Echo_Front);
  // LeftSensor=calculateAverage(Trig_Left, Echo_Left);
  // RightSensor=calculateAverage(Trig_Right, Echo_Right);
  // FrontSensor = distance;
  // SonarSensor(Trig_Left, Echo_Left);
  // LeftSensor = distance;
  // SonarSensor(Trig_Right, Echo_Right);
  // RightSensor = distance;

  Serial.print("Front Sensor: ");
  Serial.println(FrontSensor);
  Serial.print("Right Sensor: ");
  Serial.println(RightSensor);
  Serial.print("Left Sensor: ");
  Serial.println(LeftSensor);

  // Check if the robot is stuck
  if (FrontSensor < lowSensorThreshold || RightSensor < lowSensorThreshold || LeftSensor < lowSensorThreshold && currentColorID != Red_Color_ID) {
    if (!isStuck) {
      // Robot is stuck, set the stuck flag and start the stuck timer
      isStuck = true;
      stuckTimer = millis();
      Serial.println("Robot is stuck!");
    } else if (millis() - stuckTimer >= stuckTimeout) {
      // Stuck timeout has elapsed, try to find a way out
      Serial.println("Stuck timeout elapsed, trying to find a way out!");

      // Add your code here to implement the workaround for getting unstuck
      // if (RightSensor < lowSensorThreshold && LeftSensor < lowSensorThreshold) {
      //   // if (FrontSensor < lowSensorThreshold) {
      //   // Both right and left sensors are stuck, move backward
      //   Backward();
      //   Serial.println("Move Backward");
      //   // delay(100);
      // } else if (RightSensor < lowSensorThreshold) {
      //   // Only right sensor is stuck, turn left
      //   // Backward();

      //   // Serial.println("Move Backward");
      //   // delay(200);
      //   Serial.println("Turn Left");
      //   Left();
      //   delay(100);
      // } else if (LeftSensor < lowSensorThreshold) {
      //   // Only left sensor is stuck, turn right
      //   // Backward();
      //   // Serial.println("Move Backward");
      //   // delay(200);
      //   Serial.println("Turn Right");
      //   Right();
      //   delay(100);
      // }
      Backward();
      delay(300);
      Left();
      delay(200);
      // Reset the stuck flag
      isStuck = false;
    }
  } else {
    // Reset the stuck flag if the sensor readings are above the threshold
    isStuck = false;

    // Continue with obstacle avoidance logic
    if (FrontSensor >= Front_Limit) {
      if (RightSensor <= Right_Limit && LeftSensor >= Left_Limit) {
        // No obstacles detected, move forward
        Forward();
        // delay(300);
        Serial.println("Move Forward");
      } else if (RightSensor >= Right_Limit) {
        // Obstacle detected on the right, turn left
        Right();

        Serial.println("Turn Left");
      } else if (LeftSensor >= Left_Limit) {
        // Obstacle detected on the left, turn right
        Left();
        Serial.println("Turn Right");
      }
    } else {
      // if (currentColorID == Red_Color_ID) {
      //   // Red block detected, turn right around it
      //   Stop();
      //   delay(100);
      //   Right();
      //   delay(100);
      //   Serial.println("Turn Right around Red");
      // }

      if (currentColorID == Green_Color_ID) {
        // Green block detected, turn left around it
        Stop();
        delay(100);
        Left();
        // delay(100);
        Serial.println("Turn Left around Green");
      } else if (LeftSensor >= Left_Limit) {
        // No obstacles or specific color detected, turn left

        Left();
        Serial.println("Turn Left");
      } else {
        Stop();
      }
    }
  }

  delay(100);
}


void SonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;
}

void Forward() {
  if (FrontSensor < verylowSensorThreshold) {
    return;
  }
  myservo.write(90);

  analogWrite(ENA, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void Backward() {

  myservo.write(90);

  analogWrite(ENA, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void Right() {
  if (RightSensor < verylowSensorThreshold) {
    return;
  }
  myservo.write(20);

  analogWrite(ENA, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void Left() {
  if (LeftSensor < verylowSensorThreshold) {
    return;
  }
  myservo.write(160);

  analogWrite(ENA, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void Stop() {

  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
