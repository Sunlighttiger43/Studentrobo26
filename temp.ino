#include <Arduino.h>

// We communicate with the Arduino at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 1

// Encoder Pins (THESE WILL NEED TO BE ADJUSTED WHEN SETTING UP)
const byte encoderLpinA = 2; // INTERRUPT MUST BE 2 OR 3
const byte encoderLpinB = 4;
const byte encoderRpinA = 3; // INTERRUPT MUST BE 2 OR 3
const byte encoderRpinB = 5;

// Stepper Pins
// Motor 1 is the two base motors for raising and lowering the tower
// We use a distribution block in order to send pulses to these two motors at the same time
int M1dirpin = 9;  // Motor 1 direction pin
int M1stepPin = 8; // Motor 1 pulse pin
int M1enpin = 10;  // Motor 1 enable pin
// Motor 2 are the motors for closing and opening the tower
// We also use a distribution block here (Possibly, depending on direction independence TO BE TESTED BY KIROLLOS)
int M2dirpin = 12;  // Motor 2 direction pin
int M2stepPin = 11; // Motor 2 pulse pin
int M2enpin = 13;   // Motor 2 enable pin

// Defining other variables
long USduration;
int distance;
byte encoderLPinALast;
byte encoderRPinALast;
long lPulses;
long rPulses;
int duration;
boolean LDirection; // the rotation direction
boolean RDirection;

void setup()
{
    encoderInit(); // Initialise the encoder pins
    stepperInit(); // Initialise the stepper motor pins
    Serial.begin(SERIAL_BAUD);
}

void stepperInit()
{
    // Here we prepare the pins for the steppers
    pinMode(M1dirpin, OUTPUT);
    pinMode(M1stepPin, OUTPUT);
    pinMode(M1enpin, OUTPUT);

    pinMode(M2dirpin, OUTPUT);
    pinMode(M2stepPin, OUTPUT);
    pinMode(M2enpin, OUTPUT);
}

void encoderInit()
{
    // Here we prepare the pins for the encoders, on each interrupt, triggered by the encoder getting tripped,
    // we will run a function to find the direction that the motor is turning to know whether to add or
    // substract a pulse
    pinMode(encoderLpinB, INPUT);
    pinMode(encoderRpinB, INPUT);

    // The interrupt triggers once per pulse, this calls a method to add or subtract pulses, which we use to
    // determine how much the motor has rotated
    attachInterrupt(digitalPinToInterrupt(encoderLpinA), LSpeed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderRpinA), RSpeed, CHANGE);
}

void LSpeed() // Increases or decreases
{
    // Whenever there's a change in value, we check to see if it is a change from LOW to HIGH indicating a pulse
    // When we detect this change we check the direction of the motor using pinB, updating the direction if a
    // change is detected
    int Lstate = digitalRead(encoderLpinA);
    if (encoderLPinALast == LOW && Lstate == HIGH)
    {
        int val = digitalRead(encoderLpinB);
        if (val == LOW && LDirection)
        {
            LDirection = false; // Reverse
            // Serial.print("Left wheel reversing")
        }
        else if (val == HIGH && !LDirection)
        {
            LDirection = true; // Forward
            // Serial.print("Left wheel moving forward")
        }
    }

    encoderLPinALast = Lstate;

    if (!LDirection)
        lPulses++; // Increases or decreases pulses depending on whether the wheel moves forwards or backwards
    else
        lPulses--;
}

void RSpeed()
{
    // Whenever there's a change in value, we check to see if it is a change from LOW to HIGH indicating a pulse
    // When we detect this change we check the direction of the motor using pinB, updating the direction if a
    // change is detected
    int Rstate = digitalRead(encoderRpinA);
    if ((encoderRPinALast == LOW) && Rstate == HIGH)
    {
        int val = digitalRead(encoderRpinB);
        if (val == LOW && RDirection)
        {
            RDirection = false; // Reverse
            // Serial.print("Right wheel reversing")
        }
        else if (val == HIGH && !RDirection)
        {
            RDirection = true; // Forward
            // Serial.print("Right wheel moving forward")
        }
    }
    encoderRPinALast = Rstate;

    if (!RDirection)
        rPulses++;
    else
        rPulses--;
}

void GetL()
{
    Serial.print(lPulses); // Returns the number of left pulses to the python
}

void GetR()
{
    Serial.print(rPulses); // Returns the number of right pulses to the python
}

void TestRot()
{
    // Tests the rotation of the stepper motors
    // Not to be called in competition

    // Enabling and setting the direction of the motor
    digitalWrite(M1enpin, LOW);
    digitalWrite(M1dirpin, HIGH);

    for (int j = 0; j <= 800; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1dirpin, LOW); // Reverse Direction
    for (int j = 0; j <= 800; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1enpin, HIGH);
}

void RaiseArm()
{
    digitalWrite(M1enpin, LOW);
    digitalWrite(M1dirpin, HIGH);

    for (int j = 0; j <= 200; j++)
    // With no microstepping, 200 is the number of steps for a full rotation
    // To raise the arm, we will raise it by one full rotation for now (this can be adjusted, e.g. raise
    // the arm for one box height)
    {
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1enpin, HIGH);
}

void LowerArm()
{
    digitalWrite(M1enpin, LOW);
    digitalWrite(M1dirpin, LOW);

    for (int j = 0; j <= 200; j++)
    // With no microstepping, 200 is the number of steps for a full rotation
    // To raise the arm, we will lower it by one full rotation for now (this can be adjusted, e.g. lower
    // the arm for one box height)
    {
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1enpin, HIGH);
}

// Bunch of built in functions
int read_pin()
{
    while (!Serial.available())
        ;
    int pin = Serial.read();
    return (int)(pin - 'a');
}

void command_read()
{
    int pin = read_pin();
    // Read from the expected pin.
    int level = digitalRead(pin);
    // Send back the result indicator.
    if (level == HIGH)
    {
        Serial.write('h');
    }
    else
    {
        Serial.write('l');
    }
}

void command_analog_read()
{
    int pin = read_pin();
    int value = analogRead(pin);
    Serial.print(value);
}

void command_write(int level)
{
    int pin = read_pin();
    digitalWrite(pin, level);
}

void command_mode(int mode)
{
    int pin = read_pin();
    pinMode(pin, mode);
}

void loop()
{

    duration = 0;
    delay(100);
    // Fetch all commands that are in the buffer
    while (Serial.available())
    {
        int selected_command = Serial.read();
        // Do something different based on what we got:
        switch (selected_command)
        {
        case 'a':
            command_analog_read();
            break;
        case 'r':
            command_read();
            break;
        case 'l':
            command_write(LOW);
            break;
        case 'h':
            command_write(HIGH);
            break;
        case 'i':
            command_mode(INPUT);
            break;
        case 'o':
            command_mode(OUTPUT);
            break;
        case 'p':
            command_mode(INPUT_PULLUP);
            break;
        case 'v':
            Serial.print("SRcustom:");
            Serial.print(FW_VER);
            break;
        case '<':
            GetL();
            break;
        case '>':
            GetR();
            break;
        case 'f':
            TestRot();
            break;
        case 'u':
            RaiseArm();
            break;
        case 'd':
            LowerArm();
            break;
        case 't':
            Serial.print("Hello, the arduino is on");
            break;
        default:
            // A problem here: we do not know how to handle the command!
            // Just ignore this for now.
            break;
        }
        Serial.print("\n");
    }
}
