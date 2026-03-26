#include <Arduino.h>

// We communicate with the Arduino at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 1

// Encoder Pins (THESE WILL NEED TO BE ADJUSTED WHEN SETTING UP)
const byte encoderLpinA = 22; // INTERRUPT MUST BE 2 OR 3
const byte encoderLpinB = 24;
const byte encoderRpinA = 23; // INTERRUPT MUST BE 2 OR 3
const byte encoderRpinB = 25;

// Stepper Pins
// Motors 1 and 2 are the two base motors for raising and lowering the tower
int MRdirpin = 44;  // Motor 1 direction pin
int MRstepPin = 46; // Motor 1 pulse pin
int MRenpin = 42;   // Motor 1 enable pin
int MLdirpin = 36;  // Motor 2 direction pin (PURPLE)
int MLstepPin = 38; // Motor 2 pulse pin (PINK)
int MLenpin = 34;   // Motor 2 enable pin (BLUE)
// The Following Arm Motors are for opening and closing the clam
int ArmRdirPin = 12;
int ArmRstepPin = 11;
int ArmRenPin = 13;
int ArmLdirPin = 6;
int ArmLstepPin = 5;
int ArmLenPin = 7;

// End stop for calibrating raising and lowering of arm
int BottomLEndStop = 2;
int LowerLEnabled = 1;
int BottomREndStop = 3;
int LowerREnabled = 1;

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
    // encoderInit(); // Initialise the encoder pins
    stepperInit(); // Initialise the stepper motor pins
    endStopInit(); // Initialise the end stop pins
    Serial.begin(SERIAL_BAUD);
}

void endStopInit()
{
    pinMode(BottomLEndStop, INPUT);
    attachInterrupt(digitalPinToInterrupt(BottomLEndStop), BottomLInterrupt, RISING);
    pinMode(BottomREndStop, INPUT);
    attachInterrupt(digitalPinToInterrupt(BottomREndStop), BottomRInterrupt, RISING);
}

void stepperInit()
{
    // Here we prepare the pins for the steppers
    pinMode(MRdirpin, OUTPUT);
    pinMode(MRstepPin, OUTPUT);
    pinMode(MRenpin, OUTPUT);

    pinMode(MLdirpin, OUTPUT);
    pinMode(MLstepPin, OUTPUT);
    pinMode(MLenpin, OUTPUT);

    pinMode(ArmRdirPin, OUTPUT);
    pinMode(ArmRstepPin, OUTPUT);
    pinMode(ArmRenPin, OUTPUT);

    pinMode(ArmLdirPin, OUTPUT);
    pinMode(ArmLstepPin, OUTPUT);
    pinMode(ArmLenPin, OUTPUT);
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

void BottomLInterrupt()
{
    int InterruptAvaliable = LowerLEnabled;
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ((interrupt_time - last_interrupt_time > 200) && (InterruptAvaliable == 1))
    {
        LowerLEnabled = 0;
        Serial.print("Interrupt Received \n");
    };
    last_interrupt_time = interrupt_time;
}

void BottomRInterrupt()
{
    int InterruptAvaliable = LowerREnabled;
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ((interrupt_time - last_interrupt_time > 200) && (InterruptAvaliable == 1))
    {
        LowerREnabled = 0;
        Serial.print("Interrupt Received \n");
    };
    last_interrupt_time = interrupt_time;
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
    digitalWrite(MLenpin, LOW);
    digitalWrite(MRenpin, LOW);
    digitalWrite(MRdirpin, HIGH);
    digitalWrite(MLdirpin, HIGH);

    for (int j = 0; j <= 800; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MLstepPin, HIGH);
        digitalWrite(MRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MLstepPin, LOW);
        digitalWrite(MRstepPin, LOW);
        delayMicroseconds(500);
    }

    delay(500);

    digitalWrite(MLdirpin, LOW);
    digitalWrite(MRdirpin, LOW); // Reverse Direction
    for (int j = 0; j <= 800; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MLstepPin, HIGH);
        digitalWrite(MRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MLstepPin, LOW);
        digitalWrite(MRstepPin, LOW);
        delayMicroseconds(500);
    }
    delay(500);
    digitalWrite(MLenpin, HIGH);
    digitalWrite(MRenpin, HIGH);
}

void RaiseArm(int steps)
{
    digitalWrite(MLenpin, LOW);
    digitalWrite(MRenpin, LOW);
    digitalWrite(MLdirpin, HIGH);
    digitalWrite(MRdirpin, HIGH);

    LowerLEnabled = 1;
    LowerREnabled = 1;
    for (int j = 0; j <= steps; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MLstepPin, HIGH);
        digitalWrite(MRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MLstepPin, LOW);
        digitalWrite(MRstepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(MLenpin, HIGH);
    digitalWrite(MRenpin, HIGH);
}

void LowerArm(int steps)
{
    int j = 0;
    digitalWrite(MLenpin, LOW);
    digitalWrite(MRenpin, LOW);
    digitalWrite(MLdirpin, LOW);
    digitalWrite(MRdirpin, LOW); // Reverse Direction

    while (j <= steps && LowerLEnabled == 1 && LowerREnabled == 1)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MLstepPin, HIGH);
        digitalWrite(MRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MLstepPin, LOW);
        digitalWrite(MRstepPin, LOW);
        delayMicroseconds(500);
        j++;
    }

    while (j <= steps && LowerLEnabled == 1 && LowerREnabled == 0)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MRstepPin, LOW);
        delayMicroseconds(500);
        j++;
    }

    while (j <= steps && LowerLEnabled == 0 && LowerREnabled == 1)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(MLstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(MLstepPin, LOW);
        delayMicroseconds(500);
        j++;
    }

    digitalWrite(MLenpin, HIGH);
    digitalWrite(MRenpin, HIGH);
}

void ClampArm(int steps)
{
    digitalWrite(ArmLenPin, LOW);
    digitalWrite(ArmRenPin, LOW);
    digitalWrite(ArmLdirPin, HIGH);
    digitalWrite(ArmRdirPin, HIGH);

    for (int j = 0; j <= steps; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(ArmLstepPin, HIGH);
        digitalWrite(ArmRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(ArmLstepPin, LOW);
        digitalWrite(ArmRstepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(ArmLenPin, HIGH);
    digitalWrite(ArmRenPin, HIGH);
}

void OpenArm(int steps)
{
    digitalWrite(ArmLenPin, LOW);
    digitalWrite(ArmRenPin, LOW);
    digitalWrite(ArmLdirPin, LOW);
    digitalWrite(ArmRdirPin, LOW);

    for (int j = 0; j <= steps; j++)
    {
        // Sending some number of pulses sent (200 is one full rotation with no microstepping)
        digitalWrite(ArmLstepPin, HIGH);
        digitalWrite(ArmRstepPin, HIGH);
        delayMicroseconds(500); // lower delay = faster
        digitalWrite(ArmLstepPin, LOW);
        digitalWrite(ArmRstepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(ArmLenPin, HIGH);
    digitalWrite(ArmRenPin, HIGH);
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
            RaiseArm(3750);
            break;
        case 'U':
            RaiseArm(200);
            break;
        case 'd':
            LowerArm(3750);
            break;
        case '@':
            LowerArm(200);
            break;
        case 't':
            Serial.print("Hello, the arduino is on");
            break;
        case '?':
            break;
        case '-':
            ClampArm(100);
            break;
        case '+':
            OpenArm(100);
            break;
        default:
            // A problem here: we do not know how to handle the command!
            // Just ignore this for now.
            break;
        }
        Serial.print("\n");
    }
}
