#include <Thread.h>
#include <Regexp.h>
#include <Encoder.h>

const byte all_motors_addr = 128;
const byte x_motor_addr = 129;
const byte y_motor_addr = 130;

Thread reportThread = Thread();
Thread controlThread = Thread();

Encoder rotary(20, 21);

enum MotorMode {SHAKE, POSITION, HOME};

class Motor {
  private:
    long _position;
    long _velocity;
    long _acceleration;
    long _min;
    long _max;
    char _name;
    byte _address;
  public:
    Motor(char name, byte address);
    void address_this_motor();

    /* These are Value-Setting, not Commands */
    void setMax(long max);
    void setMin(long min);
    void setPosition(long pos);
    void setVelocity(long vel);
    void setAcceleration(long accel);
    void setMode(MotorMode mode);
    long Motor::getPosition();
    String printReport();

    /* These are Commands */
    void go(long position);
    void goRelative(long delta);
    void acceleration(long acceleration);
    void velocity(long velocity);
};

void Motor::address_this_motor() {
  Serial1.write(this->_address);
}

void Motor::go(long position) {
  this->address_this_motor();
  Serial1.write("MP\n"); // Positional Move
  this->address_this_motor();
  Serial1.write("P=");   // Set target position
  Serial1.write(position);
  Serial1.write('\n');
  this->address_this_motor();
  Serial1.write("G\n");  // Go (execute move)
}

void Motor::goRelative(long delta) {
  this->address_this_motor();
  if(delta>0)
    Serial1.write("P=P+");
  else
    Serial1.write("P=P");
  Serial1.print(delta);
  Serial1.write('\n');
  this->address_this_motor();
  Serial1.write("G\n");
}

void Motor::acceleration(long acceleration) {
  this->address_this_motor();
  Serial1.write("A=");
  Serial1.write(acceleration);
  Serial1.write('\n');
}

void Motor::velocity(long velocity) {
  this->address_this_motor();
  Serial1.write("V=");
  Serial1.write(velocity);
  Serial1.write('\n');
}

void Motor::setMode(MotorMode mode) {
  this->address_this_motor();
  switch (mode) {
    case SHAKE:
      Serial1.println("GOSUB1");
      break;
    case POSITION:
      Serial1.println("GOSUB2");
      break;
    case HOME:
      Serial1.println("GOSUB100");
      break;
  } 
}

Motor::Motor(char name, byte address) {
  this->_name = name;
  this->_min = 0;
  this->_max = 0;
  this->_position = 0;
  this->_address = address;
}

void Motor::setMax(long max) {
  this->_max = max;
}

void Motor::setMin(long min) {
  this->_min = min;
}

void Motor::setPosition(long pos) {
  this->_position = pos;
}

void Motor::setVelocity(long vel) {
  this->_velocity = vel;
}

void Motor::setAcceleration(long accel) {
  this->_acceleration = accel;
}

long Motor::getPosition() {
  return this->_position;
}

String Motor::printReport() {
  Serial.print("Motor Report(");
  Serial.print(String(this->_name));
  Serial.print(") P:");
  Serial.println(this->_position);
}

Motor x_motor("X", x_motor_addr);
Motor y_motor("Y", y_motor_addr);

void reportCallback() {
  //x_motor.printReport();
  //y_motor.printReport();
  Serial.print("  X:");
  Serial.print(x_motor.getPosition());
  Serial.print("  Y:");
  Serial.println(y_motor.getPosition());
  
}

void controlCallback() {
  long delta = rotary.readAndReset();
  if(delta!=0) {
    long move = pow(delta,2);
    if (delta<0)
      move = move * -1;
    y_motor.goRelative(move);
  }
}

void address_motors() {
  Serial1.write("GOSUB42\n");
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(2000);
  //address_motors();
  Serial1.write(all_motors_addr);
  Serial1.write("ZS\n");
  delay(100);
  Serial1.write("RUN\n");
  delay(100);
  x_motor.setMode(HOME);
  y_motor.setMode(HOME);
  //Serial1.write("GOSUB2\n");

  reportThread.onRun(reportCallback);
  reportThread.setInterval(5000);

  controlThread.onRun(controlCallback);
  controlThread.setInterval(100);
}

char *token;
const char *delimiter = ":";

void parseMotors() {
  if (Serial1.available()) {       // If anything comes in Serial1 (pins 0 & 1)
    String dataread = Serial1.readStringUntil(';');  // read it and send it out Serial (USB)
    char comms[100];
    char capture[50];
    dataread.toCharArray(comms, 100);
    Serial.println(comms);
    MatchState ms;
    ms.Target(comms);

    char result = ms.Match ("([XY])([XNPAV]):(%d+)");

    if (result > 0){
      Motor* m;

      // Determine which motor has been specified.
      ms.GetCapture(capture, 0);
      if (String(capture[0]) == "X") {
        m = &x_motor;
      }
      else if (String(capture[0]) == "Y") {
        m = &y_motor;
      }
      else {
        Serial.println("can't find motor");
        Serial.println(capture[0]);
        return;
      }

      long value;
      ms.GetCapture(capture, 2);
      value = atol(capture);


      // Determine which property has been specified, and set the value.
      ms.GetCapture(capture, 1);
      switch(capture[0]) {
        case 'P':
          m->setPosition(value);
          break;
        case 'V':
          m->setVelocity(value);
          break;
        case 'A':
          m->setAcceleration(value);
          break;
        case 'X':
          m->setMax(value);
          break;
        case 'N':
          m->setMin(value);
          break;
      }

    }
    else {
      //Serial.print("Not understood: ");
      //Serial.println(comms);
    }
  }
}



void loop() {

	if(reportThread.shouldRun())
		reportThread.run();

  if(controlThread.shouldRun())
    controlThread.run();

  if (Serial.available()) {        // If anything comes in Serial (USB),
    Serial1.write(Serial.read());  // read it and send it out Serial1 (pins 0 & 1)
  }
  parseMotors();

}
