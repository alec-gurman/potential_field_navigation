
// GLOBALS

int STBY = 1;  //motor driver enable pin

int AIN1 = 3;  //direction pins for motor a
int AIN2 = 2;  //direction pins for motor a
int PWMA = 5;  //pwm pin for motor a

int BIN1 = 4;  //direction pins for motor b
int BIN2 = 7;  //direction pins for motor b
int PWMB = 6;  //pwm pin for motor b

int kicker_out = 8;  //digital pin to control kicker solenoid

//byte byteRead;     //buffer to read serial string into, may need to be array

int HaveBall = 0;  //variable for ball light sensor
int BallSensorInput = 0;  //analogue values for qrd1114 sensor 10-bit 0-1023
//int BallSensorPin = A0;  //analogue input pin

int pwma_val = 0;
int pwmb_val = 0;
int kick_command = 0;


void setup() {

      //set pin directions
      
      //motor1 left

      pinMode(STBY, OUTPUT);
      pinMode(AIN1, OUTPUT);
      pinMode(AIN2, OUTPUT);
      pinMode(PWMA, OUTPUT);

      //motor2 right
      
      pinMode(BIN1, OUTPUT);
      pinMode(BIN2, OUTPUT);
      pinMode(PWMB, OUTPUT);
      

      //solenoid kicker control pin
      pinMode(kicker_out, OUTPUT);
      //pinMode(A0, INPUT);

      //begin serial connection, wait until connection complete

      Serial.begin(9600);           // set up Serial library at 9600 bps
           while (!Serial) {
          ; // wait for serial port to connect. Needed for native USB
        }
        
}

void loop() {

    //check if there is anything sent over serial

    if (Serial.available() > 0)
    {
                
         String pwma_val_read_string  = Serial.readStringUntil(',');   //read serial string until comma is seen, pass into char array
         String pwmb_val_read_string = Serial.readStringUntil(',');
         String kick_command_string = Serial.readStringUntil(',');

         //toInt gets rid of negative signs, so need to process to ensure negative signs are passed on
        
         if (pwma_val_read_string.charAt(0) == "-")   //if first element is negative
         {
          pwma_val = pwma_val_read_string.toInt() * -1;
         }
         else
         {
          pwma_val = pwma_val_read_string.toInt();
         }



         if (pwmb_val_read_string.charAt(0) == "-")   //if first element is negative
         {
          pwmb_val = pwmb_val_read_string.toInt() * -1;
         }
         else
         {
          pwmb_val = pwmb_val_read_string.toInt();
         }

         kick_command = kick_command_string.toInt();

         //debugging 

         //Serial.print(pwma_val);
         //Serial.print(pwmb_val);
         //Serial.print(kick_command);
           
    }

    //drive motors

    if (pwma_val >= 0)  // left motor forward
    {
      digitalWrite(STBY, HIGH);  //motor driver on
      digitalWrite(AIN1, HIGH);  //pin direction for forward
      digitalWrite(AIN2, LOW);  //pin direction for forward
      analogWrite(PWMA, pwma_val);  //pwm value
    }
    else if (pwma_val < 0 )  //left motor reverse
    {
      digitalWrite(STBY, HIGH);  //motor driver on
      digitalWrite(AIN1, LOW);  //pin direction for reverse
      digitalWrite(AIN2, HIGH);  //pin direction for reverse
      analogWrite(PWMA, pwma_val);  //pwm value
    }
  
    if (pwmb_val >=0 ) //right motor forward
    {
       digitalWrite(STBY, HIGH);  //motor driver on
      digitalWrite(BIN1, HIGH);  //pin direction for forward
      digitalWrite(BIN2, LOW);  //pin direction for forward
      analogWrite(PWMB, pwmb_val);  //pwm value
    }
    else if (pwmb_val < 0)  //right motor reverse
    {
      digitalWrite(STBY, HIGH);  //motor driver on
      digitalWrite(BIN1, LOW);  //pin direction for reverse
      digitalWrite(BIN2, HIGH);  //pin direction for reverse
      analogWrite(PWMB, pwmb_val);  //pwm value
    }

    //command for kicker
  
    if (kick_command == 0)
      {
        //set kick output pin low
       digitalWrite(kicker_out, LOW);
      }
  
    if (kick_command  == 1)
    {
      //set kick output pin high
      digitalWrite(kicker_out, HIGH);
  
    }
    
    //detect if ball is captured, get ir sensor values

    BallSensorInput = analogRead(A0);

    //calculate distance threshold and send have ball status back to phone
 
    if (BallSensorInput > 300)   //dont have ball, arbitrary value, need to calibrate
      {
        HaveBall = 0;
      }
    else                         //have ball, set have ball flag
    {
      HaveBall = 1;
    }

    //send HaveBall status flag back to phone over serial, ONLY EVERY SECOND

    if((millis() % 2000) > 1000)
       {
          Serial.print(HaveBall);

       }

    
    

    


             
}


