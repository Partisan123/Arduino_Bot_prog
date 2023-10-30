/*
 * Motor handler 
 * 
 * This class handles Motor instance creation. 
 * Motor control, power and direction. 
 * Useful for L298 and similar motor controllers.   
 * 
 */


class Drive {

    public:

       // bool flipDir;
        int pwm_power;
        int motor;
        int dir1;
        int dir2;

        bool flipDir;
        
        Drive(int motorPin, int dirPin1, int dirPin2, bool flipDir){
            this->motor = motorPin;
            this->dir1 = dirPin1;
            this->dir2 = dirPin2;
            this->flipDir = flipDir;
            init();

        }
        Drive(){
            this->motor = 0;
            this->dir1 = 0;
            this->dir2 = 0;
            
            init();

        }


        void init(){
        
            pinMode(motor, OUTPUT);
            pinMode(dir1, OUTPUT);
            pinMode(dir2, OUTPUT);
                
        }

        void driveMotForward(int pwr){  
            
            if(flipDir == true){           
                digitalWrite(dir1, HIGH);
                digitalWrite(dir2, LOW);
            }
            else{    
                digitalWrite(dir1, LOW);
                digitalWrite(dir2, HIGH);
            }

            analogWrite(motor, pwr);

  
        }

        void driveMotBackward(int pwr){  
            
            if(flipDir == true){           
                digitalWrite(dir1, LOW);
                digitalWrite(dir2, HIGH);
            }
            else{    
                digitalWrite(dir1, HIGH);
                digitalWrite(dir2, LOW);
            }

            analogWrite(motor, pwr);

  
        }



};
