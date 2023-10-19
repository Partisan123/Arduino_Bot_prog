
// #include <Arduino.h>


/*
 * Encoder handler 
 * 
 * This class handles encoder instance creation. 
 * reading and processing of data. Useful for coaxial 
 * encoders based on magnetic steps & hall sensors.  
 * 
 */

class Encoder {

    public: 
    
        int encPinPos =0, encPinNeg =0;

        // variables to store the number of encoder pulses
        volatile signed long temp_stp = 0, encoderCount = 0;
        
        //steps per revolution
        volatile signed int stpsPerRev = 96;
        volatile float temp_rev =  0, d_rev = 0, revolutions = 0;
        
        //rpm calculation interval in ms
        int interval = 200; 
        volatile long currentMillis = 0, previousMillis = 0;
        float rpm = 0; 
        
        Encoder(int encPos, int encNeg){
            this->encPinPos = encPos;
            this->encPinNeg = encNeg;
            init();

        }
        Encoder(){
            this->encPinPos = 0;
            this->encPinNeg = 0;
            init();

        }

        void init(){
        
            pinMode(encPinPos, INPUT_PULLUP);
            pinMode(encPinNeg, INPUT_PULLUP);
            previousMillis = millis();
        }

        void encoderRead() {

            currentMillis = millis();

            if( encoderCount != temp_stp ){

                revolutions = float(encoderCount)/float(stpsPerRev);
                temp_stp = encoderCount;
                
            }

            if (currentMillis - previousMillis > interval) {
            
                d_rev = float(revolutions) - float(temp_rev) ;   
                this->rpm = d_rev*300;
                previousMillis = currentMillis;
                temp_rev = revolutions; 

            }



     
        }
    float getRPM(){
        return rpm;
    }


};