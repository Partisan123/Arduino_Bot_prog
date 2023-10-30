#include "drive.h"
#include "encoder.h"

unsigned long currentTime,previousTime;
unsigned int elapsedTime;
float error, lastError, cumError, rateError;
float setPoint;


class Motor{
        

        public:
        
            int motPwm_Pin, motDir_Pin1, motDir_Pin2, encoder_Pin1, encoder_Pin2, power_percent;
            bool inverse_dir;
            int pwr = 0;
            Drive motor;
            Encoder encoder;

        

        Motor(int motPwm_Pin, int motDir_Pin1, int motDir_Pin2, bool inverse_dir, int encoder_Pin1, int encoder_Pin2){
            this->motPwm_Pin = motPwm_Pin;
            this->motDir_Pin1 = motDir_Pin1;
            this->motDir_Pin2 = motDir_Pin2;
            this->inverse_dir = inverse_dir;
            this->encoder_Pin1 = encoder_Pin1;
            this->encoder_Pin2 = encoder_Pin2;

            Drive motor(motPwm_Pin, motDir_Pin1, motDir_Pin2, inverse_dir);
            this->motor = motor;

            Encoder encoder(encoder_Pin1, encoder_Pin2);
            this->encoder = encoder;
  
        }

        void drive( int pwr_percent){     // pwr_percent -100 -> 100
            
            this->power_percent = pwr_percent;
            float percent = float(pwr_percent)/100;
            pwr = int(150 * percent);

            if(power_percent < 0)
                motor.driveMotBackward(abs(pwr));        
            else
                motor.driveMotForward(abs(pwr)); 
            

            encoder.encoderRead();     
        }

    private:

        int PID(float rpm_in, float kp , float ki , float kd){

            currentTime = millis();  //get current time
            elapsedTime = (float)(currentTime - previousTime);  //compute time elapsed from previous computation
            
            error = setPoint - rpm_in;                       // determine error
            cumError += error * elapsedTime;                // compute integral
            rateError = (error - lastError)/elapsedTime;   // compute derivative

            int out = kp*error + ki*cumError + kd*rateError;   //PID output               

            lastError = error;                //remember current error
            previousTime = currentTime;      //remember current time

            return out;
        }



};
