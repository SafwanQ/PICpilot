/* 
 * File:   IMU_Test_Main.c
 * Author: Mitch
 *
 * Created on June 15, 2013, 3:40 PM
 */

//Include Libraries
#include <stdio.h>
#include <stdlib.h>
#include "main.h"


//Include Header Files
#include "delay.h"
#include "VN100.h"
#include "InputCapture.h"
#include "OutputCompare.h"
#include "net.h"
#include "StartupErrorCodes.h"
#include "OrientationControl.h"

//Perform random clock things that came with the Sample Code (Keep in code)
_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl

long long time = 0;
long long lastTime = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
    //Temporary Timer Interrupt
    time += 20;
    IFS0bits.T2IF = 0;
    // Clear Timer1 Interrupt Flag
}

int main() {

    //Debug Mode initialize communication with the serial port (Computer)
    if (DEBUG) {
        InitUART1();  
    }
    checkErrorCodes();
    initDataLink();

    // Setpoints (From radio transmitter or autopilot)
    int sp_PitchRate = 0;
    int sp_ThrottleRate = 0;
    int sp_YawRate = 0;
    int sp_RollRate = 0;

    int sp_ComputedPitchRate = 0;
    //int sp_ComputedThrottleRate = 0;
    int sp_ComputedRollRate = 0;
    int sp_ComputedYawRate = 0;

    int sp_Value = 0; //0=Roll, 1= Pitch, 2=Yaw
    int sp_Type = 0; //0 = Saved Value, 1 = Edit Mode
    int sp_Switch = 0;
    int sp_GearSwitch = 0;
    char currentGain = 0;

    float sp_PitchAngle = 0;
    //float sp_YawAngle = 0;
    float sp_RollAngle = 0;

    // System outputs (get from IMU)
    float imuData[3];
    float imu_RollRate = 0;
    float imu_PitchRate = 0;
    float imu_YawRate = 0;

    //IMU integration outputs
    float imu_RollAngle = 0;
    float imu_PitchAngle = 0;
    float imu_YawAngle = 0;

    // Control Signals (Output compare value)
    int control_Roll;
    int control_Pitch;
    int control_Throttle;
    int control_Yaw;

    //System constants
    float maxRollAngle = 60; // max allowed roll angle in degrees
    float maxPitchAngle = 60;
    //float maxYawAngle = 20;

    VN100_initSPI();
    VN100_SPI_Tare(0);
//    getAngleBias();

    if (DEBUG) {
        initIC(0b11111111);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
        UART1_SendString("START OF CODE BEFORE WHILE");
    } else {
        initIC(0b11110001);
        initOC(0b1111); //Initialize only Output Compare 1,2,3 and 4
    }

    while (1) {
        if (DEBUG) {

        }
        /*****************************************************************************
         *****************************************************************************

                                    INPUT CAPTURE

         *****************************************************************************
         *****************************************************************************/

        int* icTimeDiff;
        icTimeDiff = getICValues();


            sp_RollRate = (icTimeDiff[0] - MIDDLE_PWM);
            sp_PitchRate = (icTimeDiff[1] - MIDDLE_PWM);
            sp_ThrottleRate = (icTimeDiff[2]);
            sp_YawRate = (icTimeDiff[3] - MIDDLE_PWM);

            sp_GearSwitch = icTimeDiff[4];
            sp_Type = icTimeDiff[5];
            sp_Value = icTimeDiff[6];
            sp_Switch = icTimeDiff[7];

        if (DEBUG) {


        }

        /*****************************************************************************
         *****************************************************************************

                                    IMU COMMUNICATION

         *****************************************************************************
         *****************************************************************************/
        VN100_SPI_GetRates(0, (float*)&imuData);
        //Outputs in order: Roll,Pitch,Yaw
        imu_RollRate = (imuData[ROLL_RATE]);
//        imu_PitchRate = imuData[PITCH_RATE];
//        imu_YawRate = imuData[YAW_RATE];
        imu_PitchRate = imuData[YAW_RATE];
        imu_YawRate = imuData[PITCH_RATE];

        VN100_SPI_GetYPR(0, &imuData[YAW], &imuData[PITCH], &imuData[ROLL]);
//        imu_YawAngle = imuData[YAW] - angle_zero[YAW];
//        imu_PitchAngle = imuData[PITCH] - angle_zero[PITCH];
//        imu_RollAngle = (imuData[ROLL] - angle_zero[ROLL]);
        imu_YawAngle = imuData[PITCH] - angle_zero[PITCH];
        imu_PitchAngle = imuData[YAW] - angle_zero[YAW];
        imu_RollAngle = (imuData[ROLL] - angle_zero[ROLL]);

//        VN100_SPI_GetMag(0,&imuData);


        if (DEBUG) {
//            char str[20];
//            //sprintf(str,"%f",imuData[0]);
//            UART1_SendString(str);
        }
        /*****************************************************************************
         *****************************************************************************

                                     CONTROL CODE

         *****************************************************************************
         *****************************************************************************/

        if (ORIENTATION) { 
            // If we are using accelerometer based stabalization
            // convert sp_xxxRate to an angle (based on maxAngle)
            // If we are getting input from the controller
            // convert sp_xxxxRate to an sp_xxxxAngle in degrees

            //sp_YawAngle = sp_YawRate / (sp_Range / maxYawAngle);
            sp_RollAngle = -sp_RollRate / (SP_RANGE / maxRollAngle);
            sp_PitchAngle = -sp_PitchRate / (SP_RANGE / maxPitchAngle);

            // Output to servos based on requested angle and actual angle (using a gain value)
            // Set this to sp_xxxxRate so that the gyros can do their thing aswell

            //sp_YawRate = controlSignalAngles(sp_YawAngle, imu_YawAngle, kd_Accel_Yaw, -(SP_RANGE) / (maxYawAngle)) ;
            sp_ComputedYawRate = sp_YawRate;
            sp_ComputedRollRate = controlSignalAngles(sp_RollAngle, imu_RollAngle, ROLL, -(SP_RANGE) / (maxRollAngle)) ;
            sp_ComputedPitchRate = controlSignalAngles (sp_PitchAngle, imu_PitchAngle, PITCH, -(SP_RANGE) / (maxPitchAngle));
        }else{
            sp_ComputedRollRate = sp_RollRate;
            sp_ComputedPitchRate = sp_PitchRate;
            sp_ComputedYawRate = sp_YawRate;
        }

        if (!STABILIZATION) {
            control_Roll = sp_RollRate + MIDDLE_PWM;
            control_Pitch = sp_PitchRate + MIDDLE_PWM;
            control_Yaw = sp_YawRate + MIDDLE_PWM;
            control_Throttle = sp_ThrottleRate;
        } else {

            // CONTROLLER INPUT INTERPRETATION CODE
            // TODO: Add a separate function for this
            if (sp_Switch < 600) {
                unfreezeIntegral();
                if (sp_GearSwitch > 600) {
                    if (sp_Type < 660) {
                        setGain(ROLL_RATE,GAIN_KD,((float) (sp_Value - 520)) / (SP_RANGE) * 40.0);
                        currentGain = (GAIN_KD << 4) + ROLL;
                    } else if (sp_Type > 660 && sp_Type < 699) {
                        setGain(PITCH_RATE,GAIN_KD,((float) (sp_Value - 520)) / (SP_RANGE) * 40.0);
                        currentGain = (GAIN_KD << 4) + PITCH;
                    } else if (sp_Type > 699) {
                        setGain(YAW_RATE,GAIN_KD,((float) (sp_Value - 520)) / (SP_RANGE) * 40.0);//10/0.1
                        currentGain = (GAIN_KD << 4) + YAW;
                    } 


                }
            } else {
                freezeIntegral();
            }

            // Control Signals (Output compare value)
            control_Throttle = sp_ThrottleRate;
            control_Roll = controlSignal(sp_ComputedRollRate / SERVO_SCALE_FACTOR, imu_RollRate,ROLL_RATE);
            control_Pitch = controlSignal(sp_ComputedPitchRate / SERVO_SCALE_FACTOR, imu_PitchRate, PITCH_RATE);
            control_Yaw = controlSignal(sp_ComputedYawRate / SERVO_SCALE_FACTOR, imu_YawRate, YAW_RATE);

        }
        /*****************************************************************************
         *****************************************************************************

                                    OUTPUT COMPARE

         *****************************************************************************
         *****************************************************************************/
        if (DEBUG) {

        }
        if (control_Roll > UPPER_PWM){
            control_Roll = UPPER_PWM;
            // Limits the effects of the integrator, if the output signal is maxed out
            if (getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) * 2 > sp_RollRate - sp_ComputedRollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.1);
            }
        }
        if (control_Roll < LOWER_PWM){
            control_Roll = LOWER_PWM;
            // Limits the effects of the integrator, if the output signal is maxed out
            if (getIntegralSum(ROLL) * getGain(ROLL,GAIN_KI) * 2 < sp_RollRate - sp_ComputedRollRate){
                setIntegralSum(ROLL,getIntegralSum(ROLL)/1.1);
            }
        }
        if (control_Pitch > UPPER_PWM){
            control_Pitch = UPPER_PWM;
            // Limits the effects of the integrator, if the output signal is maxed out
            if (getIntegralSum(PITCH) * getGain(PITCH,GAIN_KI) * 2 > sp_PitchRate - sp_ComputedPitchRate){
                setIntegralSum(PITCH,getIntegralSum(PITCH)/1.1);
            }
        }
        if (control_Pitch < LOWER_PWM){
            control_Pitch = LOWER_PWM;
            // Limits the effects of the integrator, if the output signal is maxed out
            if (getIntegralSum(PITCH) * getGain(PITCH,GAIN_KI) * 2 < sp_PitchRate - sp_ComputedPitchRate){
                setIntegralSum(PITCH,getIntegralSum(PITCH)/1.1);
            }
        }

        if (control_Yaw > UPPER_PWM)
            control_Yaw = UPPER_PWM;
        if (control_Yaw < LOWER_PWM)
            control_Yaw = LOWER_PWM;


        // Sends the output signal to the servo motors
        setPWM(1, control_Roll);
        setPWM(2, control_Pitch);
        setPWM(3, control_Throttle);
        setPWM(4, control_Yaw);

        if (time - lastTime > 200){
            lastTime = time;
            struct telem_block* statusData = createTelemetryBlock();
            statusData->millis = time;
            statusData->lat = 0;
            statusData->lon = 0;
            statusData->pitch = imu_PitchAngle;
            statusData->roll = imu_RollAngle;
            statusData->yaw = imu_YawAngle;
            statusData->pitchRate = imu_PitchRate;
            statusData->rollRate = imu_RollRate;
            statusData->yawRate = imu_YawRate;
            statusData->pitch_gain = getGain(PITCH,currentGain & 0xF0);
            statusData->roll_gain = getGain(ROLL,currentGain & 0xF0);
            statusData->yaw_gain = getGain(YAW,currentGain & 0xF0);
            statusData->pitchSetpoint = sp_PitchRate;
            statusData->rollSetpoint = sp_RollRate;
            statusData->yawSetpoint = sp_YawRate;
            statusData->throttleSetpoint = (int)((float)(control_Throttle - 454)/(890-454)*100);
            statusData->editing_gain = (( sp_GearSwitch > 600) * 0xFF) & (currentGain); //(sp_Switch < 600 &&

            if ( BLOCKING_MODE ) {
                sendTelemetryBlock(statusData);
                destroyTelemetryBlock(statusData);
            } else {

                pushOutboundTelemetryQueue(statusData);
                //statusData->throttleSetpoint = getOutboundQueueLength();
            }
            
        }
        bufferMaintenance();
        asm("CLRWDT");
    }
}
