package org.firstinspires.ftc.teamcode;

/**
 * Created by raymo on 2/16/2018.
 */

//Never run this, use this is a shortcut
public class InterestingMethods extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //PID Control
        moveForwardPID(0,0,0,0);

        //P Control

        //See application in Blue Right and Red Left Auto
        pidTurnLeft(90);
        pidTurnRight(90);

        //P loop to correct angle in side to side motion, along with a method that uses it
        getStrafeCorrection(0);
        strafeToColumnPAltWithRangeRTurn(0);

        //P loop to correct angle in front back motion (along with a method it is applied in), along with a method that uses it
        getLinearCorrection(0);
        moveForwardSt(0,0,0);

        //Jewel Hitting/Color Sensing

        avgColorCompare();

        //Range Sensor Reading

        getRangeSensorLeftReading();
        getRangeSensorRightReading();

        //Range Sensor Alignment

        strafeToColumnPAltWithRangeRTurn(0);


        //Vuforia

        scanImage();



    }
}
