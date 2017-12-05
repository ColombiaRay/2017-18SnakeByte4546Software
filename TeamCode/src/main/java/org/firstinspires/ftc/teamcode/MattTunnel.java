package org.firstinspires.ftc.teamcode;

/**
 * Created by rubenr on 11/9/17.
 */

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class MattTunnel {

    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor inTake;
    private CRServo frontRightTunnel;
    private CRServo backRightTunnel;
    private CRServo frontLeftTunnel;
    private CRServo backLeftTunnel;

    private boolean gateClosed = false;



    public MattTunnel(DcMotor ll, DcMotor rl,  DcMotor iT, CRServo fRT, CRServo bRT, CRServo fLT, CRServo blT) {
        liftLeft         = ll;
        liftRight        = rl;
        inTake           = iT;
        frontRightTunnel = fRT;
        backRightTunnel  = bRT;
        frontLeftTunnel  = fLT;
        backLeftTunnel   = blT;

    }

    public MattTunnel(DcMotor iT, CRServo fRT, CRServo bRT, CRServo fLT, CRServo blT) {
        liftLeft         = null;
        liftRight        = null;
        inTake           = iT;
        frontRightTunnel = fRT;
        backRightTunnel  = bRT;
        frontLeftTunnel  = fLT;
        backLeftTunnel   = blT;

    }

    public void toggleInTake(double stickPower, double rTrigger, double lTrigger) {
        /**
         * Pseudocode:
         *
         *  spin intake motors
         */

        if (stickPower > 0.1) {
            inTake.setPower(-stickPower);
            setBlocks();
        }
        else if(stickPower < -0.1) {
            inTake.setPower(stickPower);
            releaseBlocks();
        }
        else if (rTrigger > 0.1){
            frontLeftTunnel.setPower(-0.05);
            backLeftTunnel.setPower(-0.05);
            frontRightTunnel.setPower(0.05);
            backRightTunnel.setPower(-0.05);
        }
        else if (lTrigger > 0.05){
            frontLeftTunnel.setPower(0.5);
            backLeftTunnel.setPower(0.5);
            frontRightTunnel.setPower(-0.5);
            backRightTunnel.setPower(0.5);
        }
        else {
            inTake.setPower(0.0);
            frontLeftTunnel.setPower(0);
            backLeftTunnel.setPower(0);
            frontRightTunnel.setPower(0);
            backRightTunnel.setPower(0);
        }
    }


    public void setBlocks() {
            frontLeftTunnel.setPower(0.5);
            backLeftTunnel.setPower(0.5);
            frontRightTunnel.setPower(-0.5);
            backRightTunnel.setPower(0.5);
    }

    public void releaseBlocks() {
        frontLeftTunnel.setPower(-0.5);
        backLeftTunnel.setPower(-0.5);
        frontRightTunnel.setPower(0.5);
        backRightTunnel.setPower(-0.5);
    }


    public void manipulateLift(double joyStick) {
        if (joyStick < -0.05) {
            lowerLift(joyStick);
        }
        else if (joyStick > 0.05) {
            raiseLift(joyStick);
        }
        else{
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }
    }

    public void raiseLift(double joy) {
            liftLeft.setPower(-joy); // Edit later
            liftRight.setPower(joy);
    }



    public void lowerLift(double joy) {
            liftLeft.setPower(-joy); // Edit later
            liftRight.setPower(joy);
    }


}
