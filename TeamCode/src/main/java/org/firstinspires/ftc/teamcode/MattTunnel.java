package org.firstinspires.ftc.teamcode;

/**
 * Created by rubenr on 11/9/17.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class MattTunnel {

    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor inTake;
    private Servo frontRightTunnel;
    private Servo backRightTunnel;
    private Servo frontLeftTunnel;
    private Servo backLeftTunnel;


    public MattTunnel(DcMotor ll, DcMotor rl, DcMotor iT, Servo fRT, Servo bRT, Servo fLT, Servo blT) {
        liftLeft         = ll;
        liftRight        = rl;
        inTake           = iT;
        frontRightTunnel = fRT;
        backRightTunnel  = bRT;
        frontLeftTunnel  = fLT;
        backLeftTunnel   = blT;

    }

    public MattTunnel(DcMotor iT, Servo fRT, Servo bRT, Servo fLT, Servo blT) {
        liftLeft         = null;
        liftRight        = null;
        inTake           = iT;
        frontRightTunnel = fRT;
        backRightTunnel  = bRT;
        frontLeftTunnel  = fLT;
        backLeftTunnel   = blT;

    }

    public void toggleInTake(double isKeyPressed) {
        /**
         * Pseudocode:
         *
         *  spin intake motors
         */

        if (isKeyPressed > 0.1) {
            inTake.setPower(1.0);
            setBlocks(isKeyPressed);
        }
        else if(isKeyPressed < 0.1) {
            inTake.setPower(-1.0);
            releaseBlocks(-isKeyPressed);
        }
        else {
            inTake.setPower(0.0);
            frontLeftTunnel.setPosition(.5);
            backLeftTunnel.setPosition(.5);
            frontRightTunnel.setPosition(.5);
            backRightTunnel.setPosition(.5);
        }
    }


    public void setBlocks(double isKeyPressed) {

            frontLeftTunnel.setPosition(1 - (isKeyPressed/2 + .5));
            backLeftTunnel.setPosition(1 - (isKeyPressed/2 + .5));
            frontRightTunnel.setPosition(isKeyPressed);
            backRightTunnel.setPosition(isKeyPressed);

    }

    public void releaseBlocks(double isKeyPressed) {
        frontLeftTunnel.setPosition(1 - (isKeyPressed/2 + .5));
        backLeftTunnel.setPosition(1 - (isKeyPressed/2 + .5));
        frontRightTunnel.setPosition(isKeyPressed);
        backRightTunnel.setPosition(isKeyPressed);
    }



    public void manipulateLift(double joyStick) {
        if (joyStick < -0.05) {
            lowerLift(joyStick);
        } else if (joyStick > 0.05) {
            raiseLift(joyStick);
        }
    }

    public void raiseLift(double joy) {
        if (joy < 0.05) {
            liftLeft.setPower(-joy); // Edit later
            liftRight.setPower(joy);
        }
    }

    public void lowerLift(double joy) {
        if (joy > 0.05) {
            liftLeft.setPower(joy); // Edit later
            liftRight.setPower(-joy);
        }

    }
}
