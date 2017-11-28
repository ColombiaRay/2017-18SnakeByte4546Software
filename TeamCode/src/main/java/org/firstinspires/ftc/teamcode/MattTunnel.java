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



    public MattTunnel(DcMotor ll, DcMotor rl, DcMotor iT, CRServo fRT, CRServo bRT, CRServo fLT, CRServo blT) {
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

    public void toggleInTake(double stickPower) {
        /**
         * Pseudocode:
         *
         *  spin intake motors
         */

        if (stickPower > 0.1) {
            inTake.setPower(stickPower);
            setBlocks(stickPower);
        }
        else if(stickPower < -0.1) {
            inTake.setPower(-stickPower);
            releaseBlocks(-stickPower);
        }
        else {
            inTake.setPower(0.0);
            frontLeftTunnel.setPower(0);
            //backLeftTunnel.setPosition(.5);
            frontRightTunnel.setPower(0);
            //backRightTunnel.setPosition(.5);
        }
    }


    public void setBlocks(double stickPower) {

            frontLeftTunnel.setPower(0.5);
            //backLeftTunnel.setPosition(1 - (stickPower/2 + .5));
            frontRightTunnel.setPower(-0.5);
            //backRightTunnel.setPosition((stickPower/2 + .5));

    }

    public void releaseBlocks(double stickPower) {
        frontLeftTunnel.setPower(-0.5);
        //backLeftTunnel.setPosition(1 - (stickPower/2 + .5));
        frontRightTunnel.setPower(0.5);
        //backRightTunnel.setPosition((stickPower/2 + .5));
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
