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

    public void toggleInTake(boolean isKeyPressed) {
        /**
         * Pseudocode:
         *
         *  spin intake motors
         */

        if (isKeyPressed) {
            inTake.setPower(1.0);
        }
        else {
            inTake.setPower(0.0);
        }
    }


    public void setBlocks(boolean isKeyPressed) {
        if (isKeyPressed) {
            frontLeftTunnel.setPosition(0);
            backLeftTunnel.setPosition(0);
            frontRightTunnel.setPosition(1.0);
            backRightTunnel.setPosition(1.0);
        }
    }

    public void releaseBlocks(boolean isKeyPressed) {
        if (isKeyPressed) {
            frontLeftTunnel.setPosition(1.0);
            backLeftTunnel.setPosition(1.0);
            frontRightTunnel.setPosition(0);
            backRightTunnel.setPosition(0);
        }
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
