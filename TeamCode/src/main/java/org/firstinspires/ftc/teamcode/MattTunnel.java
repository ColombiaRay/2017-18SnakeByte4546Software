package org.firstinspires.ftc.teamcode;

/**
 * Created by rubenr on 11/9/17.
 */
import com.qualcomm.robotcore.hardware.DcMotor;


public class MattTunnel {
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor inTake;


    public MattTunnel(DcMotor ll, DcMotor rl, DcMotor iT) {
        liftLeft     = ll;
        liftRight    = rl;
        inTake       = iT;

    }

    public void toggleInTake(boolean isKeyPressed) {
        /**
         * Pseudocode:
         *
         *  spin intake motors
         */
        //throw new UnsupportedOperationException("Doesn't work yet");
        if(isKeyPressed) {
            inTake.setPower(1.0);
        }
        else {
            inTake.setPower(0.0);
        }
    }

    public void setBlocks() {
        throw new UnsupportedOperationException("Doesn't Work yet");
    }

    public void raiseLift() {
        throw new UnsupportedOperationException("Doesn't Work yet");
    }

    public void lowerLift() {
        throw new UnsupportedOperationException("Doesn't Work yet");
    }

}
