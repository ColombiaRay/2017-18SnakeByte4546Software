package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class AutoCryptoBoxTestRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        setInitialAngle();
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        //raiseLift(1500);
        sleep(500);
        moveForwardPID(325,0.001, 0.0000007, 0.5);
        //lowerLift(1500);
        stopMovement();
        sleep(500);
        strafeToCorrectColumnRed();

        /*

        moveStrafe(-0.6, 300);
        //moveStrafeLeftPID(300, 0.0023, 0.0000015, 0.2);
        stopMovement();
        sleep(200);
        moveStrafe(-0.6, 270);
        //moveStrafeLeftPID(400, 0.0023, 0.0000015, 0.2);
        sleep(200);
        stopMovement();
        moveStrafe(-0.6, 270);
        //moveStrafeLeftPID(400, 0.0023, 0.0000015, 0.2);
        setZero();

        sleep(200);
        */
    }
}
