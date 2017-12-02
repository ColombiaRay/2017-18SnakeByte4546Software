package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class AutoCryptoBoxTestRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        setAlliance('r');
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        moveForwardPID(345,0.001, 0.0000007, 0.5);
        stopMovement();
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
