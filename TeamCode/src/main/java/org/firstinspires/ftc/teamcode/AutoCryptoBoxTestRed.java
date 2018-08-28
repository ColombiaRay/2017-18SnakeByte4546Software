package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

//@Autonomous
public class AutoCryptoBoxTestRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        waitForStart();
        moveBackward(0.4,55);
        sleep(4000);
        //strafeToCorrectColumnRed();

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
