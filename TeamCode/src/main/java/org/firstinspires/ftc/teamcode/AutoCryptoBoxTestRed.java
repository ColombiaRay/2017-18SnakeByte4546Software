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
        waitForStart();
        moveForwardPID(335,0.001, 0.0000007, 0.5);
        brake();
        sleep(2000);
        straightenAfterDescent();
        stopMovement();
        moveStrafeLeftPID(300, 0.0014, 0.0000015, 0.5);
        setZero();
        sleep(200);
        straightenAfterDescent();
        stopMovement();
        sleep(200);
        moveStrafeLeftPID(400, 0.0014, 0.0000015, 0.5);
        setZero();
        sleep(200);
        straightenAfterDescent();
        stopMovement();
        moveStrafeLeftPID(400, 0.0014, 0.0000015, 0.5);
        setZero();
        sleep(200);
        straightenAfterDescent();
        stopMovement();
    }
}
