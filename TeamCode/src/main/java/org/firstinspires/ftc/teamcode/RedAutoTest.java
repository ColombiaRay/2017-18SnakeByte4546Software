package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/21/2017.
 */

//@Autonomous
public class RedAutoTest extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        waitForStart();
        moveForwardPID(335,0.001, 0.0000007, 0.5);
        setZero();
        sleep(500);
        //straightenAfterDescent();
        setZero();
        moveStrafeLeftPID(600, 0.004, 0.000003, 0.5);
        setZero();
        sleep(500);
        //straightenAfterDescent();
        setZero();
        moveStrafeLeftPID(600, 0.004, 0.000003, 0.5);
        setZero();
        sleep(500);
        //straightenAfterDescent();
        setZero();
        moveStrafeLeftPID(600, 0.004, 0.000003, 0.5);
        setZero();
        sleep(500);
        //straightenAfterDescent();
        setZero();
    }
}
