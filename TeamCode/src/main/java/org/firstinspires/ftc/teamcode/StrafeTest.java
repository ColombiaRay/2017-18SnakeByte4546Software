package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/21/2017.
 */

//@Autonomous
public class StrafeTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setStartAngle();
        waitForStart();
        pidTurnRight(180);
        initializeGyro();
        setZero();
        moveStrafe(0.6,500);
    }
}
