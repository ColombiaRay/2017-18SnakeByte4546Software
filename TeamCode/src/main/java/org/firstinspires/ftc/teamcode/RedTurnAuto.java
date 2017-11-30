package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/30/2017.
 */

@Autonomous
public class RedTurnAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        waitForStart();
        //scanImage();
        moveForwardPID(325,0.001, 0.0000007, 0.5);
        turnRightPID(90, 0.003, 0.000003, 0.12);
    }
}
