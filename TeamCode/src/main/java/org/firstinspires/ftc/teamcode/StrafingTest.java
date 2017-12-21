package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/21/2017.
 */

@Autonomous
public class StrafingTest extends AutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        moveStrafe(0.5, 500);
    }
}
