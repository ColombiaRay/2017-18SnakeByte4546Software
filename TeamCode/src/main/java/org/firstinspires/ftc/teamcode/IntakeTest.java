package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/16/2017.
 */
//@Autonomous
public class IntakeTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initialize();
        lowerLift(500);
        sleep(1000);
        raiseLift(500);
    }
}
