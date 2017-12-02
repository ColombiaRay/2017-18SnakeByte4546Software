package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/27/2017.
 */

@Autonomous
public class InitialAutoActionsTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        scanImage();
        lowerJewel();
        sleep(500);
        sleep(5000);
    }
}
