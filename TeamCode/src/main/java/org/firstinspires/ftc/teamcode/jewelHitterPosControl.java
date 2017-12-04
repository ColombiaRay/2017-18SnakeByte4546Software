package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/2/2017.
 */

@Autonomous
public class jewelHitterPosControl extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (!Double.isNaN(getJewelRange())) {
                raiseJewelQuick();
            }
        }
    }
}
