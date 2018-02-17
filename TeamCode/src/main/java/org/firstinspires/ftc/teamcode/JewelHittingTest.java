package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous (name = "[Testing] Jewel Hit")
public class JewelHittingTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setAlliance('r');
        waitForStart();
        lowerJewel();
        knockJewel();
    }
}
