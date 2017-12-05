package org.firstinspires.ftc.teamcode;

/**
 * Created by raymo on 12/4/2017.
 */

public class AutoBlueTurn extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        setAlliance('r');
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        sleep(500);
        moveToCorrectColumnBlue();
    }
}
