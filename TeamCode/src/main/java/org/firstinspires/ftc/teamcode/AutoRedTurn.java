package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/4/2017.
 */

@Autonomous
public class AutoRedTurn extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        setAlliance('r');
        waitForStart();
        //
        //
        //scanImage();
        lowerJewel();
        knockJewel();
        sleep(500);
        moveToCorrectColumnRed();
    }
}