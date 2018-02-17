package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous (name = "WRONGJEWEL!RedRight")
public class RPRedRight extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        //Sets to incorrect alliance to hit wrong jewel
        setAlliance('b');
        waitForStart();
        scanImage();
        // scanImage();
        lowerJewel();
        knockJewel();
        sleep(500);
        //moveForwardPID(690,0.0013, 0.0000007, 0.5);
        moveForward(0.35);
        sleep(938);
        setZero();
        telemetry.addData("Angle", getGyroYaw());
        telemetry.update();
        sleep(500);
        turn180();
        scoreGlyph();
    }
}
