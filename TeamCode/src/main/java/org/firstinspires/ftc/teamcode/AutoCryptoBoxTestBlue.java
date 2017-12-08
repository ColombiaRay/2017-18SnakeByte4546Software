package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by raymo on 11/20/2017.
 */

//@Autonomous
public class AutoCryptoBoxTestBlue extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('b');
        initialize();
        setInitialAngle();
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        moveBackwardPID(325,0.001, 0.0000007, 0.5);
        setZero();
        sleep(500);
        pidTurnRight(90);
        pidTurnRight(90);
        strafeToCorrectColumnBlue();
    }
}
