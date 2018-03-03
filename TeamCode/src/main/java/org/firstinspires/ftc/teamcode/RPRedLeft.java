package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous (name = "WRONGJEWEL!RedLeft")
public class RPRedLeft extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        setAlliance('b');
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        sleep(500);
        moveForward(0.35);
        sleep(1600);
        setZero();
        pidTurnLeft(90 + getGyroYaw());
        sleep(300);
        moveToRightRedRTurnColumn();
        shootAndStrafe();
    }
}
