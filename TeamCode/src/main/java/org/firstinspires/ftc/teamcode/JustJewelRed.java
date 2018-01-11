package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/5/2017.
 */

@Autonomous (name = "[Outdated]JustJewelRed")
public class JustJewelRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        waitForStart();
        lowerJewel();
        knockJewel();
        moveForwardPID(335,0.001, 0.0000007, 0.5);
        stopMovement();
    }
}
