package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/5/2017.
 */

@Autonomous
public class JustJewelBlue extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('b');
        initialize();
        waitForStart();
        lowerJewel();
        knockJewel();
        moveBackwardPID(335,0.001, 0.0000007, 0.5);
        stopMovement();
    }
}
