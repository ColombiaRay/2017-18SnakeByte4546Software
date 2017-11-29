package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class AutoCryptoBoxTestBlue extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        setAlliance('b');
        waitForStart();
        scanImage();
        //hitJewel();
        moveBackwardPID(280,0.001, 0.0000007, 0.5);
        stopMovement();
        strafeToCorrectColumnBlue();
    }
}
