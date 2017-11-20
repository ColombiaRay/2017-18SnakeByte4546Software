package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sopa on 10/25/17.
 */
@Autonomous (name = "Blue is the warmest color")
public class JewelHittingAutoBlue extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setAlliance('b');
        telemetry.addData("Blue", "Ready");
        waitForStart();
        //scanImage();
        moveForward(0.2,2000);
        sleep(1000);
        moveStrafe(0.2,500);
        /*
        String cryptoKey = scanImage();
        sleep (5000);
        lowerJewel();
        sleep(2500);
        hitJewel();
        sleep(1000);
        */
        //moveForward(0.2,2000);
        //sleep(1000);
    }
}
