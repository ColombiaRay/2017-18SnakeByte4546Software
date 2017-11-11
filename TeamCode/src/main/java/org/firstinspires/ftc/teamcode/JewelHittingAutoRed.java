package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sopa on 10/25/17.
 */
@Autonomous (name = "Blood Diamonds")
public class JewelHittingAutoRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("Red", "Ready");
        waitForStart();
        setAlliance('r');
        //grabGlyph();
        lowerJewel();
        sleep(2400);
        hitJewel();
        sleep(1000);
        //Forward probably needs to be less, strafe perhaps more? GJ if you read this
        moveForward(-0.25, 250);
        sleep(2000);
        moveStrafe(-0.5,300);
        setZero();
        //releaseGlyph();
        sleep(1000);
        moveForward(-0.3,100);
    }
}
