package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

/**
 * Created by sopa on 10/25/17.
 */
@Autonomous (name = "This here is for the strafe encoder test")
/**
 *  This program Strafes for 5 seconds to the right from the robot's perspective
 *  after it completes, we can then measure the increments per inch
 */
public class JewelHittingAutoStrafeTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        double startEnc = getStrafeEncoders();
        double enc = 0.0;

        int initialTime;
        Date timer = new Date();
        initialTime = (int)(System.currentTimeMillis());
        telemetry.addData("Starting Strafe Encoder Value : ", "" + startEnc);
        waitForStart();
        while ((int) (System.currentTimeMillis()) <= initialTime + 1500) {
            enc = getStrafeEncoders();
            telemetry.addData("Strafe Encoder Value : ", "" + enc);
            moveAtAngle(.25,90);


        }
        setZero();
        telemetry.addData("Ending Strafe Encoder Value : ", "" + enc);


    }
}
