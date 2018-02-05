package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

//@Autonomous
public class GhettoServoTester extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //jewelHitter.setPosition(0.25);
        leftGlyphClamp.setPosition(0);
        sleep(5000);
        for (double i = 0; i <= 1; i += 0.1){
            //Right clamp closed - 0.2, open - 0.5 left clamp - 0.7 open, 1 closed
            leftGlyphClamp.setPosition(i);
            telemetry.addData("Pos", i);
            telemetry.update();
            sleep(500);
        }
    }
}
