package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous
public class GhettoServoTester extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //jewelHitter.setPosition(0.25);
        jewelHitter.setPosition(0.70);
        sleep(5000);
        for (double i = 0; i <= 1; i += 0.1){
            jewelHitter.setPosition(i);
            telemetry.addData("p", i);
            telemetry.update();
            sleep(500);
        }
    }
}
