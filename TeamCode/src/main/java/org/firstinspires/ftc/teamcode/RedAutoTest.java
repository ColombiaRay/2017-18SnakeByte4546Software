package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/21/2017.
 */

//@Autonomous
public class RedAutoTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("r", getRangeSensorRightReading());
            telemetry.update();
        }
    }
}
