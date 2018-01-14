package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/21/2017.
 */

//@Autonomous
public class StrafingTest extends AutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Angle", getSpecialGyroYaw());
            telemetry.update();
        }
        //moveStrafe(0.6,300);
        //moveStrafe(-0.6,300);
    }
}
