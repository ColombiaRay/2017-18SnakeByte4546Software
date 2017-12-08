package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 9/27/2017.
 */


//@Autonomous(name = "Test", group = "Autonomous")
public class DriveTrainTest extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("drivetrain", "init");
        telemetry.update();
        waitForStart();
        for(int i = 0; i < 4; i++) {
            turn( .2, 45 );
            sleep(1000);
        }
    }
}
