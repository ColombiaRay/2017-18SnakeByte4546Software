package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/21/2017.
 */

@Autonomous
public class StraighteningMethodTest extends AutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        setInitialAngle();
        sleep(2000);
        straightenAfterDescent();
        sleep (2000);
        straightenAfterDescent();

    }
}
