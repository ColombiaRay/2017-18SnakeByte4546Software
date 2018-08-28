package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/18/2017.
 */

//@Autonomous
public class GyroTest extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while(opModeIsActive()){
            testGyro();
        }
    }
}
