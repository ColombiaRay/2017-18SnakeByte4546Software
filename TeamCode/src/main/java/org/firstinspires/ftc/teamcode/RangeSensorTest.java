package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class RangeSensorTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //strafeToColumnPWithRange(50);
        //strafeToColumnWithRange(48,0.4);
        //strafeToColumnWithRange(65,0.4);
        while(opModeIsActive()){
            telemetry.addData("left", getRangeSensorLeftReading());
            telemetry.addData("right", getRangeSensorRightReading());
            telemetry.update();
        }
        //shootAndStrafe();
    }
}
