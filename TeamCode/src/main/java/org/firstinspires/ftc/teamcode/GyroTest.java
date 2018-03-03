package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by raymo on 11/18/2017.
 */

@Autonomous (name = "[Testing] Gyro")
public class GyroTest extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Yaw", getGyroYaw());
            telemetry.update();
        }
    }
}
