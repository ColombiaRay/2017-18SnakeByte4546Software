package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by raymo on 11/18/2017.
 */

//@Autonomous
public class GyroTest extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        double previousTime;
        double timeTally = 0;
        double xVelocity = 0;
        double yVelocity = 0;
        double zVelocity = 0;
        double xPos = 0;
        double yPos = 0;
        Acceleration acc = new Acceleration();
        ElapsedTime timing = new ElapsedTime();
        double time;
        timing.reset();
        while(opModeIsActive()){
            //testGyro();
            acc = imu.getAcceleration();
            telemetry.addData("acc", acc);
            //telemetry.addData("vel",imu.getVelocity());
            //telemetry.addData("pos", imu.getPosition());
            time = timing.seconds();
            timing.reset();
            timeTally += (time);
            if (!(Math.abs(acc.xAccel) <= 0.15)){
                xVelocity += ((time) * acc.xAccel);
            }
            if (!(Math.abs(acc.yAccel) <= 0.15)) {
                yVelocity += ((time) * acc.yAccel);
            }
            if (!(Math.abs(xVelocity) <= 0.15)) {
                xPos += xVelocity * ((time));
            }
            if (!(Math.abs(yVelocity) <= 0.15)) {
                yPos += yVelocity * ((time));
            }
            telemetry.addData("Time diff", time);
            telemetry.addData("time", timeTally);
            telemetry.addData(xVelocity + "", yVelocity);
            telemetry.addData(xPos + "", yPos);
            telemetry.addData("unit", acc.unit);
            telemetry.update();
        }

    }
}
