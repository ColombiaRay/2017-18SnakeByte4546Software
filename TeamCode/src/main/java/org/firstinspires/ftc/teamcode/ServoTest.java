package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by raymo on 10/16/2017.
 */
@Autonomous(name = "Servo-urban")
public class ServoTest extends LinearOpMode {
    Servo relic;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        relic = hardwareMap.servo.get("relic");
        /*relicArm.setPosition(0);
        sleep(3000);
        relicArm.setPosition(0.5);
        sleep(3000);
        */
        for (double i = 0; i <= 1; i += 0.1){
            telemetry.addData("Pos", i);
            telemetry.update();
            relic.setPosition(i);
            sleep(500);
        }


        /*
        //lower arm
        relicGrabber.setPosition(0.3);
        sleep(3000);
        //raise arm
        relicGrabber.setPosition(0.7);
        sleep(3000);
        */

        /*
        //Open Relic (continuous mode)
        relicArm.setPosition(0.7);
        sleep(3000);

        //Close Relic (continuous mode)
        relicArm.setPosition(0.3);
        sleep(3000);
        */
        /*relicArm.setPosition(1);
        sleep(3000);
        relicArm.setPosition(0);
        sleep(3000);
        */
    }
}
