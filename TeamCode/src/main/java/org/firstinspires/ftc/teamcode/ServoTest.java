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
    Servo relicArm;
    Servo relicGrabber;
    @Override
    public void runOpMode() throws InterruptedException {
        relicArm = hardwareMap.servo.get("relicArm");
        relicGrabber = hardwareMap.servo.get("relicGrabber");
        /*relicArm.setPosition(0);
        sleep(3000);
        relicArm.setPosition(0.5);
        sleep(3000);
        */


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
