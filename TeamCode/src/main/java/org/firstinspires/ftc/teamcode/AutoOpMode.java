package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by raymo on 9/24/2017.
 */

public abstract class AutoOpMode extends LinearOpMode {
    double velocity = 0;
    double rotation = 0;
    double strafe = 0;
    VuforiaLocalizer vuforia;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor leftLiftSlide;
    DcMotor rightLiftSlide;
    DcMotor liftMani;
    Servo leftMani;
    Servo rightMani;
    Servo jewelHitter;
    Servo jewelKnocker;
    Servo gate;
    Servo leftArm;
    Servo rightArm;
    Servo leftRelic;
    Servo rightRelic;
    Servo leftGlyphClamp;
    Servo rightGlyphClamp;

    BNO055IMU imu;
    ColorSensor colorFront;
    ColorSensor colorBack;
    int recCount = 0;
    String cryptoboxKey = "None";
    VuforiaLocalizer.Parameters parameters;
    char alliance;
    long closeTime;
    Servo RelicGrabber;
    CRServo frontRightTunnel;
    CRServo backRightTunnel;
    CRServo frontLeftTunnel;
    CRServo backLeftTunnel;
    private double currentTime;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private double pastTime;
    private double previousYaw;
    private double Yaw;
    private double integral;
    private double angleIntegral;
    private double angleDerivative;
    private double error;
    private double previousError;
    private double deltaT;
    private double deltaError;
    private double deltaAngError;
    private int startPos;
    private double startAngle;
    private double angDisplacement;
    private int displacement;
    private double kP;
    private double kI;
    private double kD;
    private double PIDPower;
    private double angError;
    private double previousAngError;
    private double strafeError;
    private double strafeDisplacement;
    private double previousStrafeError;
    private double time;
    private DcMotor inTake;
    private int colorRec;
    private DistanceSensor jewelDistance;
    private double initialAutoAngle;
    private boolean gateOpen;
    private double previousAngle;
    private int revolutions;
    private String jColor;
    Servo relic;
    Servo relicArm;
    MattTunnel tunnel;
    private boolean scored;
    private double timesread;
    private double totalreads;
    ModernRoboticsI2cRangeSensor rangeSensorRight;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    ElapsedTime timer;
    private double heighestRead;
    //private Servo gateServo;


    public void initialize() throws InterruptedException {
        //RelicGrabber = hardwareMap.servo.get("RG");
        relic = hardwareMap.servo.get("relic");
        //FL is 0, BL is 1, FR is 2, BR is 3
        //Jewel is 0
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*leftArm = hardwareMap.servo.get("LRelicArm");
        rightArm = hardwareMap.servo.get("RRelicArm");
        leftRelic = hardwareMap.servo.get("LRelic");
        rightRelic = hardwareMap.servo.get("RRelic");
        leftMani = hardwareMap.servo.get("LMani");
        rightMani = hardwareMap.servo.get("RMani");
        rightMani.setDirection(Servo.Direction.FORWARD);
        leftMani.setDirection(Servo.Direction.REVERSE);
        leftLiftSlide = hardwareMap.dcMotor.get("LSlide");
        rightLiftSlide = hardwareMap.dcMotor.get("RSlide");
        liftMani = hardwareMap.dcMotor.get("liftMani");
        */
        //liftLeft = hardwareMap.dcMotor.get("LLift");
        //liftRight = hardwareMap.dcMotor.get("RLift");
        frontRightTunnel    = hardwareMap.crservo.get("FRT");
        backRightTunnel     = hardwareMap.crservo.get("BRT");
        frontLeftTunnel     = hardwareMap.crservo.get("FLT");
        backLeftTunnel      = hardwareMap.crservo.get("BLT");
        inTake              = hardwareMap.dcMotor.get("intake");
        relicArm            = hardwareMap.servo.get("relicArm");
        rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        tunnel              = new MattTunnel(liftLeft,liftRight, inTake, frontRightTunnel, backRightTunnel, frontLeftTunnel, backLeftTunnel);
        leftGlyphClamp = hardwareMap.servo.get("leftGlyphClamp");
        rightGlyphClamp = hardwareMap.servo.get("rightGlyphClamp");

        jewelHitter = hardwareMap.servo.get("jewelhitter");
        jewelHitter.setDirection(Servo.Direction.REVERSE);

        jewelKnocker = hardwareMap.servo.get("jewelknocker");

        //gate = hardwareMap.servo.get("gate");

        //gyro init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //color sensor init
        colorFront = hardwareMap.colorSensor.get("color");
        //colorBack = hardwareMap.colorSensor.get("color2");
        colorFront.enableLed(true);
        //colorBack.enableLed(true);
        jewelDistance = hardwareMap.get(DistanceSensor.class, "color");
        reportInitialized();
        previousAngle = 0;
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        relicArm.setPosition(1);
        ElapsedTime timer = new ElapsedTime();
    }

    public void resetTime(){
        timer.reset();
    }


    public void initializeGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    public void setPower(double velocity, double rotation, double strafe) throws InterruptedException {
        FL.setPower(velocity - rotation + strafe);
        FR.setPower(-velocity - rotation - strafe);
        BL.setPower(velocity - rotation - strafe);
        BR.setPower(-velocity - rotation + strafe);
    }

    public void setZero() throws InterruptedException{
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    public void reportInitialized() throws InterruptedException{
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        if (alliance == 98) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            });
            telemetry.addData("Blue Auto", "Initialized");
        } else {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
            telemetry.addData("Red Auto", "Initialized");
        }
        telemetry.update();
    }

    public double calculateStrafe(double velocity, double angle) throws InterruptedException {
        return Math.tan(angle) / velocity;
    }

    public double getGyroPitch() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.secondAngle * -1);
    }

    //
    public double getGyroRoll() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.thirdAngle * -1);
    }

    //firstAngle
    public double getGyroYaw() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.firstAngle * -1);
    }

    public double getSpecialGyroYaw() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        double angle = angles.firstAngle * -1;
        if (angle < 0){
            angle = 360 + angle;
        }
        return angle;
    }

    public void testGyro() throws InterruptedException {
        telemetry.addData("Yaw", getGyroYaw());
        telemetry.addData("Roll", getGyroRoll());
        telemetry.addData("Pitch", getGyroPitch());
        telemetry.addData("Position", imu.getPosition());
        telemetry.update();
    }

    public double getGyroYaw(double turn) throws InterruptedException {
        double turnAbs = Math.abs(turn);
        Orientation angles = imu.getAngularOrientation();
        return angles.secondAngle;
//        if (turnAbs > 270 && Math.abs(angles.firstAngle) < 90)
//            return (Math.abs(angles.firstAngle) - (turnAbs - 360));
//        else if (turnAbs < 90 && Math.abs(angles.firstAngle) > 270)
//            return ((Math.abs(angles.firstAngle) - 360) - turnAbs);
//        return (Math.abs(angles.firstAngle) - turnAbs);
    }

    //Refers to starting angle during auto
    public void setInitialAngle() throws InterruptedException {
        initialAutoAngle = getGyroYaw();
    }

    public int getRed(ColorSensor color) throws InterruptedException {
        return color.red();
    }

    public int getBlue(ColorSensor color) throws InterruptedException {
        return color.blue();
    }


    public void lowerJewel() throws InterruptedException {
        jewelKnocker.setPosition(0.55);
        sleep(1000);
        jewelHitter.setPosition(0.45);
        while ((jewelHitter.getPosition() < 0.9) && (opModeIsActive())){
            jewelHitter.setPosition(jewelHitter.getPosition() + 0.01);
            sleep(25);
            idle();
        }
    }

    public void raiseJewel() throws InterruptedException {
        telemetry.addData("Raising", "jewel");
        while ((jewelHitter.getPosition() > 0.45) && (opModeIsActive())){
            jewelHitter.setPosition(jewelHitter.getPosition() - 0.05);
            sleep(70);
            idle();

        }
    }

    public void raiseJewelQuick(){
        jewelHitter.setPosition(1);
    }

    public void setAlliance(char c) throws InterruptedException {
        alliance = c;
        /*
        //I think this part sets the driver station color to alliance color
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        final float[] purple = {276, 63, 78};
        if (alliance == 114) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(purple));
                }
            });
        } else if (alliance == 98) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(purple));
                }
            });
        }
        */
    }


    public int getAvgEncoder() throws InterruptedException {
        return (Math.abs(FL.getCurrentPosition()) + Math.abs(FR.getCurrentPosition())) / 2;
    }

    public void moveForward(double velocity) throws InterruptedException {
        setPower(velocity, 0, 0);
    }

    public void moveBackward(double velocity) throws InterruptedException {
        setPower(-velocity, 0, 0);
    }

    public void moveAtAngle(double velocity, double angle) throws InterruptedException {
        setPower(velocity, 0, calculateStrafe(velocity, angle));
    }

    public void turn(double rotation) throws InterruptedException {
        setPower(0, rotation, 0);
    }

    public void moveForward(double velocity, int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            moveForward(velocity);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void moveBackward(double velocity, int distance) throws InterruptedException {
        int startPos = getAvgEncoder();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive())) {
            moveBackward(velocity);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void moveForwarddMaxTime(double velocity, int distance, double timeMS) throws InterruptedException {
        int startPos = getAvgEncoder();
        double startTime = System.currentTimeMillis();
        while ((Math.abs(getAvgEncoder() - startPos) < distance) && (opModeIsActive()) && (System.currentTimeMillis() - startTime < 1000)) {
            moveForward(velocity);
            telemetry.addData("distance", getAvgEncoder() - startPos);
            telemetry.update();
            idle();
        }
        setZero();
        if (Math.abs(getAvgEncoder() - startPos) > distance + 50) {
            telemetry.addData("overshoot", "fix");
            telemetry.update();
        }
    }

    public void turn(double rotation, double angle) throws InterruptedException {
        double first = getGyroYaw();
        while ((Math.abs(getGyroYaw() - first) < angle) && (opModeIsActive())) {
            turn(rotation);
            idle();
        }
        setZero();
    }

    /*
    public void turnPID(double angle) throws InterruptedException {
        double p = 0.005;
        double i = 0.00001;
        double startTime = System.currentTimeMillis();
        double power = 0;
        double integral = 0;
        double proportion = 0;
        double start = getGyroYaw();
        double current = getGyroYaw();
        while (Math.abs(current - start) < angle && Math.abs(System.currentTimeMillis() - start) < 500){
            integral += Math.abs(current - start) * Math.abs(System.currentTimeMillis() - start) * i;
            proportion = p * Math.abs(current - start);
            power = proportion + integral;
            turn(power);
            idle();
        }
        setZero();
    }
    */

    /*public void scanImage() throws InterruptedException
    {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            if (vuMark == RelicRecoveryVuMark.LEFT){
                telemetry.addData("Key","left");
                telemetry.update();
                cryptoboxKey = "left";
            }
            if (vuMark == RelicRecoveryVuMark.CENTER){
                telemetry.addData("Key","center");
                telemetry.update();
                cryptoboxKey = "center";
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT){
                telemetry.addData("Key","right");
                telemetry.update();
                cryptoboxKey = "right";
            }
        }
        else {
            telemetry.addData("Key", "unknown");
            telemetry.update();
        }

    }
    */

    public void raiseLift(double time){
        double startTime = System.currentTimeMillis();
        liftLeft.setPower(-1);
        liftRight.setPower(1);
        sleep(500);
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void lowerLift(double time){
        double startTime = System.currentTimeMillis();
        liftLeft.setPower(1);
        liftRight.setPower(-1);
        sleep(500);
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void scanImage() throws InterruptedException {
        //Camera Set Up
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Key
        parameters.vuforiaLicenseKey = "AQ1iIdT/////AAAAGZ0U6OKRfU8tpKf9LKl/7DM85y3Wp791rb6q3WwHfYaY53vqKSjAO8wU2FgulWnDt6gLqu9hB33z1reejMz/NyfL8u11QZlMIbimmnP/v4hvoXZWu0p62V9eMG3R2PQ3Z7rZ0qK8HwsQYE/0jmBhTy0D17M4fWpNW64QQnMJqFxq/N1BXm32PEInYDHBYs7WUrHL5oa9xeSSurxUq/TqDpeJwQM+1/GYppdAqzbcM1gi3yzU7JDLdNtOZ6+lbi5uXlU++GnFvQaEXL9uVcnTwMEgBhBng6oOEVoEDXiSUBuZHuMRGZmHfVXSNE3m1UXWyEdPTlMRI5vfEwfsBHmQTmvYr/jJjng3+tBpu85Q1ivo";
        //Use Front Camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        boolean detected = false;
        long scanTime = System.currentTimeMillis();
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.RED);
            }
        });
        while ((System.currentTimeMillis() - scanTime < 5000) && (opModeIsActive()) && (!detected)) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark", "Left");
                    telemetry.update();
                    cryptoboxKey = "left";
                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.GREEN);
                        }
                    });
                    detected = true;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark", "Center");
                    telemetry.update();
                    cryptoboxKey = "center";
                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.GREEN);
                        }
                    });
                    detected = true;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark", "Right");
                    telemetry.update();
                    cryptoboxKey = "right";
                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.GREEN);
                        }
                    });
                    detected = true;
                }
            }
        }
    }

    public double getStrafeCorrection(double desiredAngle) throws InterruptedException {
        //+ means correction is turning right, - is turning left
        double difference = getGyroYaw() - desiredAngle;
        telemetry.addData("Difference", difference);
        telemetry.addData("Power", Range.clip(difference * 0.25/5, -0.25, 0.25));
        telemetry.update();
        return Range.clip(-difference * 0.25/20, -0.25, 0.25);
    }

    public double getStrafeCorrectionSpec(double desiredAngle) throws InterruptedException {
        //+ means correction is turning right, - is turning left
        double difference = getSpecialGyroYaw() - desiredAngle;
        telemetry.addData("Difference", difference);
        telemetry.addData("Power", Range.clip(difference * 0.25/5, -0.25, 0.25));
        telemetry.update();
        return Range.clip(-difference * 0.25/10, -0.25, 0.25);
    }

    /*
    public void moveToDropBlock(String place) throws InterruptedException {
        scanImage();

        if (place.equals("left")) {
            moveForward(.2, 1000);
        }
        else if (place.equals("middle")) {
            moveForward(.2,2000);
        }
        else
            moveForward(.2,2000);
    }
    */


    public void grabGlyph() throws InterruptedException {
        closeTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - closeTime < 2000) && (opModeIsActive())) {
            leftMani.setPosition(1);
            rightMani.setPosition(1);
            idle();
        }
        leftMani.setPosition(0.5);
        rightMani.setPosition(0.5);
    }

    public void releaseGlyph() throws InterruptedException {
        closeTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - closeTime < 2000) && (opModeIsActive())) {
            leftMani.setPosition(0.3);
            rightMani.setPosition(0.3);
            idle();
        }
        leftMani.setPosition(0.5);
        rightMani.setPosition(0.5);
    }

    public int getStrafeEncoders() throws InterruptedException{
        int backLeftEncoderValue = BL.getCurrentPosition();
        int backRightEncoderValue = BR.getCurrentPosition();
        int frontRightEncoderValue = FR.getCurrentPosition();
        int frontLeftEncoderValue = FL.getCurrentPosition();
        telemetry.addData("BL", backLeftEncoderValue);
        telemetry.addData("BR", backRightEncoderValue);

        int avgDiagPosition1 = (Math.abs(backLeftEncoderValue) + Math.abs(frontRightEncoderValue)) / 2;
        //double avgDiagPosition2 = (Math.abs(backRightEncoderValue) + Math.abs(frontLeftEncoderValue)) / 2.0;


        //return (avgDiagPosition1 + avgDiagPosition2) / 2.0;
        //return avgDiagPosition1;
        return backLeftEncoderValue;
    }

    public void moveStrafe(double strafe) throws InterruptedException {
        setPower(0, 0, strafe);
    }


    public void moveStrafe(double strafe, int distance) throws InterruptedException {
        double startStrafe = getStrafeEncoders();
        setStartAngle();
        while ((Math.abs(getStrafeEncoders() - startStrafe) < distance) && (opModeIsActive())) {
            setPower(0, getStrafeCorrection(startAngle),strafe);
            telemetry.addData("distance", getStrafeEncoders() - startStrafe);
            telemetry.update();
            idle();
        }
        setZero();
    }

    //2 Strafing methods, but they will stop either when it reaches a certain encoder position, or a certain time has elapsed
    //Helps motor health
    public void moveStrafeRightMaxTime(double strafe, int distance, double maxTimeMS) throws InterruptedException{
        double startStrafe = getStrafeEncoders();
        setStartAngle();
        double startTime = System.currentTimeMillis();
        while ((Math.abs(getStrafeEncoders() - startStrafe) < distance) && (opModeIsActive()) && (System.currentTimeMillis() - startTime < maxTimeMS)) {
            setPower(0, getStrafeCorrection(startAngle),strafe);
            telemetry.addData("distance", getStrafeEncoders() - startStrafe);
            telemetry.update();
            idle();
        }
        setZero();
    }

    public void moveStrafeLeftMaxTime(double strafe, int distance, double maxTimeMS) throws InterruptedException{
        double startStrafe = getStrafeEncoders();
        setStartAngle();
        double startTime = System.currentTimeMillis();
        while ((Math.abs(getStrafeEncoders() - startStrafe) < distance) && (opModeIsActive()) && (System.currentTimeMillis() - startTime < maxTimeMS)) {
            setPower(0, getStrafeCorrection(startAngle),-strafe);
            telemetry.addData("distance", getStrafeEncoders() - startStrafe);
            telemetry.update();
            idle();
        }
        setZero();
    }

    //Strafe with straightener if the robot turns 180 degrees
    public void moveStrafeSpecial(double strafe, int distance) throws InterruptedException {
        double startStrafe = getStrafeEncoders();
        setStartAngleSpec();
        while ((Math.abs(getStrafeEncoders() - startStrafe) < distance) && (opModeIsActive())) {
            setPower(0, getStrafeCorrectionSpec(startAngle),strafe);
            telemetry.addData("distance", getStrafeEncoders() - startStrafe);
            telemetry.update();
            idle();
        }
        setZero();
    }
    /*
    //Moves the robot the the correct column (values not tested)
    public void moveKey() throws InterruptedException {
        if (cryptoboxKey.equals("left")) {
            moveStrafe(0.5, 600);
        } else if (cryptoboxKey.equals("center")) {
            moveStrafe(0.5, 400);
        } else {
            moveStrafe(0.5, 200);
        }
    }
    */
    //PID Stuff

    public void setStartPos() throws InterruptedException {
        startPos = getAvgEncoder();
        findDisplacement();
    }

    public void setStartAngle() throws InterruptedException {
        startAngle = getGyroYaw();

    }

    public void setStartAngleSpec() throws InterruptedException {
        startAngle = getSpecialGyroYaw();
        telemetry.addData("Start",startAngle);
    }

    public void findDisplacement() throws InterruptedException {
        displacement = Math.abs(getAvgEncoder() - startPos);
        telemetry.addData("Displacement", displacement);
    }

    public void findAngDisplacement() throws InterruptedException {
        telemetry.addData("Yaw", getGyroYaw());
        telemetry.addData("Angle", startAngle);
        angDisplacement = Math.abs(getGyroYaw() - startAngle);
        telemetry.addData("Angular Displacement", angDisplacement);
    }

    public void setInitialError(int goalDistance) {
        error = goalDistance;
    }

    public void findError(int goalDistance) throws InterruptedException {
        previousError = error;
        error = Math.abs(goalDistance - displacement);
    }

    public void setInitialAngError(double goalAngle) throws InterruptedException {
        angError = goalAngle;
    }



    public void findAngError(double goalAngle) throws InterruptedException {
        previousAngError = angError;
        telemetry.addData("Goal", goalAngle);
        angError = (goalAngle - angDisplacement);
        telemetry.addData("Error", angError);
    }

    //Finds error, but has angle can be negative
    public void findTrueAngError(double goalAngle) throws InterruptedException {
        previousAngError = angError;
        angError = goalAngle - angDisplacement;
    }


    public void setKValues(double pValue, double iValue, double dValue) throws InterruptedException{
        kP = pValue;
        kI = iValue;
        kD = dValue;
    }

    public void getPercentTraveled(double goalDistance) throws InterruptedException{
        double percent = displacement / goalDistance * 100;

        if (Math.abs(100 - percent) <= 2) {
            telemetry.addData("Success", percent + "% Accurate");
        } else if (displacement - 100 > 2) {
            telemetry.addData("Too Much", percent + "% Accurate");
        } else if (100 - displacement > 2) {
            telemetry.addData("Too Little", percent + "% Accurate");
        }
        Log.e("Accuracy", displacement + "/" + goalDistance);
        telemetry.addData("Distance", displacement + "/" + goalDistance);
    }

    public void getPercentTurned(double goalAngle) throws InterruptedException{
        double percent = angDisplacement / goalAngle * 100;

        if (Math.abs(100 - percent) <= 2) {
            telemetry.addData("Success", percent + "% Accurate");
        } else if (displacement - 100 > 2) {
            telemetry.addData("Too Much", percent + "% Accurate");
        } else if (100 - displacement > 2) {
            telemetry.addData("Too Little", percent + "% Accurate");
        }
        Log.e("Angular Accuracy", angDisplacement + "/" + goalAngle);
        telemetry.addData("Angle", angDisplacement + "/" + goalAngle);

    }

    public void getStraightness() throws InterruptedException {
        Log.e("Straightness", getGyroYaw() - startAngle + "");
        if (getGyroYaw() - startAngle >= 0) {
            telemetry.addData("Angle", getGyroYaw() - startAngle + " right");
        }
        if (getGyroYaw() - startAngle <= 0) {
            telemetry.addData("Angle", startAngle - getGyroYaw() + " left");
        }

    }

    public void reportSuccess(int distance) throws InterruptedException{
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        if (Math.abs(displacement - distance) <= 5) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.GREEN);
                }
            });
        } else if (Math.abs(displacement - distance) <= 20) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.YELLOW);
                }
            });
        } else {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
        }
    }


    //Proportion Stuff

    public double getProportion() throws InterruptedException{
        telemetry.addData("Proportion", kP * error);
        return kP * error;
    }

    public double getAngProportion() throws InterruptedException{
        telemetry.addData("Proportion", kP * angError);
        return kP * angError;
    }

    public double getStrafeProportion() throws InterruptedException{
        telemetry.addData("Proportion", kP * strafeError);
        return kP * strafeError;
    }

    //Integral Stuff
    public void findDeltaT() throws InterruptedException{
        deltaT = (System.currentTimeMillis() - pastTime);
        pastTime = System.currentTimeMillis();
        time += deltaT;
        telemetry.addData("Time", time);
    }

    public void resetIntegral() throws InterruptedException{
        integral = 0;
        angError = 0;
        pastTime = System.currentTimeMillis();
    }

    public void resetAngIntegral() throws InterruptedException {
        angleIntegral = 0;
    }

    public void tallyIntegral() throws InterruptedException{
        integral += deltaT * error;
    }

    public void tallyAngIntegral() {
        angleIntegral += deltaT * angError;
    }

    public void tallyStrafeIntegral() {
        integral += deltaT * strafeError;
    }

    public double getIntegral() {
        telemetry.addData("Current Integral", integral * kI);
        return integral * kI;
    }

    public double getAngIntegral() {
        telemetry.addData("Current Integral", angleIntegral * kI);
        return angleIntegral * kI;
    }

    //Derivative Stuff

    public void findDeltaError() {
        telemetry.addData("Error", error);
        telemetry.addData("Previous", previousError);
        deltaError = error - previousError;
    }

    public void findDeltaAngError() {
        telemetry.addData("Error", angError);
        telemetry.addData("Previous", previousAngError);
        deltaAngError = angError - previousAngError;
        telemetry.addData("AngError", deltaAngError);
    }

    public void findStrafeDeltaError() {
        deltaError = strafeError - previousStrafeError;
    }

    public double getDerivative() {
        telemetry.addData("Current Derivative", kD * deltaError / deltaT);
        return kD * deltaAngError / deltaT;
    }

    public double getAngDerivative() {
        telemetry.addData("Current Derivative", kD * deltaAngError / deltaT);
        return kD * deltaAngError / deltaT;
    }

    public void stopMovement() throws InterruptedException {
        setPower(0, 0, 0);
        sleep(300);
    }

    public void brake() throws InterruptedException {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopMovement();
        sleep(500);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void moveForwardPID(int distance) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0002, 0.0000001, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveForward(getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        Log.e("Method", "moveForwardPID (1 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
        reportSuccess(distance);
    }

    public void moveForwardPID(int distance, double powerCoefficient) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0002, 0.0000001, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveForward((getProportion() + getIntegral() + getDerivative()) * powerCoefficient);
            idle();
        }
        setZero();
        Log.e("Method", "moveForwardPID (2 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
        reportSuccess(distance);
    }

    //This probably is the best moveForwardPID variant since it is the most adaptable
    public void moveForwardPID(int distance, double p, double i, double d) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(p, i, d);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveForward(getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        Log.e("Method", "moveForwardPID (4 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
        reportSuccess(distance);
    }

    public void moveBackwardPID(int distance) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0002, 0.0000001, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveBackward(getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        Log.e("Method", "moveBackwardPID (1 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveBackwardPID(int distance, double powerCoefficient) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0002, 0.0000001, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveBackward((getProportion() + getIntegral() + getDerivative()) * powerCoefficient);
            idle();
        }
        setZero();
        Log.e("Method", "moveBackwardPID (2 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    //Probably the best version of moveBackwardPID
    public void moveBackwardPID(int distance, double p, double i, double d) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(p, i, d);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveBackward(getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        Log.e("Method", "moveBackwardPID (4 parameters)");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }


    public void turnRightPID(double angle) throws InterruptedException {
        setStartAngle();
        resetAngIntegral();
        setInitialAngError(angle);
        //setKValues(0.002777, 0.000002, 0.1);
        setKValues(0.009, 0.00000002, 0);
        while ((angDisplacement < angle) && (opModeIsActive())) {
            findAngDisplacement();
            telemetry.addData("Yaw", getGyroYaw());
            findAngError(angle);
            findDeltaT();
            findDeltaAngError();
            tallyAngIntegral();
            telemetry.update();
            turn(getAngProportion());
            idle();
        }
        setZero();
        getPercentTurned(angle);
    }

    //Probably the best version of turnRightPID
    public void turnRightPID(double angle, double p, double i, double d) throws InterruptedException {
        setStartAngle();
        resetAngIntegral();
        setInitialAngError(angle);
        //setKValues(0.002777, 0.000002, 0.1);
        setKValues(p, i, d);
        double power;
        while ((angDisplacement < angle) && (opModeIsActive())) {
            findAngDisplacement();
            telemetry.addData("Yaw", getGyroYaw());
            findAngError(angle);
            findDeltaT();
            findDeltaAngError();
            tallyAngIntegral();
            power = getAngProportion() + getAngIntegral() + getAngDerivative();
            telemetry.addData("Power", power);
            telemetry.update();
            turn(power);
            idle();
        }
        setZero();
        Log.e("Method", "turnRightPID (4 parameters)");
        getPercentTurned(angle);
    }

    public void turnLeftPID(double angle) throws InterruptedException {
        setStartAngle();
        resetAngIntegral();
        setInitialAngError(angle);
        setKValues(0.002777, 0.000015, 0.1);
        //setKValues(0.004, 0.000015, 3.0);
        while ((angDisplacement < angle) && (opModeIsActive())) {
            findAngDisplacement();
            telemetry.addData("Yaw", getGyroYaw());
            findAngError(angle);
            findDeltaT();
            findDeltaAngError();
            tallyAngIntegral();
            telemetry.update();
            turn(-1 * (getAngProportion() + getAngIntegral() + getAngDerivative()));
            idle();
        }
        setZero();
        getPercentTurned(angle);
    }

    public void turnLeftPID(double angle, double p, double i, double d) throws InterruptedException {
        setStartAngle();
        resetAngIntegral();
        setInitialAngError(angle);
        setKValues(p, i, d);
        //setKValues(0.004, 0.000015, 3.0);
        while ((angDisplacement < angle) && (opModeIsActive())) {
            findAngDisplacement();
            telemetry.addData("Yaw", getGyroYaw());
            findAngError(angle);
            findDeltaT();
            findDeltaAngError();
            tallyAngIntegral();
            telemetry.update();
            turn(-1 * (getAngProportion() + getAngIntegral() + getAngDerivative()));
            idle();
        }
        setZero();
        Log.e("Method", "turnLeftPID (4 parameters)");
        getPercentTurned(angle);
    }

    public void moveForwardStraight(int distance) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.00015, 0.00000015, 0.25);
        while ((displacement < distance) && (opModeIsActive())) {
            findDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            setPower(getProportion() + getIntegral() + getDerivative(), complexStraighten(), 0);
            idle();
        }
        setZero();
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveBackwardStraight(int distance) throws InterruptedException {
        setStartPos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.00015, 0.00000015, 2);
        while ((displacement < distance) && opModeIsActive()) {
            findDisplacement();
            findError(distance);
            deltaT = System.currentTimeMillis() - pastTime;
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            moveBackward(getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    //Untested
    public double simpleStraighten() throws InterruptedException {
        findAngDisplacement();
        findTrueAngError(0);
        if (angError >= 0.1) {
            return angError * 0.13;
        } else if (angError <= -0.1) {
            return angError * 0.13;
        }
        return 0;
    }

    public double complexStraighten() throws InterruptedException {
        findAngDisplacement();
        findTrueAngError(0);
        angleIntegral += deltaT * angError;
        angleDerivative = (angError - previousAngError) / deltaT;
        return -1 * (0.004 * angError + 0.000015 * angleDerivative + 2 * angleIntegral);

    }

    //Straightens robot after it falls from balance board in auto
    public void straightenAfterDescent() throws InterruptedException {
        telemetry.addData("ang Error", getGyroYaw());
        telemetry.update();
        if (getGyroYaw() > 1) {
            //find good values
            //turnLeftPID(Math.abs(getGyroYaw()), 0.003777, 0.000009, 0.11);
            turn(-0.3, getGyroYaw());
        } else if (getGyroYaw() < -1) {
            //find good values
            //turnRightPID(Math.abs(getGyroYaw()), 0.003777, 0.000009, 0.11);
            turn(0.3, getGyroYaw());
        }
    }

    //PID Strafing

    public void setStartStrafePos() throws InterruptedException{
        startPos = getStrafeEncoders();
        findStrafeDisplacement();
    }

    public void findStrafeDisplacement() throws InterruptedException{
        displacement = Math.abs(getStrafeEncoders() - startPos);
    }

    public void moveStrafeRightPID(int distance) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        //setKValues(0.0008, 0.0000004, 0.5);
        setKValues(0.0014, 0.0000008, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            setPower(0, 0, getProportion() + getIntegral() + getDerivative());
            idle();
        }
        setZero();
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveStrafeRightPID(int distance, double powerCoefficient) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0008, 0.0000004, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            setPower(0, 0, (getProportion() + getIntegral() + getDerivative()) * powerCoefficient);
            idle();
        }
        setZero();
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveStrafeRightPID(int distance, double p, double i, double d) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(p, i, d);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            telemetry.addData("Power", getProportion() + getIntegral() + getDerivative());
            setPower(0, 0, (getProportion() + getIntegral() + getDerivative()));
            idle();
        }
        setZero();
        Log.e("Method", "MoveStrafeRightPID");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveStrafeLeftPID(int distance) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0008, 0.0000004, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            setPower(0, 0, -1 * (getProportion() + getIntegral() + getDerivative()));
            idle();
        }
        setZero();
        Log.e("Method", "MoveStrafeLeftPID");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveStrafeLeftPID(int distance, double powerCoefficient) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(0.0008, 0.0000004, 0.5);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            setPower(0, 0, -1 * (getProportion() + getIntegral() + getDerivative()) * powerCoefficient);
            idle();
        }
        setZero();
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveStrafeLeftPID(int distance, double p, double i, double d) throws InterruptedException {
        setStartStrafePos();
        setStartAngle();
        resetIntegral();
        setInitialError(distance);
        setKValues(p, i, d);
        while ((displacement < distance) && (opModeIsActive())) {
            findStrafeDisplacement();
            findError(distance);
            findDeltaT();
            findDeltaError();
            tallyIntegral();
            telemetry.update();
            telemetry.addData("Power", getProportion() + getIntegral() + getDerivative());
            setPower(0, 0, -(getProportion() + getIntegral() + getDerivative()));
            idle();
        }
        setZero();
        Log.e("Method", "MoveStrafeLeftPID");
        getPercentTraveled(distance);
        getStraightness();
        telemetry.update();
    }

    public void moveToColumn() throws InterruptedException {
        if (cryptoboxKey.equals("left")) {
            moveStrafeLeftPID(2000);
        } else if (cryptoboxKey.equals("center")) {
            moveStrafeLeftPID(1000);
        } else {
            moveStrafeLeftPID(400);
            sleep(1000);
        }
    }

    public void moveToRightRedRTurnColumn() throws InterruptedException{
        if (cryptoboxKey.equals("left")) {
            moveToRightColumnRTurn(57);
        } else if (cryptoboxKey.equals("center")) {
            moveToRightColumnRTurn(42);
        } else {
            moveToRightColumnRTurn(26);
        }
    }

    //Color Sensor + Hitting the Jewel

    /*
    public void hitJewel() throws InterruptedException {
        telemetry.addData("ColorSensor", "Reading");
        telemetry.update();
        String direction = pickDirection();
        if (direction.equals("forward")) {
            //park in safe zone
            moveForwardPID(200,7);
            setZero();
            moveBackwardPID(200,7);
        } else if (direction.equals("backward")) {
            moveBackwardPID(200,7);
            setZero();
            moveForwardPID(200,7);
        }
        raiseJewel();
    }
    */

    public void loadTunnel(int time){
        sleep(500);
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            frontLeftTunnel.setPower(1);
            backLeftTunnel.setPower(1);
            frontRightTunnel.setPower(-1);
            backRightTunnel.setPower(1);
        }
    }

    public void outTunnel(int time) throws InterruptedException
    {
        pidTurnRight(90);
        pidTurnRight(90);

        frontLeftTunnel.setPower(-1);
        backLeftTunnel.setPower(-1);
        frontRightTunnel.setPower(1);
        backRightTunnel.setPower(-1);
        sleep(time);
        frontLeftTunnel.setPower(0);
        backLeftTunnel.setPower(0);
        frontRightTunnel.setPower(0);
        backRightTunnel.setPower(0);
    }

    public void spitItOut(int time, double power) throws InterruptedException
    {
        frontLeftTunnel.setPower(0.5);
        backLeftTunnel.setPower(0.5);
        frontRightTunnel.setPower(-0.5);
        backRightTunnel.setPower(0.5);
        setPower(power, 0, 0);
        sleep(time);
        setPower(0, 0 ,0);
    }

    public void expelGlyphs(int time){
        double startTime = System.currentTimeMillis();
        while((System.currentTimeMillis() - startTime < time) && (opModeIsActive())){
            frontLeftTunnel.setPower(0.5);
            backLeftTunnel.setPower(0.5);
            frontRightTunnel.setPower(-0.5);
            backRightTunnel.setPower(0.5);
        }
        frontLeftTunnel.setPower(0);
        backLeftTunnel.setPower(0);
        frontRightTunnel.setPower(0);
        backRightTunnel.setPower(0);
    }

    public void knockJewel() throws InterruptedException {
        String direction = pickDirection();
        if (direction.equals("forward")) {
            jewelKnocker.setPosition(0.85);
            sleep(500);
        } else if (direction.equals("backward")) {
            jewelKnocker.setPosition(0.25);
            sleep(500);
        }
        raiseJewel();
        sleep(1000);
        jewelKnocker.setPosition(0.25);
        sleep(500);
    }


    public String pickDirection() throws InterruptedException {
        String jewelColor = avgColorCompare();
        if (alliance == 114) {
            if (jewelColor.equals("red")) {
                return "forward";
            } else if (jewelColor.equals("blue")) {
                return "backward";
            }
        } else if (alliance == 98) {
            if (jewelColor.equals("blue")) {
                return "forward";
            } else if (jewelColor.equals("red")) {
                return "backward";
            }
        }
        return "not functioning";
    }

    public String simpleColorDetect() {
        double red = colorFront.red();
        if (red > 18)
            return "red";
        else if (red < 10)
            return "blue";
        else if (colorRec < 4) {
            sleep(500);
            colorRec++;
            simpleColorDetect();
        }
        telemetry.addData("Color", "Not Found");
        telemetry.update();
        return "unknown";
    }

    public String simpleColorCompare() {
        double red = colorFront.red();
        sleep(500);
        double blue = colorFront.blue();

        if (red > blue + 6)
            return "red";
        else if (blue > red + 6)
            return "blue";
        else if (colorRec < 4) {
            colorRec++;
            simpleColorCompare();
        }
        telemetry.addData("Color", "Not Found");
        telemetry.update();
        return "unknown";
    }

    public String avgColorDetect() {
        double red = 0;
        sleep(500);
        for (int i = 0; i < 10; i++) {
            sleep(200);
            red += colorFront.red();
        }
        red /= 10;
        telemetry.addData("Red", red);
        telemetry.update();

        if (red > 18)
            return "red";
        else if (red < 10)
            return "blue";
        else if (colorRec < 1) {
            colorRec++;
            avgColorDetect();
        }
        telemetry.addData("Color", "Not Found");
        telemetry.update();
        return "unknown";
    }

    public void pidTurnRight(double angle) throws InterruptedException {
        double kP = 0.23/90;
        setStartAngle();
        setInitialAngError(angle);
        while ((opModeIsActive()) && (angError > 0.25)){
            findAngDisplacement();
            findAngError(angle);
            telemetry.update();
            turn(kP * angError + 0.23);
        }
        setZero();
    }

    public void pidTurnLeft(double angle) throws InterruptedException {
        double kP = 0.25/90;
        setStartAngle();
        setInitialAngError(angle);
        while ((opModeIsActive()) && (angError > 0.3)){
            findAngDisplacement();
            findAngError(angle);
            telemetry.update();
            turn(-(kP * angError + 0.18));
        }
        setZero();
    }

    public void pidTurnRightAndMove(double angle, double velocity) throws InterruptedException {
        double kP = 0.23/angle;
        setStartAngle();
        setInitialAngError(angle);
        Yaw = getGyroYaw();
        while ((opModeIsActive()) && (angError > 0.25) && (!scored)){
            findAngDisplacement();
            findAngError(angle);
            telemetry.update();
            setPower(velocity, kP * angError + 0.23, 0);
            previousYaw = Yaw;
            Yaw = getGyroYaw();
            telemetry.addData("Yaw Diff", Yaw - previousYaw);
            if (Math.abs(Yaw - previousYaw) < 0.03){
                scored = true;
            }
        }
        setZero();
    }

    public void pidTurnLeftAndMove(double angle, double velocity) throws InterruptedException {
        double kP = 0.23/angle;
        setStartAngle();
        setInitialAngError(angle);
        Yaw = getGyroYaw();
        while ((opModeIsActive()) && (angError > 0.25) && (!scored)){
            findAngDisplacement();
            findAngError(angle);
            telemetry.update();
            setPower(velocity, -(kP * angError + 0.23), 0);
            previousYaw = Yaw;
            Yaw = getGyroYaw();
            telemetry.addData("Yaw Diff", Yaw - previousYaw);
            if (Math.abs(Yaw - previousYaw) < 0.01){
                scored = true;
                telemetry.addData("Boo","Yah");
                telemetry.update();
            }
        }
        setZero();
    }

    public String avgColorCompare() throws InterruptedException {
        double red = 0;
        double blue = 0;
        double redData = 0;
        double blueData = 0;
        sleep(500);
        for (int i = 0; i < 30; i++) {
            sleep(50);
            if (i < 8) {
                redData = (double) (colorFront.red()) / 8;
                red += redData;
                telemetry.addData("ReadingRed", redData * 8);
            } else if (i >= 22) {
                blueData = (double) (colorFront.blue()) / 8;
                blue += blueData;
                telemetry.addData("ReadingBlue", blueData * 8);
            }
        }

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.update();

        if (red > blue + 3) {
            Log.e("Red Certainty", "" + (red - blue));
            return "red";
        } else if (red + 3 < blue) {
            Log.e("Blue Certainty", "" + (red - blue));
            return "blue";
        } else if (colorRec < 2) {
            colorRec++;
            avgColorCompare();
        }

        Log.e("Color Detection", "Failed");
        telemetry.addData("Color", "Not Found");
        telemetry.update();
        return "unknown";
    }


    public String colorCompare() throws InterruptedException {
        double red = 0;
        double blue = 0;
        double redData = 0;
        double blueData = 0;
        sleep(500);
        for (int i = 0; i < 30; i++) {
            sleep(100);
            if (i < 8) {
                redData = (double) (colorFront.red()) / 8;
                red += redData;
            } else if (i >= 22) {
                blueData = (double) (colorFront.blue()) / 8;
                blue += blueData;
            }
        }
        if (red > blue + 3) {
            return "red";
        } else if (red + 3 < blue) {

            return "blue";
        } else if (colorRec < 2) {
            colorRec++;
            colorCompare();
        }
        return "unknown";
    }

//Range Sensor (on jewel hitter)

    public double getJewelRange() {
        double range = jewelDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("Range", range);
        telemetry.update();
        return range;
    }

    public void outputGlyphs(int timeMS, double power){
        frontLeftTunnel.setPower(-1);
        backLeftTunnel.setPower(-1);
        frontRightTunnel.setPower(1);
        backRightTunnel.setPower(-1);
        sleep(timeMS);
        frontLeftTunnel.setPower(0);
        backLeftTunnel.setPower(0);
        frontRightTunnel.setPower(0);
        backRightTunnel.setPower(0);
    }

    public void strafeToRedColumnStrafe() throws InterruptedException {
        moveForwardPID(325,0.001, 0.0000007, 0.5);
        sleep(500);
        if (cryptoboxKey.equals("left")){
            moveStrafe(-0.6, 925);
        }
        else if (cryptoboxKey.equals("center")){
            moveStrafe(-0.6, 600);
        }
        else{
            moveStrafe(-0.6, 330);
        }
        sleep(500);
    }

    public void strafeToBlueColumnStrafe() throws InterruptedException {
        moveBackwardPID(250,0.001, 0.0000007, 0.5);
        sleep(500);
        pidTurnRight(90);
        pidTurnRight(90);
        sleep(500);
        if (cryptoboxKey.equals("right")){
            moveStrafeSpecial(0.6,625);
        }
        else if (cryptoboxKey.equals("center")){
            moveStrafeSpecial(0.6, 390);
        }
        else{
            moveStrafeSpecial(0.6, 140);
        }
        sleep(500);

    }

    public void strafeToRedColumnTurn() throws InterruptedException {
        moveForwardPID(430,0.001, 0.0000007, 0.5);
        sleep(500);
        pidTurnRight(90);
        sleep(500);
        if (cryptoboxKey.equals("left")){
            moveStrafe(-0.6,585);
        }
        else if (cryptoboxKey.equals("center")){
            moveStrafe(-0.6,320);
        }
        sleep(500);

    }

    public void strafeToBlueColumnTurn() throws InterruptedException {
        moveBackwardPID(250,0.001, 0.0000007, 0.5);
        sleep(500);
        pidTurnRight(90);
        sleep(500);
        if (cryptoboxKey.equals("right")){
            moveStrafe(0.6,420);
        }
        else if (cryptoboxKey.equals("center")){
            moveStrafe(0.6,230);
        }
        sleep(500);



    }

    public void fizzleIn(double velocity, double angle) throws InterruptedException {
        int movement = 0;
        scored = false;
        int repetitions = 0;
        int startPos = getAvgEncoder();
        while((repetitions < 5) && (!scored) && (opModeIsActive())){
            telemetry.addData("reps", repetitions);
            telemetry.addData("movement", movement);
            telemetry.update();
            switch (movement){
                case 0:
                    pidTurnRightAndMove(angle, 0.1);
                    movement ++;
                    break;
                case 1:
                    pidTurnLeftAndMove(angle, 0.1);
                    movement ++;
                    break;
                case 2:
                    pidTurnLeftAndMove(angle, 0.1);
                    movement ++;
                    break;
                case 3:
                    pidTurnRightAndMove(angle, 0.1);
                    movement -= 3;
                    repetitions ++;
                    break;
            }
        }
        setZero();
    }

    public void simpleFizzleIn() throws InterruptedException{
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 3 && opModeIsActive()){
            if((int)(2*(System.currentTimeMillis() - start))%2 == 0){
                turn(0.5);
            }
            else{
                turn(-0.5);
            }
        }
        setZero();
    }

    public void executeEndActions() throws InterruptedException{
        moveForwarddMaxTime(0.4,300,2500);
        sleep(300);
        moveBackward(0.5,55);
        sleep(300);
        fizzleIn(0.1,15);
        sleep(300);
        unclampGlyph();
        sleep(3000);
        moveBackward(0.4, 95);
    }

    public void executeEndActionsBlue() throws InterruptedException{
        moveForwarddMaxTime(0.4,300,2500);
        moveBackward(0.5,55);
        sleep(300);
        fizzleIn(0.1,15);
        sleep(300);
        unclampGlyph();
        sleep(3000);
        moveBackward(0.4, 50);
    }



    public void unclampGlyph(){
        leftGlyphClamp.setPosition(0.35);
        rightGlyphClamp.setPosition(0.55);
    }

    public void clampGlyph(){
        leftGlyphClamp.setPosition(0.0);
        rightGlyphClamp.setPosition(0.9);
    }

    public void backUpFromGlyph() throws InterruptedException {
        moveBackward(0.4,150);
    }

    public void backUpFromColumn() throws InterruptedException{
        moveBackward(0.3,50);
    }

    public double getRangeSensorRightReading() throws InterruptedException{
        double rangeValue;
        double reading = rangeSensorRight.getDistance(DistanceUnit.CM);
        while (reading > 500 || Double.isNaN(reading) && opModeIsActive()){
            reading = rangeSensorRight.getDistance(DistanceUnit.CM);
        }
        telemetry.addData("Reading", reading);
        return reading;
    }

    public double getRangeSensorRightReadingFilter() throws InterruptedException{
        double rangeValue;
        double reading = rangeSensorRight.getDistance(DistanceUnit.CM);
        while (reading > 500 || Double.isNaN(reading) && opModeIsActive() && (reading < 3/4*heighestRead)){
            reading = rangeSensorRight.getDistance(DistanceUnit.CM);
        }
        if (reading > heighestRead){
            heighestRead = reading;
        }
        telemetry.addData("Reading", reading);
        return reading;
    }

    public double getRangeSensorLeftReading() throws InterruptedException{
        double reading = rangeSensorLeft.getDistance(DistanceUnit.CM);
        while (reading > 500 || Double.isNaN(reading) && opModeIsActive()){
            reading = rangeSensorLeft.getDistance(DistanceUnit.CM);
        }
        telemetry.addData("Reading", reading);
        return reading;
    }

    public double getRangeSensorLeftReadingFilter() throws InterruptedException{
        double reading = rangeSensorLeft.getDistance(DistanceUnit.CM);
        while (reading > 500 || Double.isNaN(reading) && opModeIsActive() && (reading < 2/3 * totalreads/timesread)){
            reading = rangeSensorLeft.getDistance(DistanceUnit.CM);
        }
        timesread++;
        totalreads += reading;
        telemetry.addData("Reading", reading);
        return reading;
    }

    public void strafeToColumnPIDWithRangeSensor(double distance) throws InterruptedException{
        double rsError;
        double deltaTime;
        double lastTime = System.currentTimeMillis();
        double rsInteg = 0;
        double p = 0.02;
        double i = 0.0001;
        while (getRangeSensorRightReading() < distance && opModeIsActive()){
            rsError = distance - getRangeSensorRightReading();
            deltaTime = System.currentTimeMillis() - lastTime;
            rsInteg += deltaTime * rsError;
            moveStrafe(rsInteg * i + rsError * p);
            lastTime = System.currentTimeMillis();
        }
    }

    public void strafeToColumnWithRange(double distance, double power) throws InterruptedException {
        while (getRangeSensorRightReading() < distance){
            moveStrafe(-power);
            telemetry.addData("moving","moving");
            telemetry.update();
        }
        setZero();
    }

    public void strafeToColumnPWithRange(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while (getRangeSensorRightReading() < distance && opModeIsActive()){
            telemetry.addData("Encoder",getAvgEncoder());
            setPower(0,getStrafeCorrection(0),-(0.35 + 0.4*Math.abs((distance - getRangeSensorRightReading())/distance)));
        }
        int finen = getAvgEncoder();
        telemetry.addData("blah blah", finen - startenc);
        setZero();
        sleep(500);
        if (getRangeSensorRightReading() > distance){
            while (getRangeSensorRightReading() > distance && opModeIsActive()){
                telemetry.addData("correction", "active");
                setPower(0,getStrafeCorrection(0),(0.4));
            }
        }
    }

    public void strafeToColumnPWithRangeSpecial(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while (getRangeSensorRightReading() < distance){
            telemetry.addData("Encoder",getAvgEncoder());
            setPower(0,getStrafeCorrection(0),-(0.35 + 0.4*Math.abs((distance - getRangeSensorRightReading())/distance)));
        }
        setZero();
        sleep(500);
        if (getRangeSensorRightReading() > distance){
            while (getRangeSensorRightReading() > distance){
                telemetry.addData("correction", "active");
                setPower(0,getStrafeCorrection(0),(0.4));
            }
        }
    }

    public void straighten() throws InterruptedException{
        if (getGyroYaw() > 2){
            pidTurnLeft(Math.abs(getGyroYaw()));
        }
        else if (getGyroYaw() < -2){
            pidTurnRight(Math.abs(getGyroYaw()));
        }
    }

    public void straightenB() throws InterruptedException{
        if (getGyroYaw() > 2){
            pidTurnRight(Math.abs(getGyroYaw()));
        }
        else if (getGyroYaw() < -2){
            pidTurnLeft(Math.abs(getGyroYaw()));
        }
    }

    public void strafeToColumnPAltWithRange(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while ((getRangeSensorLeftReading() < distance ) && (opModeIsActive())){
            //setPower(0,getStrafeCorrectionSpec(180),(0.30 + (distance - getRangeSensorLeftReading())/distance));
            setPower(0,getStrafeCorrectionSpec(0),(0.30 + (distance - getRangeSensorLeftReading())/distance));
        }
        int stopenc = getAvgEncoder();
        telemetry.addData("Encoder Displament", stopenc - startenc);
        telemetry.update();
        sleep(3000);
        setZero();
        if (getRangeSensorLeftReading() > distance + 6){
            telemetry.addData("correction", "active");
            telemetry.update();
            while (getRangeSensorLeftReading() > distance && opModeIsActive()){
                setPower(0,getStrafeCorrection(180),(-(0.30 + (distance - getRangeSensorLeftReading())/distance)));
            }
        }
    }

    public void strafeToColumnPAltWithRangeRTurn(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while ((getRangeSensorLeftReading() < distance ) && (opModeIsActive())){
            //setPower(0,getStrafeCorrectionSpec(180),(0.30 + (distance - getRangeSensorLeftReading())/distance));
            setPower(0,getStrafeCorrection(-90),((0.35 )));
        }
        int stopenc = getAvgEncoder();
        setZero();
        telemetry.addData("Encoder Displament", stopenc - startenc);
        telemetry.update();
        sleep(3000);
        if (getRangeSensorLeftReading() > distance + 6){
            telemetry.addData("correction", "active");
            telemetry.update();
            while (getRangeSensorLeftReading() > distance){
                setPower(0,getStrafeCorrection(-90),(-(0.35)));
            }
        }
    }

    public void strafeToColumnPAltGreaterWithRange(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while ((getRangeSensorLeftReading() > distance ) && (opModeIsActive())){
            setPower(0,getStrafeCorrectionSpec(0),-(0.30 + (distance - getRangeSensorLeftReading())/distance));
            //setPower(0,getStrafeCorrectionSpec(180),-(0.30 + (distance - getRangeSensorLeftReading())/distance));
        }
        int stopenc = getAvgEncoder();
        telemetry.addData("Encoder Displament", stopenc - startenc);
        telemetry.update();
        sleep(200);
        setZero();
        if (getRangeSensorLeftReading() > distance + 6){
            telemetry.addData("correction", "active");
            telemetry.update();
            while (getRangeSensorLeftReading() > distance){
                setPower(0,getStrafeCorrection(180),(-(0.30 + (distance - getRangeSensorLeftReading())/distance)));
            }
        }
    }

    public void strafeToColumnPAltGreaterWithRangeRTurn(double distance) throws InterruptedException{
        int startenc = getAvgEncoder();
        while ((getRangeSensorLeftReading() > distance ) && (opModeIsActive())){
            setPower(0,getStrafeCorrection(-90),-((0.35 )));
            //setPower(0,getStrafeCorrectionSpec(180),-(0.30 + (distance - getRangeSensorLeftReading())/distance));
        }
        int stopenc = getAvgEncoder();
        telemetry.addData("Encoder Displament", stopenc - startenc);
        telemetry.update();
        setZero();
        sleep(200);
        if (getRangeSensorLeftReading() > distance + 6){
            telemetry.addData("correction", "active");
            telemetry.update();
            while (getRangeSensorLeftReading() > distance){
                setPower(0,getStrafeCorrection(-





                        90),((0.35 )));
            }
        }
    }

    public void strafeToColumnPAltWithRangeEncoder(double distance) throws InterruptedException{
        int startEnc = getAvgEncoder();
        int overshootEnc = 1800;
        while ((getRangeSensorLeftReading() < distance ) && (opModeIsActive()) && (getAvgEncoder() - startEnc < overshootEnc)){
            setPower(0,getStrafeCorrectionSpec(180),(0.30 + (distance - getRangeSensorLeftReading())/distance));
        }
        setZero();
        if (getRangeSensorLeftReading() > distance + 6){
            telemetry.addData("correction", "active");
            telemetry.update();
            while (getRangeSensorLeftReading() > distance){
                setPower(0,getStrafeCorrection(180),(-(0.30 + (distance - getRangeSensorLeftReading())/distance)));
            }
        }
    }

    public void strafeToColumnPWithRangeSpecialEncoder(double distance) throws InterruptedException{
        int startEnc = getAvgEncoder();
        int overshootEnc = 1800;
        while ((getRangeSensorRightReading() < distance) && (opModeIsActive()) && (getAvgEncoder() - startEnc < overshootEnc)){
            telemetry.addData("Encoder",getAvgEncoder());
            setPower(0,getStrafeCorrection(0),-(0.35 + 0.4*Math.abs((distance - getRangeSensorRightReading())/distance)));
        }
        setZero();
        sleep(500);
        if (getRangeSensorRightReading() > distance){
            while (getRangeSensorRightReading() > distance){
                telemetry.addData("correction", "active");
                setPower(0,getStrafeCorrection(0),(0.4));
            }
        }
    }


    //Method to push the glyph in, seems kinda sucky so probably wont be used, since
    //we feel like it has a high risk of getting the glyph stuck in or touching the clamps
    public void hitIn() throws InterruptedException{
        unclampGlyph();
        sleep(300);
        moveBackward(0.5,75);
        clampGlyph();
        moveForwarddMaxTime(0.5,250,2000);
        sleep(300);
        moveBackward(0.5,75);
    }
    public void releaseBlocks() {
        frontLeftTunnel.setPower(0.5);
        backLeftTunnel.setPower(0.5);
        frontRightTunnel.setPower(-0.5);
        backRightTunnel.setPower(0.5);
    }

    public void shootGlyph(int time) throws InterruptedException {

        releaseBlocks();
    }

    public void stopTunnelServos() throws InterruptedException {

        frontLeftTunnel.setPower(0);
        frontRightTunnel.setPower(0);
        backLeftTunnel.setPower(0);
        backRightTunnel.setPower(0);

    }

    public void moveBackwardMaxTime(double power, double distance, double mTime) throws InterruptedException{
        double startPos = getAvgEncoder();
        double timing = System.currentTimeMillis();
        while ((getAvgEncoder() - startPos < distance) && (opModeIsActive()) && (System.currentTimeMillis() - timing < mTime)){
            moveBackward(power);
        }
        setZero();
    }

    //Drops the block, backs up, and strafes into the column
    public void shootAndStrafe() throws InterruptedException{
        moveBackward(0.3);
        sleep(700);
        setZero();
        shootGlyph(2000);
        sleep(2000);
        spitItOut(200, 0.4);
        sleep(2000);
        stopTunnelServos();
//        pidTurnLeft(90);
//        sleep(300);
//        moveStrafeLeftMaxTime(1,100,500);
//        sleep(300);
//        moveStrafeRightMaxTime(0.6, 100, 1000);
    }

    public void pushIn() throws InterruptedException{
        pidTurnLeft(90);
        sleep(300);
        moveStrafeLeftMaxTime(0.6,700,1000);
        sleep(300);
        moveStrafeRightMaxTime(0.6, 700, 1000);
    }
    public void turn180() throws InterruptedException{
        pidTurnRight(90);
        pidTurnRight(90);
    }

    public void scoreGlyph() throws InterruptedException{
        sleep(300);
        //strafeToColumnPAltWithRange(61);
        //strafeToColumnPAl tWithRange(45);
        moveToRightRedColumn();
        shootAndStrafe();
    }

    public void scoreGlyphB() throws InterruptedException{
        sleep(300);
        //strafeToColumnPAltWithRange(61);
        //strafeToColumnPAltWithRange(45);
        moveToRightBlueColumn();
        shootAndStrafe();
    }

    public void scoreGlyphBRight () throws InterruptedException{
        sleep(300);
        // make new moveToRightBlueColumn with vals to get there for blue right
        shootAndStrafe();
    }

    public void moveToRightRedColumn() throws InterruptedException {
        if (cryptoboxKey.equals("left")){
            strafeToColumnPAltWithRange(82);
        }
        else if (cryptoboxKey.equals("center")){
            strafeToColumnPAltWithRange(67.5);
        }
        else{
            strafeToColumnPAltWithRange(47);
        }
    }

    public void moveToRightBlueColumn() throws InterruptedException {
        if (cryptoboxKey.equals("right")){
            strafeToColumnPWithRange(80);
            //80
        }
        else if (cryptoboxKey.equals("center")){
            strafeToColumnPWithRange(64);
            //63
        }
        else{
            strafeToColumnPWithRange(45);
        }

    }

   // public void moveToColumn()


    public void turnl180() throws InterruptedException{
        pidTurnLeft(90);
        pidTurnLeft(90);
    }

    public void dropAndStrafeV2() throws InterruptedException{
        shootGlyph(3000);
        sleep(500);
        moveBackward(0.5,110);
        pidTurnRight(90);
        sleep(300);
        moveStrafeRightMaxTime(1,400,500);
        sleep(300);
        moveStrafeLeftMaxTime(0.6, 125, 1000);
    }

    //Like the previous one but it turns left
    public void dropAndStrafeAlternate() throws InterruptedException{
        shootGlyph(3000);
        sleep(300);
        moveBackward(0.5,75);
        pidTurnLeft(90);
        sleep(300);
        moveStrafeRightMaxTime(0.6,600,2000);
        sleep(300);
        moveStrafeLeftMaxTime(0.6, 250, 1500);
    }

    //Extremely wild and unlikely to work
    public void golfIn() throws InterruptedException{
        moveStrafeLeftMaxTime(0.4,300,2000);
        sleep(300);
        jewelHitter.setPosition(0.72);
        sleep(400);
        pidTurnLeft(45);
        raiseJewel();
    }

    public void moveToRightColumnRTurn(double goal) throws InterruptedException {
        if (getRangeSensorLeftReading() < goal){
            strafeToColumnPAltWithRangeRTurn(goal);
        }
        else if (getRangeSensorLeftReading() > goal){
            strafeToColumnPAltGreaterWithRangeRTurn(goal);
        }
    }

    public void moveToRightColumnRBTurn(double goal) throws InterruptedException {
        if (getRangeSensorRightReading() < goal){
            strafeToColumnPAltWithRange(goal);
        }
        else if (getRangeSensorRightReading() > goal){
            strafeToColumnPAltWithRange(goal);
        }
    }


//Gate on tunnel

    /*
    public void closeGate() {
        gate.setPosition(0.35);
    }

    public void raiseGate(){
        gate.setPosition(0.7);
    }
    */

}

/*
                    Code Graveyard (R.I.P.)
public String chooseColor(char c) throws InterruptedException {
        //hitting blue
        if (c == 114) {
            if (getBlue(colorFront) < getBlue(colorBack)) {
                telemetry.addData("hit", "forwards");
                telemetry.update();
                return "forwards";
            } else if (getBlue(colorFront) > getBlue(colorBack)) {
                telemetry.addData("hit", "backwards");
                telemetry.update();
                return "backwards";
            } else {
                sleep(1000);
                if (recCount < 2)
                    recCount++;
                chooseColor(c);
                telemetry.addData("ColorSensors", "broken");
                telemetry.update();
                return "broken";

            }
        }
        //hitting red
        if (c == 98) {
            if (getRed(colorFront) < getRed(colorBack)) {
                telemetry.addData("hit", "forwards");
                telemetry.update();
                return "forwards";
            } else if (getRed(colorFront) > getRed(colorBack)) {
                telemetry.addData("hit", "backwards");
                telemetry.update();
                return "backwards";
            } else {
                sleep(1000);
                if (recCount < 2)
                    recCount++;
                chooseColor(c);
                telemetry.addData("ColorSensors", "broken");
                telemetry.update();
                return "broken";
            }
        }
        return "broken";
    }

    public String choseOneColor(char c) throws InterruptedException {
        //hitting blue
        if (c == 114) {
            if (getRed(colorBack) < getBlue(colorBack)) {
                telemetry.addData("hit", "backwards");
                telemetry.update();
                return "forwards";
            } else if (getRed(colorBack) > getBlue(colorBack)) {
                telemetry.addData("hit", "forwards");
                telemetry.update();
                return "backwards";
            } else {
                sleep(1000);
                if (recCount < 2)
                    recCount++;
                chooseColor(c);
            }
        }
        //hitting red
        if (c == 98) {
            if (getRed(colorBack) < getBlue(colorBack)) {
                telemetry.addData("hit", "forwards");
                telemetry.update();
                return "forwards";
            } else if (getRed(colorBack) > getBlue(colorBack)) {
                telemetry.addData("hit", "backwards");
                telemetry.update();
                return "backwards";
            } else {
                sleep(1000);
                if (recCount < 2)
                    recCount++;
                chooseColor(c);
            }
        }
        return "broken";
    }

    public void hitJewel() throws InterruptedException {
        telemetry.addData("ColorSensor", "Reading");
        telemetry.update();
        String direction = pickDirection();
        if (direction.equals("forward")) {
            //park in safe zone
            moveForwardPID(365,0.001, 0.0000007, 0.5);
        } else if (direction.equals("backward")) {
            moveBackwardPID(120,0.001, 0.0000007, 0.5);
            setZero();
            moveForwardPID(365,0.001, 0.0000007, 0.5);
        }
    }

    public void hitJewelTurn() throws InterruptedException {
        telemetry.addData("ColorSensor", "Reading");
        telemetry.update();
        sleep(300);
        String direction = pickDirection();
        if (direction.equals("forward")) {
            //park in safe zone
            turnLeftPID(20);
            setZero();
            turnRightPID(20);
        } else if (direction.equals("backward")) {
            turnRightPID(20);
            setZero();
            turnLeftPID(20);
        }
        raiseJewel()
    }


    public void strafeToCorrectColumnRed() throws InterruptedException {
        if (cryptoboxKey.equals("left")) {
            Values for League Meet 2
            moveStrafe(-0.6, 220);
            setZero();
            moveStrafe(-0.6, 305);
            setZero();
            moveStrafe(-0.6, 305);
            setZero();
        } else if (cryptoboxKey.equals("center")) {
            moveStrafe(-0.6, 220);
            setZero();
            moveStrafe(-0.6, 305);
            setZero();
                } else {
                moveStrafe(-0.6, 220);
                setZero();
                }
                expelGlyphs(4500);
                moveForward(0.5, 100);
                moveBackward(0.5,100);
                //moveForwardPID(100, 0.006, 0.0000012, 0.5);
                //moveForwardPID(100, 0.002, 0.0000007, 0.5);
                //moveBackwardPID(90, 0.004, 0.0000012, 0.5);
                }

public void strafeToCorrectColumnBlue() throws InterruptedException {
        if (cryptoboxKey.equals("right")) {
        moveStrafe(0.6, 200);
        setZero();
        moveStrafe(0.6, 305);
        setZero();
        moveStrafe(0.6, 305);
        setZero();
        } else if (cryptoboxKey.equals("center")) {
        moveStrafe(0.6, 200);
        setZero();
        moveStrafe(0.6, 305);
        setZero();
        } else {
        moveStrafe(0.6, 200);
        setZero();
        }
        expelGlyphs(4500);
        moveForward(0.5, 100);
        moveBackward(0.5,100);
        //moveForwardPID(100, 0.006, 0.0000012, 0.5);
        //moveForwardPID(100, 0.002, 0.0000007, 0.5);
        //moveBackwardPID(90, 0.004, 0.0000012, 0.5);
        }

public void moveToCorrectColumnRed() throws InterruptedException {
        cryptoboxKey = "right";
        if (cryptoboxKey.equals("right")) {
        moveBackwardPID(305,0.001, 0.0000007, 0.5);
        } else if (cryptoboxKey.equals("center")) {
        moveBackwardPID(325,0.001, 0.0000007, 0.5);
        } else {
        moveBackwardPID(345,0.001, 0.0000007, 0.5);
        }
        setZero();
        pidTurnLeft(90);
        expelGlyphs(4500);
        moveForward(0.5, 100);
        moveBackward(0.5,100);
        //moveForwardPID(100, 0.006, 0.0000012, 0.5);
        //moveForwardPID(100, 0.002, 0.0000007, 0.5);
        //moveBackwardPID(90, 0.004, 0.0000012, 0.5);
        }

public void moveToCorrectColumnBlue() throws InterruptedException {
        if (cryptoboxKey.equals("right")) {
        moveForwardPID(305,0.001, 0.0000007, 0.5);
        } else if (cryptoboxKey.equals("center")) {
        moveForwardPID(325,0.001, 0.0000007, 0.5);
        } else {
        moveForwardPID(345,0.001, 0.0000007, 0.5);
        }
        setZero();
        pidTurnRight(90);
        expelGlyphs(4500);
        moveForward(0.5, 100);
        moveBackward(0.5,100);
        //moveForwardPID(100, 0.006, 0.0000012, 0.5);
        //moveForwardPID(100, 0.002, 0.0000007, 0.5);
        //moveBackwardPID(90, 0.004, 0.0000012, 0.5);
        }
 */

/*Code for EN Pictures
Week 4
public void getJewelColorOneSensor() throws InterruptedException {
        if (colorFront.blue() > colorFront.red()){
            //jColor refers to color of jewel in the front
            jColor = "blue";
        }
        else if (colorFront.red() > colorFront.blue()) {
            jColor = "red";
        }
        else if (recCount < 3){
            sleep(500);
            getJewelColorOneSensor();
        }
        else{
            telemetry.addData("Jewel Color", "Not Determined");
            telemetry.update();
        }
    }

    public void getJewelColorTwoSensors() throws InterruptedException {
        if ((colorFront.blue() > colorBack.blue()) && (colorFront.red() < colorBack.red())){
            //jColor refers to color of jewel in the front
            jColor = "blue";
        }
        else if ((colorFront.blue() < colorBack.blue()) && (colorFront.red() > colorBack.red())){
            jColor = "red";
        }
        else if (recCount < 3){
            sleep(500);
            getJewelColorTwoSensors();
        }
        else{
            telemetry.addData("Jewel Color", "Not Determined");
            telemetry.update();
        }
    }
 */