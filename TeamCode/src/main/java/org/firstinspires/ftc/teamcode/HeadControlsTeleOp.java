
        package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        /**
 * Created by raymo on 10/4/17.
 */

@TeleOp (name = "HeadShouldersKneesAndToes", group = "TeleOp")
public class HeadControlsTeleOp extends OpMode {

    private double rightMotion;
    private double leftMotion;
    private double leftRelicPosition;
    private double rightRelicPosition;
    private double currentGyro;
    private double prevGyro;

    private boolean halfSpeed;

    private boolean liftOut;

    private long currentTime;
    private long lastTime;
    private long closeTime;

    private int revolutions;

    // Drive Train
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private boolean braked;

    private Servo gate;
    private DcMotor leftLiftSlide;
    private DcMotor rightLiftSlide;
    private DcMotor liftMani;
    private DcMotor relicMotor;

    // Matt Tunnel
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor inTake;
    private CRServo frontRightTunnel;
    private CRServo backRightTunnel;
    private CRServo frontLeftTunnel;
    private CRServo backLeftTunnel;
    private DcMotor relicLift;


    private Servo leftMani;
    private Servo rightMani;

    private Servo jewelHitter;
    private Servo leftArm;
    private Servo rightArm;
    private Servo leftRelic;
    private Servo rightRelic;
    private Servo leftGlyphClamp;
    private Servo rightGlyphClamp;

    private MattTunnel tunnel;
    private Servo relicGrabber;
    private Servo relicClamp;
    private Servo relic;
    private Servo relicArm;
    private final int DELAYTOGGLE = 200;
    private boolean relicOut;
    private boolean clampClosed;
    private boolean gateClosed;
    private boolean parked;
    private Servo gateServo;
    private double recentPressTime;
    private double recentRelicArmTime = 0;
    private boolean relicClosed;
    private boolean relicLowering;
    private boolean relicRaising;
    private final double DELAY_TIME_MS = 300;
    private final double RELIC_MOTION_TIME_MS = 50;
    private double lastRelicTime;
    boolean clampOpen = true;
    long lastGlyphTime;
    private boolean firstRelicPos = true;
    private BNO055IMU imu;

    private double relicMoveTime;
    private int relicPos;


    // added 12/12- for clamp lift

    private DcMotor leftCLift;
    private DcMotor rightCLift;


    private CRServo leftClamp;
    private CRServo rightClamp;

    //jewelstate 0 is upright, 1 is near upright, 2 is position to hit jewel
    String jewelState = "down";
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        /*
        Expansion Hub 1: Motor 0 is BR, Motor 1 is FR, Motor 2 is BL, Motor 3 is FL
        Expansion Hub 1: Servo 0 is RRelicArm, Servo 1 is LRelioArm, Servo 2 is LRelio, Servo 3 is RRelic
        Expansion Hub 1: Servo 4 is LMani, Servo 5 is RMani
        Expansion Hub 2: Motor 0 is LSlide, Motor 1 is liftMani, Motor 2 is RSlide
        Expansion Hub 2: Servo 0 is JewelHitter
        */

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        //leftArm             = hardwareMap.servo.get("LRelicArm");
        //rightArm            = hardwareMap.servo.get("RRelicArm");
        //leftRelic           = hardwareMap.servo.get("LRelic");
        //rightRelic          = hardwareMap.servo.get("RRelic");
        //leftMani            = hardwareMap.servo.get("LMani");
        //rightMani           = hardwareMap.servo.get("RMani");
        //leftLiftSlide       = hardwareMap.dcMotor.get("LSlide");
        //rightLiftSlide      = hardwareMap.dcMotor.get("RSlide");
        //liftMani            = hardwareMap.dcMotor.get("liftMani");
        jewelHitter = hardwareMap.servo.get("jewelhitter");
        //liftLeft            = hardwareMap.dcMotor.get("leftLift");
        //liftRight           = hardwareMap.dcMotor.get("rightLift");
        inTake = hardwareMap.dcMotor.get("intake");

        frontRightTunnel = hardwareMap.crservo.get("FRT");
        backRightTunnel = hardwareMap.crservo.get("BRT");
        frontLeftTunnel = hardwareMap.crservo.get("FLT");
        backLeftTunnel = hardwareMap.crservo.get("BLT");
        leftGlyphClamp = hardwareMap.servo.get("leftGlyphClamp");
        rightGlyphClamp = hardwareMap.servo.get("rightGlyphClamp");
        relic = hardwareMap.servo.get("relic");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        recentPressTime = 0;
        relicPos = 3;
        //gate = hardwareMap.servo.get("gate");

        //relicGrabber        = hardwareMap.servo.get("relicGrabber");
        relicArm = hardwareMap.servo.get("relicArm");
        halfSpeed = false;
        braked = false;
        liftOut = false;
        relicOut = false;
        clampClosed = false;
        gateClosed = false;
        relicClosed = false;
        rightMotion = 0;
        leftMotion = 0;
        //:)liftLeft = hardwareMap.dcMotor.get("LLift");
        //:)liftRight = hardwareMap.dcMotor.get("RLift");
        tunnel = new MattTunnel(liftLeft, liftRight, inTake, frontRightTunnel, backRightTunnel, frontLeftTunnel, backLeftTunnel);
        //:)relicGrabber        = hardwareMap.servo.get("relic");
        currentTime = 0;
        //relicClamp          = hardwareMap.servo.get("relicClamp");


        leftCLift = hardwareMap.dcMotor.get("LeftClampLift");
        rightCLift = hardwareMap.dcMotor.get("RightClampLift");

        //rightMani.setDirection(Servo.Direction.FORWARD);
        //leftMani.setDirection(Servo.Direction.REVERSE);
        jewelHitter.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Initialization", "done");
        telemetry.update();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        prevGyro = 0;
        currentGyro = 0;
    }

    //Activates when start, not init is pressed. Use it for setting servos to initial positions
    public void start() {
        relicArm.setPosition(1);
        timer.reset();
    }

    @Override
    public void loop() {
        grabRelic();
        //alterRelicMotion();
        //changeRelicPos();
        //lowerRelicArm();
        setPower();
        toggleHalfSpeed();
        useJewel();
        findTrueGyro();
        useRelicArm();
        raiseClamps();
        useGlyphClamps();
        tunnel.toggleIntakeTank(gamepad2.left_stick_y, gamepad2.right_stick_y);
        //tunnel.toggleInTake(gamepad2.left_stick_y, gamepad2.right_trigger, gamepad2.left_trigger);
        //alterRelicMotion();
        //tunnel.manipulateLift(gamepad2.right_stick_y); // TODO: add gamepad contols for these methods
        //manipLift();
        extendRelic();
        reportTelemetry();
    }



    public double getRightVelocity() {
        if (Math.abs(gamepad1.right_stick_y) > 0.05)
            return gamepad1.right_stick_y;
        return 0;
    }

    public double getLeftVelocity() {
        if (Math.abs(gamepad1.left_stick_y) > 0.05)
            return gamepad1.left_stick_y;
        return 0;
    }

    public double getLeftShoulder() {
        if (gamepad1.left_bumper)
            return 1.0;
        return 0;
    }

    public void useJewel() {
        if (gamepad1.dpad_left) {
            jewelHitter.setPosition(0.70);
        } else if (gamepad1.dpad_right) {
            jewelHitter.setPosition(0.25);
        }
    }

    public void alterRelicMotion() {
        if ((gamepad2.x) && (System.currentTimeMillis() - relicMoveTime > DELAY_TIME_MS)) {
            relic.setPosition(0.25);
            relicRaising = true;
            relicLowering = false;
            relicMoveTime = System.currentTimeMillis();
        } else if ((gamepad2.a) && (System.currentTimeMillis() - relicMoveTime > DELAY_TIME_MS)) {
            relic.setPosition(0.65);
            relicLowering = true;
            relicRaising = false;
            relicMoveTime = System.currentTimeMillis();
        }

    }

    public void changeRelicPos() {
        if (System.currentTimeMillis() - lastRelicTime > RELIC_MOTION_TIME_MS) {
            if (relicRaising) {
                if (relic.getPosition() > 0.25) {
                    relic.setPosition(relic.getPosition() - 0.05);
                    lastRelicTime = System.currentTimeMillis();
                }
            } else if (relicLowering) {
                if (relic.getPosition() < 0.65) {
                    relic.setPosition(relic.getPosition() + 0.05);
                    lastRelicTime = System.currentTimeMillis();
                }
            }
        }
    }

    public double getGyroYaw(){
        Orientation angles = imu.getAngularOrientation();
        return (angles.firstAngle * -1);
    }

    public void toggleHalfSpeed() {
        if (halfSpeed) {
            halfSpeed = false;
            telemetry.addData("halfspeed", "disabled");
        } else if (!halfSpeed) {
            halfSpeed = true;
            telemetry.addData("halfspeed", "enabled");
        }
    }

    public void reportTelemetry(){
        getRemainingTime();
        getMotorEncoders();
        telemetry.addData("Angle", revolutions * 360 + currentGyro + "");
        telemetry.update();
    }


    public double getRightShoulder() {
        if (gamepad1.right_bumper)
            return 1.0;
        return 0;
    }

    public double getHalfSpeed() {
        if (halfSpeed)
            return 0.5;
        return 1
                ;

    }

    public void grabRelic() {
        if (System.currentTimeMillis() - recentPressTime > 150) {
            if (gamepad2.x) {
                if (!relicClosed) {
                    relic.setPosition(1);
                    relicClosed = true;
                } else if (relicClosed) {
                    relic.setPosition(0);
                    relicClosed = false;
                }
            }
            recentPressTime = System.currentTimeMillis();
        }
    }


    /*
    public void lowerRelicArm(){
        if (gamepad2.right_bumper) {
            relicArm.setPosition(0.3);
        }
        else if (gamepad2.left_bumper) {
            relicArm.setPosition(1);
        }
        else {
            relicArm.setPosition(0.5);
        }
    }
    */



    /*
    public void useGate() {
        if ((gamepad2.b) && ( System.currentTimeMillis() - currentTime < 500)) {
            if (!gateClosed) {
                gate.setPosition(0.35);
                gateClosed = true;
            } else if (gateClosed) {
                gate.setPosition(0.7);
                gateClosed = false;
            }
            currentTime = System.currentTimeMillis();
        }
    }
    */


    //Jewel Hitter (have not tested positions)


    public void useRelicArm() {
        if (System.currentTimeMillis() - recentRelicArmTime > DELAY_TIME_MS) {
            if (gamepad2.left_bumper) {
                if (relicPos == 2) {
                    telemetry.addData("RelicArmPosition", "up");
                    relicArm.setPosition(1);
                    relicPos = 3;
                    recentRelicArmTime = System.currentTimeMillis();
                } else if (relicPos == 1) {
                    telemetry.addData("RelicArmPosition", "mid");
                    relicArm.setPosition(0.4);
                    relicPos = 2;
                    recentRelicArmTime = System.currentTimeMillis();
                }
            } else if (gamepad2.right_bumper) {
                if (relicPos == 3) {
                    telemetry.addData("RelicArmPosition", "mid");
                    relicArm.setPosition(0.4);
                    relicPos = 2;
                    recentRelicArmTime = System.currentTimeMillis();
                } else if (relicPos == 2) {
                    telemetry.addData("RelicArmPosition", "down");
                    relicArm.setPosition(0.05);
                    relicPos = 1;
                    recentRelicArmTime = System.currentTimeMillis();
                }
            }
        }
    }


    public void raiseClamps() {
        if (gamepad2.y) {
            rightCLift.setPower(0.7);
            leftCLift.setPower(-0.7);
        } else if (gamepad2.a) {
            rightCLift.setPower(-0.7);
            leftCLift.setPower(0.7);
        } else {
            rightCLift.setPower(0);
            leftCLift.setPower(0);
        }
    }


    public void setPower() {
        FL.setPower(getLeftVelocity() - getLeftShoulder() + getRightShoulder());
        FR.setPower(-getRightVelocity() + getLeftShoulder() - getRightShoulder());
        BL.setPower(getLeftVelocity() + getLeftShoulder() - getRightShoulder());
        BR.setPower(-getRightVelocity() - getLeftShoulder() + getRightShoulder());
    }


    //Has some potential for balancing
    public void brake() {
        if (gamepad2.left_bumper) {
            if (!braked) {
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                braked = true;
            } else {
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                braked = false;
            }
        }
    }

    public void useGlyphClamps() {
        if ((gamepad2.dpad_right) && (System.currentTimeMillis() - lastGlyphTime > DELAY_TIME_MS)) {
            if (clampOpen) {
                leftGlyphClamp.setPosition(0.12);
                rightGlyphClamp.setPosition(0.78);
                clampOpen = false;
            } else if (!clampOpen) {
                leftGlyphClamp.setPosition(0.35);
                rightGlyphClamp.setPosition(0.55);
                clampOpen = true;
            }
            lastGlyphTime = System.currentTimeMillis();
        }
    }

    public void extendRelic() {
        if (gamepad2.dpad_up) {
            relicMotor.setPower(1);
        } else if (gamepad2.dpad_down) {
            relicMotor.setPower(-1);
        } else {
            relicMotor.setPower(0);
        }
    }

    public void getRemainingTime(){
        telemetry.addData("Time Remaining", secondsToMinutes());
    }

    public void getMotorEncoders(){
        telemetry.addData("FL Encoder", FL.getCurrentPosition());
        telemetry.addData("FR Encoder", FR.getCurrentPosition());
        telemetry.addData("BL Encoder", BL.getCurrentPosition());
        telemetry.addData("BR Encoder", BR.getCurrentPosition());
    }

    public String secondsToMinutes(){
        double seconds = 120 - timer.seconds();
        return (int)(seconds/60) + ":" + (int)(seconds%60);
    }

    //Method to drop relic at the last second
    public void backupRelicDrop(){
        relic.setPosition(0);
    }

    public void findTrueGyro(){
        prevGyro = currentGyro;
        currentGyro = getGyroYaw();
        if ((prevGyro < 0) && (currentGyro > 0)){
            revolutions ++;
        }
        else if ((prevGyro > 0) && (currentGyro < 0)){
            revolutions--;
        }
    }


    /* Methods for original clamp bot (obsolete)
    //relic lift
    //Relic
    public void setArmPower() {
        if (gamepad2.right_stick_y < -0.2) {
            leftArm.setPosition(1);
            rightArm.setPosition(0);
        }
        else if (gamepad2.right_stick_y > 0.2){
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }
        else{
            leftArm.setPosition(0.5);
            rightArm.setPosition(0.5);
        }
    }
    public void setLiftSlide(){
        if (gamepad2.left_stick_y < -0.1){
            leftLiftSlide.setPower(-1);
            rightLiftSlide.setPower(1);
        }
        else if (gamepad2.left_stick_y > 0.1){
            rightLiftSlide.setPower(-1);
            leftLiftSlide.setPower(1);
        }
        else{
            rightLiftSlide.setPower(0);
            leftLiftSlide.setPower(0);
        }
    }
    public void useRelicGrabber() {
        if (gamepad2.right_stick_y > 0.05){
            relicGrabber.setPosition(1);
            relicOut = true;
        }
        if (gamepad2.right_stick_y < -0.05){
            relicGrabber.setPosition(0);
            relicOut = false;
        }
        if ((gamepad2.right_stick_button) && (relicOut) && (!clampClosed)){
            relicClamp.setPosition(1);
            clampClosed = true;
        }
        if ((gamepad2.right_stick_button) && (relicOut) && (clampClosed)){
            relicClamp.setPosition(1);
            clampClosed = false;
        }
    }
    //clamp for glyphs
    public void setManiPower(){
        if (gamepad2.b){
            leftMani.setPosition(1);
            rightMani.setPosition(1);
            closeTime = System.currentTimeMillis();
        }
        else if (System.currentTimeMillis() - closeTime < 1000){
            leftMani.setPosition(0.3);
            rightMani.setPosition(0.3);
        }
        else{
            leftMani.setPosition(0.5);
            rightMani.setPosition(0.5);
        }
    }
    public void grapRelic() {
        if(gamepad2.x) {
            leftRelicPosition += .005;
            rightRelicPosition -= .005;
            leftRelic.setPosition(leftRelicPosition);
            rightRelic.setPosition(rightRelicPosition);
        }
        if(gamepad2.y) {
            leftRelicPosition += .005;
            leftRelicPosition -= .005;
            leftRelic.setPosition(leftRelicPosition);
            rightRelic.setPosition(rightRelicPosition);
        }
    }
    public void pickRelic(){
        if(gamepad2.x) {
            leftRelic.setPosition(1);
            rightRelic.setPosition(0);
        }
        else if (gamepad2.y){
            leftRelic.setPosition(0);
            rightRelic.setPosition(1);
        }
    }
    public void raiseMani() {
        if (gamepad2.dpad_down){
            liftMani.setPower(1);
        }
        else if (gamepad2.dpad_up){
            liftMani.setPower(-1);
        }
        else{
            liftMani.setPower(0);
        }
    }
    */

}
