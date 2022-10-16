package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.code.Lint;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Robot {

    // Define Motors and sensors
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public Servo storage = null;
    public Servo freightGate = null;

    public DcMotor intake = null;
    public DcMotor carousel = null;
    public DcMotor lifter = null;

    public DigitalChannel lifterSwitch1 = null;
    public DigitalChannel lifterSwitch2 = null;
    public DigitalChannel redLED;
    public DigitalChannel greenLED;
    public DigitalChannel redLEDBox;
    public DigitalChannel greenLEDBox;

    public NormalizedColorSensor boxColorSensor;

    // ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();


    public OpMode systemTools;

    public enum States {
        On, Off, Backwards, Forwards
    }
    public enum lightsStates {
        Off, Red, Green, Amber
    }
    public enum gateStates {
        Open, Close, None
    }

    //Define Vuforia variables
    private static final String VUFORIA_KEY =
            "AWby6Eb/////AAABmXfrdIVO1EIvvPlEM+qHg31MuXYM562OEzF23bcuSbxQBt5et3v1ugI+6fn/JqTmkODjtPtFAkZCWCkzT9s9LrQwTFEyl9f/rBYu3Z/6JHR8vNRsIO+21HUs3BmG3gvfoPUfuTOAy+TfMA7HHe2R5Cj3w07dOyZGeVWRUIpNb4WVdy+baZoVU08+F2kJkOZw3sCq4TG1/I2UgkuysnGkDtSxj5NTTX/a+RgjeO8aKwlBaxC4ijAmGdLe9NNCL2WxlB8HsH6boRE02Oy+WISw0KN2Xw4ucur7CPRJUeJANaycD8vQsM7T595F8SU8QPQzdDGlJ7k4FflZVBnZjVJy0YiGiri/T7b2ZU2jKAuyL17C";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    public static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //Define other variables
        /*    public final int lifterMinimum = -20;     // these values are for REV 20:1 spur motor
            public final int lifterLevelOne = 1500;
            public final int lifterLevelTwo = 2330;
            public final int lifterLevelThree = 3050;*/
    public final int lifterMinimum = -10;       //These values are for Gobilda 13:1 motor
    public final int lifterLevelOne = 1050;
    public final int lifterLevelTwo = 1570;
    public final int lifterLevelThree = 2060;
    HardwareMap hwMap;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public int lifterHome = 0;
    public lightsStates lastLEDColor = lightsStates.Off;
    public lightsStates LEDColor = lightsStates.Off;
    public lightsStates lastLEDColorBox = lightsStates.Off;
    public lightsStates LEDColorBox = lightsStates.Off;

    public gateStates gate = gateStates.Open;

    //Init Methods
    public void initAuto(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoders();
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set this position as the encoder 0 place
        //lifter.setDirection(DcMotorSimple.Direction.REVERSE); // needed after changing cartridge from 40:1 to 20:1 1-22-22
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotors();
        resetDriveEncoders();   // added on 2-13-22
        startDriveEncoders();
    }

    //Init Methods
    public void initTele(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoderless();


     /*   PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);*/

       lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
        //lifter.setDirection(DcMotorSimple.Direction.REVERSE); // needed after changing cartridge from 40:1 to 20:1 1-22-22
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set this position as the encoder 0 place

        //carousel.setMode(DcMotorex.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        storage.setPosition(0);
        stopAllMotors();
        startDriveEncoderless();
    }

    public void setupMotorsGeneric() {
        // Define and Initialize Motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftRear");
        backRightMotor = hwMap.get(DcMotor.class, "rightRear");
        /*
        storage = hwMap.get(Servo.class,"storage_servo");
        freightGate = hwMap.get(Servo.class,"freight_gate_servo");

        intake = hwMap.get(DcMotor.class, "intake_motor");
        carousel = hwMap.get(DcMotor.class, "carousel_motor");
        lifter = hwMap.get(DcMotor.class, "lifter_motor");

        lifterSwitch1 = hwMap.get(DigitalChannel.class, "lifter_switch_one");
        lifterSwitch2 = hwMap.get(DigitalChannel.class, "lifter_switch_two");
        redLED = hwMap.get(DigitalChannel.class, "red_LED");
        greenLED = hwMap.get(DigitalChannel.class, "green_LED");
        redLEDBox = hwMap.get(DigitalChannel.class, "red_LED_box");
        greenLEDBox = hwMap.get(DigitalChannel.class, "green_LED_box");
        boxColorSensor = hwMap.get(NormalizedColorSensor.class, "box_color");
        boxColorSensor.setGain(2);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLEDBox.setMode(DigitalChannel.Mode.OUTPUT);
        greenLEDBox.setMode(DigitalChannel.Mode.OUTPUT);

         */
        DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
    }

    //Autonomous Movement Commands (forward, turn, strafe, lifter, ect.)

    //Auto go distance function (CM, power, handoff, opmode) - uses gotoTarget
    public void GoDistanceA(double centimeters, double power, boolean Handoff, LinearOpMode linearOpMode) {
        // holds the conversion factor for TICKS to centimeters
        final double conversionFactor = 17.59; // Number came from testing, may need to be improved

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversionFactor);

        resetDriveEncoders();

        //Calculate the target for each specific motor
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);

        startDriveEncodersTarget();

        setDrivePower(power, power, power, power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //     (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        if (!Handoff) stopDriveMotors();
    }

    // Rotate Robot (degress,power,op,telOn) - uses position checks
    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode, boolean TelemetryOn) {

        final double conversionFactor = 9.6; // for outreach robot: 8.46, for FreightFrenzy robot: 9.6

        if (degrees < 0 && power > 0) power = -power;

        int ticks = (int) Math.abs(Math.round(degrees * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }
        //Conversions to rotate
        int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();
        //Starts to rotate
        // setDrivePower(power, -power, -power, power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(frontRightMotor.getCurrentPosition()) < ticks && Math.abs(backLeftMotor.getCurrentPosition()) < ticks && Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }
//*************************************************************************************************
    // Strafe program inputs (cm,power,opmode) - uses position checks POSITIVE DISTACE GOES RIGHT
    public void StrafeCM(double Centimeters, double Power, LinearOpMode linearOpMode, boolean TelemetryOn) {
        final double conversionFactor = 18.38; // outreach robot: 8.46 FreightFrenzy robot: TBD
        Centimeters= Centimeters*-1;

        if (Centimeters < 0 && Power > 0) Power = -Power;

        int ticks = (int) Math.abs(Math.round(Centimeters * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }

        int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() - ticks;
        int BRtarget = backRightMotor.getCurrentPosition() + ticks;

        startDriveEncoders();

        //setDrivePower(Power, -Power, Power, -Power);
        frontLeftMotor.setPower(Power);
        frontRightMotor.setPower(-Power);
        backRightMotor.setPower(Power);
        backLeftMotor.setPower(-Power);

        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks || Math.abs(frontRightMotor.getCurrentPosition()) < ticks || Math.abs(backLeftMotor.getCurrentPosition()) < ticks || Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }
//**************************************************************************************************

// Strafe program inputs (cm,power,opmode) - uses position checks POSITIVE DISTACE GOES RIGHT
    //closes storage at same time
public void StrafeandcloseCM(double Centimeters, double Power, double Timeout, LinearOpMode linearOpMode, boolean TelemetryOn) {
    final double conversionFactor = 18.38; // outreach robot: 8.46 FreightFrenzy robot: TBD
    Centimeters= Centimeters*-1;

    if (Centimeters < 0 && Power > 0) Power = -Power;

    int ticks = (int) Math.abs(Math.round(Centimeters * conversionFactor));

    if (TelemetryOn) {
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();
    }

    resetDriveEncoders();

    if (TelemetryOn) {
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();
    }
    double elapsedTime = 0;
    double startTime = systemTools.getRuntime();

    int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
    int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
    int BLtarget = backLeftMotor.getCurrentPosition() - ticks;
    int BRtarget = backRightMotor.getCurrentPosition() + ticks;

    startDriveEncoders();

    //setDrivePower(Power, -Power, Power, -Power);
    frontLeftMotor.setPower(Power);
    frontRightMotor.setPower(-Power);
    backRightMotor.setPower(Power);
    backLeftMotor.setPower(-Power);

    while (linearOpMode.opModeIsActive() &&
            (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks || Math.abs(frontRightMotor.getCurrentPosition()) < ticks || Math.abs(backLeftMotor.getCurrentPosition()) < ticks || Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
            if ((systemTools.getRuntime()-startTime)> Timeout)
                storage.setPosition(0);       //Close storage
    }

    stopDriveMotors();

    startDriveEncoders();

    if (TelemetryOn) {
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", ticks);
        systemTools.telemetry.update();
    }

}
    //**************************************************************************************************


    // movement method for lifter in autonomous
    // ticks should always be positive and the power will set the direction
    public void lifterA(double ticks, double power, LinearOpMode linearOpMode) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(power);
        if (power > 0) {
            while (linearOpMode.opModeIsActive() && lifterposition < target) {
                lifterposition = lifter.getCurrentPosition();
                systemTools.telemetry.addData("counts",lifterposition );
                systemTools.telemetry.update();
            }
        }
        else {
            while (linearOpMode.opModeIsActive() && lifterposition > target && !(lifterSwitchTriggered())) {
                lifterposition = lifter.getCurrentPosition();
            }
        } // treats 0 power as negative but it wont do anything anyway

        lifter.setPower(0);
    }

    // ASYNCH LIFT

    public void asynchLift(double ticks, double power, LinearOpMode linearOpMode) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setTargetPosition((int)target);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }

    // DISTANCE LIFT
    public void DistanceLift(double centimeters, double drivingpower,double ticks, double lifterpower, LinearOpMode linearOpMode){
        final double conversionFactor = 17.59; // Number came from testing, may need to be improved

        // sets the power negative if the distance is negative
        if (centimeters < 0 && drivingpower > 0) {
            drivingpower = drivingpower * -1;
        }
        double lifterposition = lifter.getCurrentPosition();
        if (lifterpower < 0) {
            ticks = -ticks;
        }
        int target = (int)(lifterposition + ticks);

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversionFactor);

        resetDriveEncoders();

        //Calculate the target for each specific motor
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);

        startDriveEncodersTarget();

        lifter.setTargetPosition(target);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lifter.setPower(lifterpower);
        setDrivePower(drivingpower, drivingpower, drivingpower, drivingpower);

        /*
        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //     (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) && lifterposition < target && (!lifterSwitchTriggered() || lifterpower > 0)) { // Do not put OR! Changed from OR to AND for if statement
            lifterposition = lifter.getCurrentPosition();

            // systemTools.telemetry.addData("counts",lifterposition );
            // systemTools.telemetry.update();
        }
        if (linearOpMode.opModeIsActive() &&(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())){
            lifter.setPower(0);
            while (linearOpMode.opModeIsActive() &&(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            }
        }
        else if(linearOpMode.opModeIsActive() && lifterposition < target && !(lifterSwitchTriggered())) {
            stopDriveMotors();
            while(linearOpMode.opModeIsActive() && lifterposition < target && !(lifterSwitchTriggered())){
                lifterposition = lifter.getCurrentPosition();
            }

        }

         */

        boolean stillDriving = true;
        boolean stillLifting = true;
        while (linearOpMode.opModeIsActive() && (stillLifting || stillDriving)){
            lifterposition = lifter.getCurrentPosition();
            systemTools.telemetry.addData("counts",lifterposition );
            systemTools.telemetry.update();

            if (!((frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()))){
                stopDriveMotors();
                stillDriving = false;
            }
            if(!lifter.isBusy() || (lifterSwitchTriggered() && lifterpower < 0)) {
                lifter.setPower(0);
                stillLifting = false;
            } else if ((lifterposition < 100) && (lifterpower < 0)) {    // slow down only on return to home
                lifter.setPower(.06);       //power was 0.3 for REV motor
            }
            else if ((lifterposition < 300 ) && lifterpower >0) {
                lifter.setPower(0.5);
            }
            else {
                lifter.setPower(lifterpower);
            }
            if (!stillLifting && !stillDriving){
                break;
            }
        }

        //lifter.setPower(0);
        //stopDriveMotors();
    }

    public void rotateCarouselEnc(int encoderCount, double power, LinearOpMode linearOpMode) {
        int targetPos = carousel.getTargetPosition() + encoderCount;

        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setPower(power);

        while (linearOpMode.opModeIsActive() && carousel.getCurrentPosition() < targetPos) {

        }

        carousel.setPower(0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void carouselAutoAcceleration(double timeS, double maxPower, ElapsedTime runtime, LinearOpMode linearOpMode) {
        double startTime = systemTools.getRuntime();
        double targetTime = startTime + timeS;
        double power = 0.1;
        while (linearOpMode.opModeIsActive() && (targetTime) > systemTools.getRuntime()) {
            systemTools.telemetry.addData("Runtime: ", systemTools.getRuntime());
            systemTools.telemetry.addData("Start time: ", startTime);
            systemTools.telemetry.addData("Power!!!!: ", power);
            systemTools.telemetry.update();
            power = Range.clip((3 * (targetTime/systemTools.getRuntime()) + .1), 0, maxPower);
            carousel.setPower(power);
            linearOpMode.sleep(50);
        }
        carousel.setPower(0);
    }

    // Strafe program inputs (cm,power,opmode) - uses position checks POSITIVE DISTACE GOES RIGHT
    // Normal strafe function but closes box half way through
    public void StrafeStorageCM(double Centimeters, double Power, LinearOpMode linearOpMode, boolean TelemetryOn) {
        final double conversionFactor = 18.38; // outreach robot: 8.46 FreightFrenzy robot: TBD
        Centimeters= Centimeters*-1;
        boolean storageClosing = false;

        if (Centimeters < 0 && Power > 0) Power = -Power;

        int ticks = (int) Math.abs(Math.round(Centimeters * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }

        int FLstart  = frontLeftMotor.getCurrentPosition();
        int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() - ticks;
        int BRtarget = backRightMotor.getCurrentPosition() + ticks;

        startDriveEncoders();

        //setDrivePower(Power, -Power, Power, -Power);
        frontLeftMotor.setPower(Power);
        frontRightMotor.setPower(-Power);
        backRightMotor.setPower(Power);
        backLeftMotor.setPower(-Power);

        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks || Math.abs(frontRightMotor.getCurrentPosition()) < ticks || Math.abs(backLeftMotor.getCurrentPosition()) < ticks || Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
            if (!storageClosing && (frontLeftMotor.getCurrentPosition() < (FLstart + .5 * ticks))) {
                storage.setPosition(0);
                storageClosing = true;
            }
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }

    //Acceleration movement functions (drive and strafe)

    public void goDistanceAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // Holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 17.59;  // was 22 last season, now 17.59 based on godistance, may need tweaked
        double setPower = 0.0;
        double percent;
        double percent2;
        boolean backwards;

        // Sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }
        backwards = power < 0;
        power = Math.abs(power);

        // Calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
         systemTools.telemetry.addLine();
         systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                 frontLeftMotor.getCurrentPosition(),
                 frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
          systemTools.telemetry.update();


        // sets the target position for each of the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        //setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        // Sets the motors to start running to the position they have been given
        //startDriveEncodersTarget();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //   (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
                // if (setPower  <0.03) {setPower = 0.03;} // Minimum if needed to stop without rolling down too much disabled for tourney 2 and 3.
            }

            // set the power the motors need to be going at
            if (!backwards) {
                //setDrivePower(setPower, setPower, setPower, setPower);
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(setPower);
            } else {
                //setDrivePower(-setPower, -setPower, -setPower, -setPower);
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

        // Send telemetry message to indicate successful Encoder reset
         systemTools.telemetry.addLine();
         systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                 frontLeftMotor.getCurrentPosition(),
                 frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
         //systemTools.telemetry.update();
    }

    public void strafeAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // Holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 18.38; // Changed to current strafe conversion - modify to match for acceleration
        double setPower = 0.0;
        double percent;
        double percent2;
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;
        boolean left = centimeters < 0;

        centimeters = Math.abs(centimeters);
        power = Math.abs(power);

        // Calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
         systemTools.telemetry.addLine();
         systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                 frontLeftMotor.getCurrentPosition(),
                 frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
          systemTools.telemetry.update();

        // Sets the target position for each of the motor encoders
        if (left) {
            FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = backRightMotor.getCurrentPosition() + TICKS;
        }

        //setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        // Sets the motors to start running to the position they have been given
        //startDriveEncodersTarget();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
                //set minimum power to 0.05 to allow the robot to actually hit the target
                if (setPower < 0.05) {
                    setPower = 0.05;
                }
            }

            // set the power the motors need to be going at
            if (left) {
                //setDrivePower(-setPower, setPower, -setPower, setPower);
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(setPower);
            } else {
                //setDrivePower(setPower, -setPower, setPower, -setPower);
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

        // Send telemetry message to indicate successful Encoder reset
         systemTools.telemetry.addLine();
         systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                 frontLeftMotor.getCurrentPosition(),
                 frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
         //systemTools.telemetry.update();
    }

    // Teleoperated commands
    public void setLifterPower(double lifterPower) {
        // This method sets the power on the lifter while checking it is safe to do so. The current max speed for the lifter is 1, change that in here
        if (lifterSwitchTriggered()) {
            lifter.setPower(-Range.clip(lifterPower, -1.0, 0.0));
        } else if (getLifterPosition() <= lifterMinimum) {
            lifter.setPower(-Range.clip(lifterPower, -1.0, 0.0));
        } else if (getLifterPosition() >= lifterLevelThree) { // Not overextending lifter
            lifter.setPower(-Range.clip(lifterPower, 0.0, 1.0));
        } else {
            lifter.setPower(-lifterPower);
        }
    }

    public int getLifterPosition() {
        return (lifter.getCurrentPosition() - lifterHome);
    }

    public void updateLightsTele(boolean flashFreezeActive) {
        if (flashFreezeActive) {
            LEDColor = lightsStates.Red;
        } else {
            LEDColor = lightsStates.Green;
        }
        if (LEDColor != lastLEDColor) {
            setLightsState(LEDColor);
            lastLEDColor = LEDColor;
        }

    }

    public boolean movingGate = false;
    public void updateBoxLightsTele(double storageHue) {
        if (storageHue > 40 && storageHue < 200) { // Something in storage box, may need tweaked
            LEDColorBox = lightsStates.Amber;
        } else {
            LEDColorBox = lightsStates.Off;
        }
        if (LEDColorBox != lastLEDColorBox) {
            setLightsState(LEDColorBox);
            lastLEDColorBox = LEDColorBox;
        }
    }

    // Basic movement commands (powers, drivemodes)

    public void setDrivePower(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }
    // Basic move and target distance
    public void setDriveTarget(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(backLeftTarget);
        backRightMotor.setTargetPosition(backRightTarget);
    }

    public void stopDriveMotors() {
        setDrivePower(0,0,0,0);
    }
    public void wait(long timeout, LinearOpMode linearOpMode) {
        linearOpMode.sleep(timeout);
    }
    //End all movement
    public void stopAllMotors() {
        stopDriveMotors();
        intake.setPower(0);
        carousel.setPower(0);
        lifter.setPower(0);
        // Add any other motors we use here and set them to 0 power
    }
    // DriveEncoders are reset
    public void resetDriveEncoders() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

         frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startDriveEncoders() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startDriveEncoderless() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startDriveEncodersTarget() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }



        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean lifterSwitchTriggered() {
        return (lifterSwitch1.getState() || lifterSwitch2.getState());
    }

    public float getStorageHue() {
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = boxColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[0];
    }

    public void setLightsState(lightsStates state) {
        LEDColor = state;
        if (LEDColor == lightsStates.Amber) {
            redLEDBox.setState(false);
            greenLEDBox.setState(false);
        } else if (LEDColor == lightsStates.Red) {
            redLED.setState(true);
            greenLED.setState(false);
        } else if (LEDColor == lightsStates.Green) {
            redLED.setState(false);
            greenLED.setState(true);
        } else { // Off
            redLEDBox.setState(true);
            greenLEDBox.setState(true);
        }
    }

    public void setLightsStateBox(lightsStates state) {
        LEDColorBox = state;
        if (LEDColorBox == lightsStates.Amber) {
            redLEDBox.setState(false);
            greenLEDBox.setState(false);
        } else if (LEDColorBox == lightsStates.Red) {
            redLEDBox.setState(true);
            greenLEDBox.setState(false);
        } else if (LEDColorBox == lightsStates.Green) {
            redLEDBox.setState(false);
            greenLEDBox.setState(true);
        } else { // Off
            redLEDBox.setState(true);
            greenLEDBox.setState(true);
        }
    }
    public void autoIntake(double drivingpower,double maxdistance,boolean Handoff, LinearOpMode linearOpMode){
        final double conversionFactor = 17.59;
        int TICKS = (int) Math.round(maxdistance * conversionFactor);
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;
        int FLstart = frontLeftMotor.getCurrentPosition() ;
        int FRstart = frontRightMotor.getCurrentPosition() ;
        int BLstart = backLeftMotor.getCurrentPosition() ;
        int BRstart = backRightMotor.getCurrentPosition() ;
        setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        startDriveEncodersTarget();
        intake.setPower(1);
        setDrivePower(drivingpower, drivingpower, drivingpower, drivingpower);
        float storageHue;
        boolean f_storage=false;
        while ( (!f_storage && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()))){
            storageHue = getStorageHue();
            f_storage=(storageHue > 40 && storageHue < 200);
            systemTools.telemetry.addData("Storage Box Full? ", (storageHue > 40 && storageHue < 200));
            systemTools.telemetry.update();
        }
        linearOpMode.sleep(300);
        //freightGate.setPosition(0);  //close intake
        stopDriveMotors();
        linearOpMode.sleep(300);
        //freightGate.setPosition(0);  //close intake   for debugging only
        intake.setPower(0);         // turn off intake
        drivingpower*=-1;
        setDriveTarget(FLstart, FRstart, BLstart, BRstart);
        startDriveEncodersTarget();
        //setDrivePower(drivingpower, drivingpower, drivingpower, drivingpower);
        setDrivePower(-0.4, -0.4, -0.4, -0.4);
        linearOpMode.sleep(300);
        intake.setPower(-1);
        while ((frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
        }
        freightGate.setPosition(0);  //close intake
        intake.setPower(0);
        // Intake isn't stopped - stop in regular auto
        if (!Handoff) stopDriveMotors();
    }
    // Vuforia related functions
    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    public Recognition findObject(String objectName, LinearOpMode linearOpMode) {
        // This function will check once for the specified object, and returns the object if found, or if not found, returns null
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            systemTools.telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            boolean isDuckDetected = false;
            for (Recognition recognition : updatedRecognitions) {
                systemTools.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                systemTools.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                systemTools.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                i++;

                // check if object is detected
                if (recognition.getLabel().equals("Duck")) {
                    systemTools.telemetry.addData("Object Detected", "Duck");
                } else if (recognition.getLabel().equals("Cube")){
                    systemTools.telemetry.addData("Object Detected", "Cube");
                }
                if (recognition.getLabel().equals(objectName) || recognition.getLabel().equals("Cube")) {
                    systemTools.telemetry.update();
                    return recognition;
                }
            }
            systemTools.telemetry.update();
        }
        return null;
    }

    public Recognition findObjectLoop(String objectName, double timeoutS, LinearOpMode linearOpMode) {
        // This function loops till it finds the object you specify in objectName, and returns the object
        // The function will return none if the object can't be found before the timeout, or if the opmode is stopped.
        //Duck's object name is "Duck"
        Recognition object = null;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (linearOpMode.opModeIsActive() && tfod != null && object == null && runtime.seconds() < timeoutS) {
            linearOpMode.sleep(1000);
            object = findObject(objectName, linearOpMode);
        }
        return object;
    }

    public int getDuckLocationBlue(Recognition rec, boolean telemetry) {
        int pos=1; // 1 is default location if no duck
        if (rec != null) {
            if (rec.getLeft() > 240 && rec.getRight() > 330) { // pixel counts go from left to right
                pos = 3;
            } else if (rec.getLeft() < 240 && rec.getRight() < 320) {
                pos = 2;
            }
        }
        if (telemetry) {
            systemTools.telemetry.addData(String.format("label (%d)", pos), null);
            systemTools.telemetry.update();
        }
        return pos;
    }

    public int getDuckLocationRed(Recognition rec, boolean telemetry) {
        int pos=3; // 3 is default location if no duck
        if (rec != null) {
            if (rec.getLeft() > 240 && rec.getRight() > 330) {
                pos = 2;
            } else if (rec.getLeft() < 240 && rec.getRight() < 320) {
                pos = 1;
            }
        }
        if (telemetry) {
            systemTools.telemetry.addData(String.format("label (%d)", pos), null);
            systemTools.telemetry.update();
        }
        return pos;
    }

    public int getDuckLocationBlue2(Recognition rec, boolean telemetry) { // for the second position on Autonomous on the Blue side
        int pos=1; // 1 is default location if no duck
        if (rec != null) {
            if (rec.getLeft() > 240) { // pixel counts go from left to right REMOVED FROM AND STATEMENT: && rec.getRight() > 330
                pos = 3;
            } else if (rec.getLeft() < 240) { // REMOVED FROM AND STATEMENT: && rec.getRight() > 320
                pos = 2;
            }
        }
        if (telemetry) {
            systemTools.telemetry.addData(String.format("label (%d)", pos), null);
            systemTools.telemetry.update();
        }
        return pos;
    }

    public int getDuckLocationRed2(Recognition rec, boolean telemetry) {
        int pos=3; // 3 is default location if no duck
        if (rec != null) {
            if (rec.getLeft() > 240) {
                pos = 2;
            } else if (rec.getLeft() < 240) {
                pos = 1;
            }
        }
        if (telemetry) {
            systemTools.telemetry.addData(String.format("label (%d)", pos), null);
            systemTools.telemetry.update();
        }
        return pos;
    }

    //Additonal robot functions

    // Changes intakes state/motors
    public void intake(States setting) {
        if (setting == States.Off) {
            intake.setPower(0);
        } else if (setting == States.Forwards) {
            intake.setPower(1);
        } else if (setting == States.Backwards) {
            intake.setPower(-1);
        }
    }
    // Storage
    public void storage(boolean up) {
        if (up) {
            storage.setPosition(0);
        } else {
            storage.setPosition(1);
        }
    }

    public void GyroRotateDEGTele(int maxDegrees, double power, double angle) {
        // IMU output is positive for left turn and negative for right turn.  Max degrees determines direction.
        //angle needs to be positive for left turn and negative for right turn as off 3-19-2
        //

        //   imu.initialize(parameters);   No need to initialize here as it sets the teh zero to teh worng angle
        // IMU initialization is done in power shot.  It takes along time >1-sec

        boolean turnRight;  // flag to check rotation direction
        boolean turnComplete = false;   // flag while loop

        // conversion for ticks to ticks
        final double conversion_factor = 12.73;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;

        // Check which direction to turn robot and adjust drive motor power direction
        if (angle < 0) {
            turnRight = true;
        } else {
            turnRight = false;
            power = power * -1;
        }

        // drive with encoders slows the IMU down and it is very inaccurate.  Use run without encoders here

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        //while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
        while (!turnComplete) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle <= 0 && turnRight) {
                turnComplete = true;
                //break;

            } else if (angles.firstAngle >= 0 && !turnRight) {
                turnComplete = true;
                //break;
            }
        }
        stopDriveMotors();

    } //end RotateDegTele
    //*************************************************************************************************

    public void detectSleeve() {

    }

}
