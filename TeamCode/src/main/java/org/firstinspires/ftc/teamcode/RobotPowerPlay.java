package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

public class RobotPowerPlay {

    // Define Motors, Sensors, and Variables *******************************************************
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public Servo intake = null;
    public DcMotor lifter = null;

    public OpMode systemTools;

    //Define Vuforia variables
    private static final String VUFORIA_KEY =
            "ARI7qt//////AAABmW/33bXIIUNlo/jG0i2Bv6oSRXgcZObQM8sOa6wLU4ANjKD6+Eg4L9wKrR2NrInHhXRk/LU/Wfc8DL9+eKBlJhMQWznsFsSYHUVFgLkiZW7PIzsszc29IAMMrjsSzv+HgwUpIKGvA8JkvCnmw5R7lK4eDiphX0nFJ2bp7yi7bQw1xbw5+OnyQPh21yg/50u1XEcWODt3ser7kaJWhVPON4O475BCCW5ZaUs3dzLqsd169RQGSr+BcWxDqlKFvZXl0YByv8yiXIqgOFflKR2MLHOR5N4dKneEny7quns3RUsUQwIWBqkVQrv3Bou/yXIuzGOyVAkUDJGUIPSsTqt6HjlDz2bPACOvJCL1ngCBoA7/ ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public RobotPowerPlay.lightsStates LEDColor = RobotPowerPlay.lightsStates.Off;
    public RobotPowerPlay.lightsStates lastLEDColorBox = RobotPowerPlay.lightsStates.Off;
    public RobotPowerPlay.lightsStates LEDColorBox = RobotPowerPlay.lightsStates.Off;
    public RobotPowerPlay.lightsStates lastLEDColor = RobotPowerPlay.lightsStates.Off;
    public DigitalChannel redLED;
    public DigitalChannel greenLED;

    public enum lightsStates {
        Off, Red, Green, Amber
    }


    HardwareMap hwMap;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;


    public final int lifterMinimum = 0;
    public final int lifterLevelOne = -1050; //Old: -1150  11/1/2022  New: -1000    dropping by 150   in future potentially drop by 170
    public final int lifterLevelTwo = -1600; //Old: -1700  11/1/2022  New: -1550
    public final int lifterLevelThree = -2600;//Old: -2600  11/1/2022 New: -2450
    public final int lowJunctionPos = -400;  //Old: -400    11/1/2022 New: -250

    //Init Methods *********************************************************************************
    public void initAuto(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoders();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotors();
        resetDriveEncoders();   // added on 2-13-22
        startDriveEncoders();
    }

    public void initAutoTester(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;
    }

    //Init Methods**********************************************************************************
    public void initTele(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoderless();

        // reset lifter encoder
        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


     /*   PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);*/

        //carousel.setMode(DcMotorex.RunMode.RUN_USING_ENCODER);



        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //intake.setPosition(1);


        stopAllMotors();
        startDriveEncoderless();
    }

    public void setupMotorsGeneric() {
        // Define and Initialize Motors
        // MOTOR DISABLED FOR LIFTER TESTING
        frontLeftMotor  = hwMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftRear");
        backRightMotor = hwMap.get(DcMotor.class, "rightRear");
        lifter = hwMap.get(DcMotor.class, "lifter");
        intake = hwMap.get(Servo.class,"intake_servo");
        DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        redLED = hwMap.get(DigitalChannel.class, "red_LED");
        greenLED = hwMap.get(DigitalChannel.class, "green_LED");
        lifter = hwMap.get(DcMotor.class, "lifter");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }


    //Autonomous Movement Commands (forward, turn, strafe, lifter, ect.)

    //Auto go distance function (CM, power, handoff, opmode) - uses gotoTarget
    public void GoDistance(double centimeters, double power, boolean Handoff, LinearOpMode linearOpMode) {
        // holds the conversion factor for TICKS to centimeters
        final double conversionFactor = 17.59; // Number came from testing, may need to be improved

        // FIX REVERSE GODISTANCE MOVEMENT
        //power *= -1;

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
    public void intake(boolean up) {
        if (up) {
            intake.setPosition(0);
        } else {
            intake.setPosition(1);
        }
    }
    // Rotate Robot (degress,power,op,telOn) - uses position checks
    public void Rotate(int degrees, double power, LinearOpMode linearOpMode, boolean TelemetryOn) {

        final double conversionFactor = 9.6; // for outreach robot: 8.46, for FreightFrenzy robot: 9.6

        if (degrees < 0 && power > 0) power = -power;

        int ticks = (int) abs(Math.round(degrees * conversionFactor));

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
                (abs(frontLeftMotor.getCurrentPosition()) < ticks && abs(frontRightMotor.getCurrentPosition()) < ticks && abs(backLeftMotor.getCurrentPosition()) < ticks && abs(backRightMotor.getCurrentPosition()) < ticks)) {
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
    public void Strafe(double Centimeters, double Power, LinearOpMode linearOpMode, boolean TelemetryOn) {
        final double conversionFactor = 18.38; // outreach robot: 8.46 FreightFrenzy robot: TBD
        Centimeters = Centimeters * -1;

        if (Centimeters < 0 && Power > 0) Power = -Power;

        int ticks = (int) abs(Math.round(Centimeters * conversionFactor));

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

        int FLtarget = frontLeftMotor.getCurrentPosition() - ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() + ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();

        //setDrivePower(Power, -Power, Power, -Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotor.setPower(Power);
        backRightMotor.setPower(-Power);
        backLeftMotor.setPower(+Power);

        while (linearOpMode.opModeIsActive() &&
                (abs(frontLeftMotor.getCurrentPosition()) < ticks || abs(frontRightMotor.getCurrentPosition()) < ticks || abs(backLeftMotor.getCurrentPosition()) < ticks || abs(backRightMotor.getCurrentPosition()) < ticks)) {
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

    //**************************************************************************************************

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
        power = abs(power);

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

        centimeters = abs(centimeters);
        power = abs(power);

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
        setDrivePower(0, 0, 0, 0);
    }

    public void wait(long timeout, LinearOpMode linearOpMode) {
        linearOpMode.sleep(timeout);
    }

    //End all movement
    public void stopAllMotors() {
        stopDriveMotors();
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
        // DISABLED FOR LIFTER TESTING
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startDriveEncoderless() {
        // DISABLED FOR LIFTER TESTING
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //Additonal robot functions


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

    public void updateLightsTele(boolean flashFreezeActive) {
        if (flashFreezeActive) {
            LEDColor = RobotPowerPlay.lightsStates.Red;
        } else {
            LEDColor = RobotPowerPlay.lightsStates.Green;
        }
        if (LEDColor != lastLEDColor) {
            setLightsState(LEDColor);
            lastLEDColor = LEDColor;
        }

    }

    public void setLightsState(RobotPowerPlay.lightsStates state) {
        LEDColor = state;
        if (LEDColor == RobotPowerPlay.lightsStates.Red) {
            redLED.setState(true);
            greenLED.setState(false);
        } else if (LEDColor == RobotPowerPlay.lightsStates.Green) {
            redLED.setState(false);
            greenLED.setState(true);
        }
    }

    //*************************************************************************************************

    // Lifter

    // TeleOp lifter functions

    public void asynchLift(double ticks, double power, LinearOpMode linearOpMode) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setTargetPosition((int) target);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }

    // Auto lifter functions

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
                systemTools.telemetry.addData("counts", lifterposition);
                systemTools.telemetry.update();
            }
        } else {
            while (linearOpMode.opModeIsActive() && lifterposition > target) {
                lifterposition = lifter.getCurrentPosition();
            }
        } // treats 0 power as negative but it wont do anything anyway

        lifter.setPower(0);
    }

    //*************************************************************************************************

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
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
            tfod.setZoom(1.0, 16.0 / 9.0);
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
                } else if (recognition.getLabel().equals("Cube")) {
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


    public Recognition detectSleeveLoop(LinearOpMode linearOpMode, double timeoutS) {
        // This function loops till it finds the object you specify in objectName, and returns the object
        // The function will return none if the object can't be found before the timeout, or if the opmode is stopped.
        //Duck's object name is "Duck"
        Recognition object = null;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (linearOpMode.opModeIsActive() && tfod != null && object == null && runtime.seconds() < timeoutS) {
            linearOpMode.sleep(4000);
            object = detectSleeve(linearOpMode);
        }
        return object;

    }



    public Recognition detectSleeve(LinearOpMode linearOpMode) {
        //initVuforia();
        //initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }



            List<Recognition> recs = tfod.getUpdatedRecognitions();

            if (recs != null) {
                if (recs.size() != 0) {
                    return recs.get(0);
                }
            }

            return null; // returns first thing it detects out of the 3 detected

    }

    public Recognition detectFrame(LinearOpMode linearOpMode) {
        //initVuforia();
        //initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }


        //linearOpMode.sleep(1000);
        List<Recognition> recs = tfod.getUpdatedRecognitions();


        return recs.get(0); // returns first thing it detects out of the 3 detected

    }




    public int getRoute(LinearOpMode linearOpMode, Recognition detected) {


        linearOpMode.telemetry.addData("whats detected: ", detected.getLabel());
        int route = -1;

        if (detected.getLabel() == "1 Bolt") {
            route = 1;
            //return 1;
        } else if (detected.getLabel() == "2 Bulb") {
            route = 2;
            //return 2;
        } else if (detected.getLabel() == "3 Panel") {
            route = 3;
            //return 3;
        }

        linearOpMode.telemetry.addData("route: ", route);
        linearOpMode.telemetry.addData("recognition: ", detected);


        return route; // returns -1 for error
    }

}
