package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotFreightFrenzy;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@TeleOp(name="TeleMeet1", group="Iterative Opmode")

public class TelePowerPlayMeet1 extends OpMode {
    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer

    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;
    double lifterPower;

    // enums
    enum States {
        Forwards, Backwards, Off, On
    }

    enum lifterStates {
        Home, Low, Middle, High, Manual, Junction
    }

    // Setup booleans for state machines
    boolean lowSpeedActivated = false;
    boolean leftBumperDown = false;

    // intake variables
    double initialin;
    boolean intakeUp = true;
    boolean movingintake = false;
    boolean intakePressed = false;

    // lifter encoder positions
    // b is 0
    // a is -1150
    // x is -1700
    // y is -2600

    // state machine variables
    States intakeState = States.Off;
    lifterStates targetLifterLocation = lifterStates.Home;
    lifterStates lifterLocation = lifterStates.Home;

    // Setup booleans for state machines
    boolean firstHomeLift = true;
    boolean freightGateUp = false;
    boolean freightGatePressed = false;
    boolean storageUp = true;
    boolean movingStorage = false;
    boolean storagePressed = false;
    boolean movingLifter = false;
    boolean driverAButtonDown = false;
    boolean bumperPressed = false;
    boolean carouselMovingBlue = false;
    boolean carouselMovingRed = false;


    public void setLifterPos() {

    }

    // Exponential drive values
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }


    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        // robot.lifter.setMode(DcMotor.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // controller variables
        double y = exponential(gamepad1.left_stick_y, 1);
        double x = exponential(-gamepad1.left_stick_x, 1); //* strafingConstant; // coefficient counteracts imperfect strafing
        double rx = exponential(-gamepad1.right_stick_x, 1);
        double ry2 = gamepad2.left_stick_y;

        //**********************************************************************************************************************
        // FlashFreeze

        // Toggle for FlashFreeze
        if (gamepad1.left_bumper) {
            if (!leftBumperDown) {
                leftBumperDown = true;
                lowSpeedActivated = !lowSpeedActivated;
            }
        } else {
            leftBumperDown = false;
        }

        // REDUCED TURNING SPEED
        if (lowSpeedActivated) {
            rx = rx/1.5;
        }

        //****************************************************************************************
        // Calculate motor power

        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = y - x + rx;
        backRightPower = y + x - rx;

        lifterPower = ry2;

        // Make sure driving power is -1 to 1 and set max/min values
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        lifterPower = Range.clip(lifterPower, -1.0, 1.0);

        // Reducing power for each drive motor to one third of its original power for flash freeze
        if (lowSpeedActivated) { // was divison by 1.5 - 2/3, now times .8
            frontLeftPower *= .6;
            frontRightPower *= .6;
            backLeftPower *= .6;
            backRightPower *= .6;}
        //    robot.setLightsState(RobotPowerPlay.lightsStates.Red);
        //}else{
        //    robot.setLightsState(RobotPowerPlay.lightsStates.Green);
        //}
        robot.updateLightsTele(lowSpeedActivated);
        //******************************************************************************************

        // Set all the drive motors power
        robot.setDrivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);


        //******************************************************************************************

        // Lifter implementation

        /* TODO:
        preset buttons for location
        nudge using bumpers
        DONE - joystick that controls power
         */

        if (gamepad2.b) { // Home Position
            if (!movingLifter) {
                //robot.storage.setPosition(0); // closes storage automatically - caused issues sometimes
                if (lifterLocation != lifterStates.Home) {
                        movingLifter = true;
                        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lifter.setTargetPosition(robot.lifterMinimum);
                        robot.lifter.setPower(1);
                        targetLifterLocation = lifterStates.Home;
                }
            }
        }
        else if (gamepad2.y) { // Highest Level
            // Checks we aren't already autonomously moving the lifter. If we are going to home, let it through as an override
            if (!movingLifter || targetLifterLocation == lifterStates.Home) {
                if (lifterLocation != lifterStates.High || targetLifterLocation == lifterStates.Home) { // Don't go to a currently set state
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterLevelThree);   // Now using 20:1 motor was 6100 with 40:1 motor.
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(-1.0);
                    targetLifterLocation = lifterStates.High;
                }
            }
        }
        else if (gamepad2.x) { // Middle Level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Middle) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterLevelTwo); // May be changed later  Now using 20:1 motor was 4500 with 40:1 motor.
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterLevelTwo) { // Set the power to match with the goal direction
                        robot.lifter.setPower(-1.0);
                    } else {
                        robot.lifter.setPower(1.0);
                    }
                    targetLifterLocation = lifterStates.Middle;
                }
            }
        }
        else if (gamepad2.a) {// Lower level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Low) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterLevelOne); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterLevelOne) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-1.0);
                    }
                    targetLifterLocation = lifterStates.Low;
                }
            }
        } else if (gamepad2.right_bumper) {// low junction level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Junction) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lowJunctionPos); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lowJunctionPos) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-1.0);
                    }
                    targetLifterLocation = lifterStates.Junction;
                }
            }
        }

        if (movingLifter) { // Check if lifter is moving using driver enhancement (autonomous)
            if (targetLifterLocation != lifterStates.Home) {
                if (!robot.lifter.isBusy()) { // Check if lifter is finished reaching target position
                    robot.lifter.setPower(0); // Stop motion and state once it gets there
                    movingLifter = false;
                    lifterLocation = targetLifterLocation;
                }
            } else {
                if (!robot.lifter.isBusy()) {
                    robot.lifter.setPower(0);
                    movingLifter = false;
                    lifterLocation = lifterStates.Home;
                    targetLifterLocation = lifterStates.Manual;
                }
            }
        } else { // Manual lifter motion
            robot.lifter.setPower(lifterPower); // Performs safety checks internally
            if (Math.abs(lifterPower) > 0.1) { // Clear automated state if moving manually
                if (lifterLocation != lifterStates.Manual) {
                    lifterLocation = lifterStates.Manual;
                    robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                robot.lifter.setPower(lifterPower);//
            }
        }


        //******************************************************************************************

        // Gripper stuff


        if (!movingintake) {
            if (gamepad1.right_bumper) {
                intakePressed = true;
                movingintake = true;
                initialin = getRuntime();
            }
        }

        if (intakePressed && movingintake) {
            if (!intakeUp) {
                robot.intake.setPosition(0);
            } else if (intakeUp) {
                robot.intake.setPosition(1);
            }
        }
        if (movingintake) {
            if (getRuntime() - initialin > .3) {
                intakePressed = false;
                intakeUp = !intakeUp;
                movingintake = false;
            }
        }


        // robot.lifter.setPower(lifterPower);

        telemetry.addData("Lifter Power: ", lifterPower);
        telemetry.addData("Lifter Ticks: ", robot.lifter.getCurrentPosition());

        // Update telemetry at end
        telemetry.update();

    }

    public void stop() {
        robot.stopAllMotors();
    }

}
