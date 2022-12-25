package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TeleRR", group="Iterative Opmode")

public class TeleRR extends LinearOpMode {
    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer
    double slowTime; // initial time for slowmode timeout
    double slowTime2; // initial time for super slowmode timeout

    double checkTimeL, checkTimeH;

    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;
    double lifterPower;
    int smalllift=400;
    // enums
    enum States {
        Forwards, Backwards, Off, On
    }

    enum lifterStates {
        Home, Low, Middle, High, Manual, Junction, Stack, Between
    }

    // Setup booleans for state machines
    boolean lowSpeedActivated = false;
    boolean leftBumperDown = false;
    boolean superLowSpeedActivated = false;
    boolean leftTriggerPressed = false;

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
    TelePowerPlayMeet1.States intakeState = TelePowerPlayMeet1.States.Off;
    TelePowerPlayMeet1.lifterStates targetLifterLocation = TelePowerPlayMeet1.lifterStates.Home;
    TelePowerPlayMeet1.lifterStates lifterLocation = TelePowerPlayMeet1.lifterStates.Home;

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


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        robot.resetDriveEncoders();
        // run using encoders makes it slower but drive very straight
        robot.startDriveEncoders();
        //robot.startDriveEncoderless();
        // robot.lifter.setMode(DcMotorEx.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
        robot.lifter.setTargetPositionTolerance(15);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //**********************************************************************************************************************
            // FlashFreeze

            // Toggle for FlashFreeze
            if (gamepad1.left_bumper && !leftBumperDown) {
                leftBumperDown = true;
                lowSpeedActivated = !lowSpeedActivated;
                superLowSpeedActivated = false;
                slowTime = getRuntime();
            }
            else if (getRuntime() - slowTime > 0.3) {
                leftBumperDown = false;
            }

            //activate or deacti  vate super slow speed when left trigger receives any input (code includes debounce as well)
            if (gamepad1.left_trigger > 0 && !leftTriggerPressed){
                leftTriggerPressed = true;
                superLowSpeedActivated = !superLowSpeedActivated;
                lowSpeedActivated = false;
                slowTime2 = getRuntime();
            }
            else if (getRuntime() - slowTime2 > 0.3) {
                leftTriggerPressed = false;
            }


            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input;

            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            if (lowSpeedActivated) input = new Vector2d(
                    -gamepad1.left_stick_y / 1.5,
                    -gamepad1.left_stick_x / 1.5
            ).rotated(-poseEstimate.getHeading());

            if (superLowSpeedActivated) input = new Vector2d(
                    -gamepad1.left_stick_y / 2.5,
                    -gamepad1.left_stick_x / 2.5
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            robot.updateLightsTele(lowSpeedActivated, superLowSpeedActivated);

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //******************************************************************************************
            lifterPower = gamepad2.left_stick_y;

            // Lifter implementation

            if (getRuntime() > checkTimeL && getRuntime() < checkTimeH){
                if(robot.lifter.isBusy()) {
                    robot.lifter.setPower(0);
                }
            }

            if (gamepad2.b) { // Home Position
                if (!movingLifter) {
                    //robot.storage.setPosition(0); // closes storage automatically - caused issues sometimes
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Home) {
                        movingLifter = true;
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.lifter.setTargetPosition(robot.lifterMinimum);
                        robot.lifter.setPower(1);
                        checkTimeL = getRuntime() + 3000;
                        checkTimeH = checkTimeL + 200;
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Home;
                    }
                }
            }
            else if (gamepad2.y) { // Highest Level
                // Checks we aren't already autonomously moving the lifter. If we are going to home, let it through as an override
                if (!movingLifter || targetLifterLocation == TelePowerPlayMeet1.lifterStates.Home) {
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.High || targetLifterLocation == TelePowerPlayMeet1.lifterStates.Home) { // Don't go to a currently set state
                        movingLifter = true;
                        robot.lifter.setTargetPosition(robot.lifterLevelThree);   // Now using 20:1 motor was 6100 with 40:1 motor.
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.lifter.setPower(-1.0);
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.High;
                    }
                }
            }
            else if (gamepad2.x) { // Middle Level
                if (!movingLifter) {
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Middle) {
                        movingLifter = true;
                        robot.lifter.setTargetPosition(robot.lifterLevelTwo); // May be changed later  Now using 20:1 motor was 4500 with 40:1 motor.
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        if (robot.lifter.getCurrentPosition() > robot.lifterLevelTwo) { // Set the power to match with the goal direction
                            robot.lifter.setPower(-1.0);
                        } else {
                            robot.lifter.setPower(1.0);
                        }
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Middle;
                    }
                }
            }
            else if (gamepad2.a) {// Lower level
                if (!movingLifter) {
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Low) {
                        movingLifter = true;
                        robot.lifter.setTargetPosition(robot.lifterLevelOne); // Now using 20:1 motor was 3000 with 40:1 motor.
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        if (robot.lifter.getCurrentPosition() > robot.lifterLevelOne) { // Set the power to match with the goal direction
                            robot.lifter.setPower(1.0);
                        } else {
                            robot.lifter.setPower(-1.0);
                        }
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Low;
                    }
                }
            } else if (gamepad2.right_bumper) {// low junction level
                if (!movingLifter) {
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Junction) {
                        movingLifter = true;
                        robot.lifter.setTargetPosition(robot.lowJunctionPos); // Now using 20:1 motor was 3000 with 40:1 motor.
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        if (robot.lifter.getCurrentPosition() > robot.lowJunctionPos) { // Set the power to match with the goal direction
                            robot.lifter.setPower(1.0);
                        } else {
                            robot.lifter.setPower(-1.0);
                        }
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Junction;
                    }
                }
            } else if (gamepad2.left_bumper) {// stack level
                if (!movingLifter) {
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Stack) {
                        movingLifter = true;
                        robot.lifter.setTargetPosition(robot.stackPos); // Now using 20:1 motor was 3000 with 40:1 motor.
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        if (robot.lifter.getCurrentPosition() > robot.stackPos) { // Set the power to match with the goal direction
                            robot.lifter.setPower(1.0);
                        } else {
                            robot.lifter.setPower(-1.0);
                        }
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Stack;
                    }
                }
            }

            if (movingLifter) { // Check if lifter is moving using driver enhancement (autonomous)
                if (targetLifterLocation != TelePowerPlayMeet1.lifterStates.Home) {
                    if (!robot.lifter.isBusy()) { // Check if lifter is finished reaching target position
                        robot.lifter.setPower(0); // Stop motion and state once it gets there
                        movingLifter = false;
                        lifterLocation = targetLifterLocation;
                    }
                }/* else { // Check if it is for home position
                if (robot.lifterSwitchTriggered()) { // Stops if limit switch is pressed
                    robot.lifter.setPower(0);
                    // bet this line below is why the home position code never worked - should get real motor encoder position
                    robot.setLifterMinimum(robot.lifter.getCurrentPosition());// Sets this position as the new "0" in terms of encoder count
                    robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    movingLifter = false;
                    firstHomeLift = false;
                    lifterLocation = lifterStates.Home;
                    targetLifterLocation = lifterStates.Manual; // ends the override using high button
                }*/
                else {
                    if (!robot.lifter.isBusy()) {
                        robot.lifter.setPower(0);
                        movingLifter = false;
                        lifterLocation = TelePowerPlayMeet1.lifterStates.Home;
                        targetLifterLocation = TelePowerPlayMeet1.lifterStates.Manual;
                    }
                }}
            else { // Manual lifter motion
                robot.lifter.setPower(lifterPower); // Performs safety checks internally
                if (Math.abs(lifterPower) > 0.1) { // Clear automated state if moving manually
                    if (lifterLocation != TelePowerPlayMeet1.lifterStates.Manual) {
                        lifterLocation = TelePowerPlayMeet1.lifterStates.Manual;
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    robot.lifter.setPower(lifterPower);//
                }
            }


            //******************************************************************************************

            if(gamepad2.dpad_down){
                if (!movingLifter) {
                    movingLifter = true;
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(1);
                    if(robot.lifter.getCurrentPosition()>-smalllift){
                        robot.lifter.setTargetPosition(robot.lifterMinimum);
                    }else{
                        robot.lifter.setTargetPosition((robot.lifter.getCurrentPosition()+smalllift));
                    }
                    targetLifterLocation = TelePowerPlayMeet1.lifterStates.Between;
                }
            }
            if(gamepad2.dpad_up){
                if (!movingLifter) {
                    movingLifter = true;
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(-1);
                    if(robot.lifter.getCurrentPosition()<((robot.lifterLevelThree)+smalllift)){
                        robot.lifter.setTargetPosition(robot.lifterLevelThree);
                    }else{
                        robot.lifter.setTargetPosition((robot.lifter.getCurrentPosition()-smalllift));
                    }
                    targetLifterLocation = TelePowerPlayMeet1.lifterStates.Between;
                }
            }
            //}if(gamepad2.left_bumper){
            //    robot.lifter.setTargetPosition(-513);
            //}if(gamepad2.left_trigger>0){
            //    robot.lifter.setTargetPosition(-513);
            //}

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
            telemetry.addData("Low: ", robot.lifterMinimum);
            // update telemetry of drive motors in order to figure out why the robot is not driving straight 11/8/22
            telemetry.addData("front left ticks", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("back left ticks", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("front right ticks", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("back right ticks", robot.backRightMotor.getCurrentPosition());

            //telemetry.addData("right encoder", robot.rightEncoder.getCurrentPosition());
            //telemetry.addData("left encoder", robot.leftEncoder.getCurrentPosition());
            //telemetry.addData("lateral encoder", robot.frontEncoder.getCurrentPosition());

            // flash freeze states
            telemetry.addData("slow mode?", lowSpeedActivated);
            telemetry.addData("super slow mode?", superLowSpeedActivated);

            // Update telemetry at end
            telemetry.update();
        }
    }
}