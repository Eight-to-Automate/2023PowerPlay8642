package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
@Disabled
@TeleOp(name="TelePowerPlayLocalizer", group="Iterative Opmode")

public class TelePowerPlayLocalizer extends OpMode {
    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer
    double slowTime; // initial time for slowmode timeout
    double slowTime2; // initial time for super slowmode timeout

    StandardTrackingWheelLocalizer myLocalizer;

    double checkTimeL, checkTimeH;

    double bfrReset;

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
        Home, Low, Middle, High, Manual, Junction, Stack, Between, secondCone, thirdCone
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
    boolean firstRun = true;

    // Exponential drive values
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }


    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        robot.resetDriveEncoders();
        // run using encoders makes it slower but drive very straight
        robot.startDriveEncoders();
        //robot.startDriveEncoderless();
        // robot.lifter.setMode(DcMotorEx.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
        robot.lifter.setTargetPositionTolerance(15);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        if (firstRun) {
            myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

            // Set your initial pose to x: 10, y: 10, facing 90 degrees
            myLocalizer.setPoseEstimate(new Pose2d(-35.7,-62.7, Math.toRadians(90)));

            firstRun = false;
        }

        // controller variables
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x; //* strafingConstant; // coefficient counteracts imperfect strafing
        double rx = -gamepad1.right_stick_x;
        double ry2 = gamepad2.left_stick_y;

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


        // REDUCED TURNING SPEED
        if (lowSpeedActivated) {
            rx = rx/1.5;
        }

        if (superLowSpeedActivated) rx /= 2.5;


        //****************************************************************************************
        // Calculate motor power

        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = y - x + rx;
        backRightPower = y + x - rx;

        lifterPower = ry2;


        // Make sure driving power is -1 to 1 and set max/min values
        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backLeftPower = Range.clip(backLeftPower, -1, 1);
        backRightPower = Range.clip(backRightPower, -1, 1);
        //lifterPower = Range.clip(lifterPower, -1, 1);


        if (robot.lifterSwitchTriggered()) lifterPower = Range.clip(lifterPower, -1, 0);
        else lifterPower = Range.clip(lifterPower, -1, 1);



        // Reducing power for each drive motor to one third of its original power for flash freeze
        if (lowSpeedActivated) { // was divison by 1.5 - 2/3, now times .8
            frontLeftPower /= 0.6;
            frontRightPower /= 0.6;
            backLeftPower /= 0.6;
            backRightPower /= 0.6;}

        else if (superLowSpeedActivated){
            frontLeftPower /= 2;
            frontRightPower /= 2;
            backLeftPower /= 2;
            backRightPower /= 2;
        }
        //    robot.setLightsState(RobotPowerPlay.lightsStates.Red);
        //}else{
        //    robot.setLightsState(RobotPowerPlay.lightsStates.Green);
        //}
        robot.updateLightsTele(lowSpeedActivated, superLowSpeedActivated);
        //******************************************************************************************

        // Set all the drive motors power
        robot.setDrivePower(frontLeftPower * 0.6, frontRightPower * 0.6, backRightPower * 0.6, backLeftPower * 0.6);


        //******************************************************************************************

        // Lifter implementation

        if (getRuntime() > checkTimeL && getRuntime() < checkTimeH){
            if(robot.lifter.isBusy()) {
                robot.lifter.setPower(0);
            }
        }

        if (gamepad2.b) { // Home Position
            if (!movingLifter) {
                //robot.storage.setPosition(0); // closes storage automatically - caused issues sometimes
                /*
                if (lifterLocation != lifterStates.Home) {
                        movingLifter = true;
                        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.lifter.setTargetPosition(robot.lifterMinimum);
                        robot.lifter.setPower(1);
                        checkTimeL = getRuntime() + 3000;
                        checkTimeH = checkTimeL + 200;
                        targetLifterLocation = lifterStates.Home;
                }
                 */

                if (lifterLocation != lifterStates.Home) {
                    if (!robot.lifterSwitch1.getState() && !robot.lifterSwitch2.getState()) { // do not try to move further down if switch is activated
                        movingLifter = true;
                        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.lifter.setPower(1);
                        targetLifterLocation = lifterStates.Home;
                    } else { // Set the lifter to the home state if it is at home
                        robot.lifter.setPower(0);
                        lifterLocation = lifterStates.Home;
                    }
                }
            }
        }
        else if (gamepad2.y) { // Highest Level
            // Checks we aren't already autonomously moving the lifter. If we are going to home, let it through as an override
            if (!movingLifter || targetLifterLocation == lifterStates.Home) {
                if (lifterLocation != lifterStates.High || targetLifterLocation == lifterStates.Home) { // Don't go to a currently set state
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterLevelThree);   // Now using 20:1 motor was 6100 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterLevelThree) { // Set the power to match with the goal direction
                        robot.lifter.setPower(-1.0);
                    } else { //lifter is going down
                        robot.lifter.setPower(0.7);
                    }
                    targetLifterLocation = lifterStates.High;
                }
            }
        }
        else if (gamepad2.x) { // Middle Level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Middle) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterLevelTwo); // May be changed later  Now using 20:1 motor was 4500 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterLevelTwo) { // Set the power to match with the goal direction
                        robot.lifter.setPower(-1.0);
                    } else { //lifter is going down
                        robot.lifter.setPower(0.7);
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterLevelOne) { // Set the power to match with the goal direction
                        robot.lifter.setPower(-1.0);
                    } else {
                        robot.lifter.setPower(0.7);
                    }
                    targetLifterLocation = lifterStates.Low;
                }
            }
        } else if (gamepad2.right_bumper) {// low junction level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Junction) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lowJunctionPos); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lowJunctionPos) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.7);
                    }
                    targetLifterLocation = lifterStates.Junction;
                }
            }
        } else if (gamepad2.left_bumper) {// stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Stack) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.stackPos); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.stackPos) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.7);
                    }
                    targetLifterLocation = lifterStates.Stack;
                }
            }
        } else if (gamepad2.dpad_left) {// 2nd cone stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.secondCone) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.secondCone); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.secondCone) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.7);
                    }
                    targetLifterLocation = lifterStates.secondCone;
                }
            }
        } else if (gamepad2.dpad_right) {// 3rd cone stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.thirdCone) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.thirdCone); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.thirdCone) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.7);
                    }
                    targetLifterLocation = lifterStates.thirdCone;
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
            } else { // Check if it is for home position
                if (robot.lifterSwitchTriggered()) { // Stops if limit switch is pressed
                    robot.lifter.setPower(0);
                    // bet this line below is why the home position code never worked - should get real motor encoder position
                    bfrReset = robot.lifter.getCurrentPosition();
                    robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    double timeS = runtime.milliseconds();
                    while (runtime.milliseconds() < timeS + 20){}

                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    movingLifter = false;
                    firstHomeLift = false;
                    lifterLocation = lifterStates.Home;
                    targetLifterLocation = lifterStates.Manual; // ends the override using high button
                }
                else if (robot.lifter.getCurrentPosition() > -500){
                    robot.lifter.setPower(0.2);
                }
                /*else {
                    if (!robot.lifter.isBusy()) {
                        robot.lifter.setPower(0);
                        movingLifter = false;
                        lifterLocation = lifterStates.Home;
                        targetLifterLocation = lifterStates.Manual;
                    }
                }*/
            }
        } else { // Manual lifter motion
            robot.lifter.setPower(lifterPower); // Performs safety checks internally
            if (Math.abs(lifterPower) > 0.1) { // Clear automated state if moving manually
                if (lifterLocation != lifterStates.Manual) {
                    lifterLocation = lifterStates.Manual;
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
                targetLifterLocation = lifterStates.Between;
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
                targetLifterLocation = lifterStates.Between;
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

        myLocalizer.update();

        // Retrieve your pose
        Pose2d myPose = myLocalizer.getPoseEstimate();

        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());

        telemetry.addData("Lifter Power: ", lifterPower);
        telemetry.addData("Lifter ticks before reset", bfrReset);
        telemetry.addData("current Lifter Ticks: ", robot.lifter.getCurrentPosition());
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

        telemetry.addData("At home? ", lifterLocation == lifterStates.Home);
        telemetry.addData("Limit switches activated? ", robot.lifterSwitchTriggered());

        // Update telemetry at end
        telemetry.update();

    }

    public void stop() {
        robot.stopAllMotors();
    }

}
