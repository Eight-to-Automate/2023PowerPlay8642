package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

import java.util.ArrayList;

@TeleOp(name="TelePowerPlayPremoveTest", group="Iterative Opmode")

public class TelePowerPlayPremoveTest extends OpMode {
    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer
    double slowTime; // initial time for slowmode timeout
    double slowTime2; // initial time for super slowmode timeout

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
        Home, Low, Middle, High, Manual, Junction, Stack
    }
    //TODO: Premove states
    enum driveMode{
        idle, premove, joystick, nudge
    }
    enum preMove{
        Forward, Backward, Left, Right, TLeft, TRight
    }

    driveMode currStateDrive = driveMode.idle;

    ArrayList<preMove> order = new ArrayList<>();

    int FLtarget;
    int FRtarget;
    int BLtarget;
    int BRtarget;
    boolean start = true;



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

        //TODO: Joystick Controller Override **********************************************************
        if((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) ) {
            //if detects gamepad input
            driveMode currStateDrive = driveMode.joystick;
            order.clear();
            robot.startDriveEncoderless(); //Prevent robot from continuing to hold its position after premove due to RUN_TO_POSITION
            telemetry.addData("DriveMode", "Joystick");

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
            } else if (getRuntime() - slowTime > 0.3) {
                leftBumperDown = false;
            }

            //activate or deactivate super slow speed when left trigger receives any input (code includes debounce as well)
            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                leftTriggerPressed = true;
                superLowSpeedActivated = !superLowSpeedActivated;
                lowSpeedActivated = false;
                slowTime2 = getRuntime();
            } else if (getRuntime() - slowTime2 > 0.3) {
                leftTriggerPressed = false;
            }


            // REDUCED TURNING SPEED
            if (lowSpeedActivated) {
                rx = rx / 1.5;
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
            lifterPower = Range.clip(lifterPower, -1, 1);

            // Reducing power for each drive motor to one third of its original power for flash freeze
            if (lowSpeedActivated) { // was divison by 1.5 - 2/3, now times .8
                frontLeftPower /= 0.6;
                frontRightPower /= 0.6;
                backLeftPower /= 0.6;
                backRightPower /= 0.6;
            } else if (superLowSpeedActivated) {
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

        }
        //TODO: Premove Queues****************************************************************************************
        //detect dpad input and queue in arraylist
        if (gamepad1.dpad_up) {
            order.add(preMove.Forward);
        }
        if (gamepad1.dpad_down) {
            order.add(preMove.Backward);
        }
        if (gamepad1.dpad_left) {
            order.add(preMove.Left);
        }
        if (gamepad1.dpad_right) {
            order.add(preMove.Right);
        }
        if (order.isEmpty()) {
        }
        if(currStateDrive != driveMode.premove && !order.isEmpty() && currStateDrive != driveMode.joystick){
            //if not currently premoving and order is not empty and is not in joystick mode
            //get first element in array and execute command
            if(order.get(0) == preMove.Forward){
                order.remove(0);//remove first element and shift all other elements index down 1
                TeleGoDistance(60,0.4);//forward 1 tile (60cm per tile)
            }
            if(order.get(0) == preMove.Backward){
                order.remove(0);
                TeleGoDistance(-60,0.4);
            }/* TODO: implement strafe if new telegodistance method works
            if(order.get(0) == preMove.Left){
                order.remove(0);

            }
            if(order.get(0) == preMove.Right){
                order.remove(0);

            }*/


        }
        if (start || (robot.frontLeftMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) || (FLtarget != robot.frontLeftMotor.getCurrentPosition()) && (FRtarget != robot.frontRightMotor.getCurrentPosition()) && (BLtarget != robot.backLeftMotor.getCurrentPosition()) && (BRtarget != robot.backRightMotor.getCurrentPosition())){
            currStateDrive = driveMode.premove;
            //if robot is not currently at the position it should be at then it is currently in premove and is not in RUN_WITH_ENCODER mode
        }else{
            currStateDrive = driveMode.idle;
        }


























        //****************************************************************************************














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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lowJunctionPos) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-1.0);
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
                        robot.lifter.setPower(-1.0);
                    }
                    targetLifterLocation = lifterStates.Stack;
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
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
                robot.lifter.setPower(lifterPower);//
            }
        }


        //******************************************************************************************
        //if(gamepad2.right_trigger>0){
        //    robot.lifter.setTargetPosition(-350);
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

        // update telemetry of drive motors in order to figure out why the robot is not driving straight 11/8/22
        telemetry.addData("front left ticks", robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("back left ticks", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("front right ticks", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("back right ticks", robot.backRightMotor.getCurrentPosition());

        // flash freeze states
        telemetry.addData("slow mode?", lowSpeedActivated);
        telemetry.addData("super slow mode?", superLowSpeedActivated);

        // Update telemetry at end
        telemetry.update();

    }
    //TODO: GO DISTANCE METHOD ***************************************************
    public void TeleGoDistance(double centimeters, double power) {
        // holds the conversion factor for TICKS to centimeters
        final double conversionFactor = 17.59; // Number came from testing, may need to be improved
        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversionFactor);

        robot.resetDriveEncoders();

        //Calculate the target for each specific motor
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;

        start = false;






        robot.setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);

        robot.startDriveEncodersTarget();

        robot.setDrivePower(power, power, power, power);

    }
















    public void stop() {
        robot.stopAllMotors();
    }

}
