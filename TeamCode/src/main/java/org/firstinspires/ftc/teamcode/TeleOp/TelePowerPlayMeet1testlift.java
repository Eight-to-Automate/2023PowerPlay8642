package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp(name="Teletestliftermotor", group="Iterative Opmode")

public class TelePowerPlayMeet1testlift extends OpMode {
    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer
    double slowTime; // initial time for slowmode timeout
    double slowTime2; // initial time for super slowmode timeout
    double liftTimestart, liftTime;  // initial time for lifter timing

    double checkTimeL, checkTimeH;

    double bfrReset;

    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;
    double lifterPower;
    int smalllift=500;//400; //400 with faster motor
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

    //SHooting motor new PIDF values
    public  double NEW_P = 15;
    public double NEW_I = 3;
    public  double NEW_D = 1.5;
    public  double NEW_F = 14;  //was 12.6

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
        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        // PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        robot.lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);


        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

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
            frontLeftPower /= 0.8;
            frontRightPower /= 0.8;
            backLeftPower /= 0.8;
            backRightPower /= 0.8;}

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
        robot.setDrivePower(frontLeftPower * 0.8, frontRightPower * 0.8, backRightPower * 0.8, backLeftPower * 0.8);


        //******************************************************************************************

        // Lifter implementation

        if (getRuntime() > checkTimeL && getRuntime() < checkTimeH){
            if(robot.lifter.isBusy()) {
                robot.lifter.setPower(0);
            }
        }

        if (gamepad1.right_trigger > 0.5 && movingLifter){      // lifter override
            robot.lifter.setPower(0);
            movingLifter = false;
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
                    robot.lifter.setTargetPosition(robot.lifterY);   // Now using 20:1 motor was 6100 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftTimestart=runtime.milliseconds();
                    robot.lifter.setPower(-1.0);
                    targetLifterLocation = lifterStates.High;
                }
            }
        }
        else if (gamepad2.x) { // Middle Level
            if (!movingLifter|| targetLifterLocation == lifterStates.Home) {
                if (lifterLocation != lifterStates.Middle|| targetLifterLocation == lifterStates.Home) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterX); // May be changed later  Now using 20:1 motor was 4500 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterX) { // Set the power to match with the goal direction
                        robot.lifter.setPower(-1.0);
                    } else {
                        robot.lifter.setPower(1.0);
                    }
                    targetLifterLocation = lifterStates.Middle;
                    liftTimestart=runtime.milliseconds();
                }
            }
        }
        else if (gamepad2.a) {// Lower level
            if (!movingLifter|| targetLifterLocation == lifterStates.Home) {
                if (lifterLocation != lifterStates.Low|| targetLifterLocation == lifterStates.Home) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.lifterA); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.lifterA) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-1.0);
                    }
                    targetLifterLocation = lifterStates.Low;
                    liftTimestart=runtime.milliseconds();
                }
            }
        } else if (gamepad2.right_bumper) {// low junction level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Junction) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.drivingHeight); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.drivingHeight) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-1.0);
                    }
                    targetLifterLocation = lifterStates.Junction;
                    liftTimestart=runtime.milliseconds();
                }
            }
        } else if (gamepad2.left_bumper) {// stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.Stack) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.fourStack); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.fourStack) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.8);
                    }
                    targetLifterLocation = lifterStates.Stack;
                    liftTimestart=runtime.milliseconds();
                }
            }
        } else if (gamepad2.dpad_left) {// 2nd cone stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.secondCone) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.twoStack); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.twoStack) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.8);
                    }
                    targetLifterLocation = lifterStates.secondCone;
                    liftTimestart=runtime.milliseconds();
                }
            }
        } else if (gamepad2.dpad_right) {// 3rd cone stack level
            if (!movingLifter) {
                if (lifterLocation != lifterStates.thirdCone) {
                    movingLifter = true;
                    robot.lifter.setTargetPosition(robot.threeStack); // Now using 20:1 motor was 3000 with 40:1 motor.
                    robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (robot.lifter.getCurrentPosition() > robot.threeStack) { // Set the power to match with the goal direction
                        robot.lifter.setPower(1.0);
                    } else {
                        robot.lifter.setPower(-0.8);
                    }
                    targetLifterLocation = lifterStates.thirdCone;
                    liftTimestart=runtime.milliseconds();
                }
            }
        }

        if (movingLifter) { // Check if lifter is moving using driver enhancement (autonomous)
            if (targetLifterLocation != lifterStates.Home) {
                if (!robot.lifter.isBusy()) { // Check if lifter is finished reaching target position
                    robot.lifter.setPower(0); // Stop motion and state once it gets there
                    movingLifter = false;
                    lifterLocation = targetLifterLocation;
                    liftTime = runtime.milliseconds()-liftTimestart;
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
                    robot.lifter.setPower(0.3);  // was 0.2 with faster motor
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
                if(robot.lifter.getCurrentPosition()<((robot.lifterY)+smalllift)){
                    robot.lifter.setTargetPosition(robot.lifterY);
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
        telemetry.addData("Lifter time ", liftTime);
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
