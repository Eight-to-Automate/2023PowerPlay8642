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

import org.firstinspires.ftc.teamcode.TeleOp.TelePowerPlayMeet1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="[TEST]TelePowerPlayGuidedMovementTest", group="Iterative Opmode")

public class TelePowerPlayGuidedMovementTest extends LinearOpMode {
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


    double posX;
    double posY;
    double junctionX;
    double junctionY;
    double slowAreaRadius;
    double stopAreaRadius;
    double robotRadius;
    double heading;
    double joystickHeading;
    double movementHeading;
    double junctionDistanceX;
    double junctionDistanceY;
    double nudgeConstant;
    double junctionDeadzone;
    enum guidedMovement{
        groundJunction, poleJunctionActive, poleJunctionInactive, active, inactive
    }
    guidedMovement currGuidedMovementMode = guidedMovement.active;


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
        drive.setPoseEstimate(new Pose2d(62,-62,Math.toRadians(180))); //sets initial robot position at the Left Red terminal (bottom left corner) facing forwards

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        // drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {






            // controller variables
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x; //* strafingConstant; // coefficient counteracts imperfect strafing
            double rx = -gamepad1.right_stick_x;
            /*OLD CONTROLLER VARIABLES
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x; //* strafingConstant; // coefficient counteracts imperfect strafing
            double rx = -gamepad1.right_stick_x;
             */



            // double ry2 = gamepad2.left_stick_y;














            //Guided Movement


            //read roadrunner pose and update pose
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();



            //record x and y positions
            telemetry.addData("X: ", poseEstimate.getX());
            telemetry.addData("Y: ", poseEstimate.getY());
            telemetry.addData("Heading(degrees): ", Math.toDegrees(poseEstimate.getHeading()));


            //update values
            posX = poseEstimate.getX();
            posY = poseEstimate.getY();
            heading = Math.toDegrees(poseEstimate.getHeading());  //global robot heading    180 degrees is forward/up(|)  0 degrees is right -> just like a normal coordinate plane
            joystickHeading = Math.toDegrees(Math.atan((gamepad1.left_stick_y)/(gamepad1.left_stick_x)));  //angle of joystick which is equal to direction of movement from robot's POV
            movementHeading = heading + joystickHeading; //global robot movement heading   180 degrees is forward/up(|)  0 degrees is right -> just like a normal coordinate plane


            //return heading values
            telemetry.addData("Joystick Heading Angle (Degrees): ",joystickHeading);
            telemetry.addData("Global Movement Heading Angle(Degrees): ",movementHeading);

            //(47,-47) ground junction nearest to robot
            //junctionX = 47;
            // junctionY = -47;
//ALL JUNCTION POSITIONS-------------------------------------------------------------------------------------------
            //top right junction (47, 47)
            //top left junction (-47, 47)
            //bottom left junction (-47, -47)
            //bottom right junction(47, -47)

            //middle right junction (47, 0)
            //middle left junction (-47, 0)
            //middle top junction (0, 47)
            //middle bottom junction (0, -47)

            //center junction (0, 0)


            //Switch restricted junction based on which section of map robot is in



            //each tile is 70/3 inches by 70/3 inches wide


            if((posX >= -70 / 3.0) && (posX <= 70 / 3.0)){// middle junctions
                if((posY <= 70 / 3.0) && (posY >= -70 / 3.0)){// center junction
                    //(0, 0)
                    junctionX = 0;
                    junctionY = 0;
                }
                if(posY > 70 / 3.0){// top middle junction
                    //(0, 47)
                    junctionX = 0;
                    junctionY = 47;

                }
                if(posY < 70 / 3.0){// bottom middle junction
                    //(0, -47)
                    junctionX = 0;
                    junctionY = -47;

                }

            }
            if(posX > 70 / 3.0) {//right side junctions
                if((posY <= 70 / 3.0) && (posY >= -70 / 3.0)){// middle right junction
                    //(47, 0)
                    junctionX = 47;
                    junctionY = 0;

                }
                if(posY > 70 / 3.0){// top right junction
                    //(47, 47)
                    junctionX = 47;
                    junctionY = 47;

                }
                if(posY < 70 / 3.0){// bottom right junction
                    //(47, -47)
                    junctionX = 47;
                    junctionY = -47;

                }
            }
            if(posX < 70 / 3.0){// left side junctions
                if((posY <= 70 / 3.0) && (posY >= -70 / 3.0)){// middle left junction
                    //(-47, 0)
                    junctionX = -47;
                    junctionY = 0;

                }
                if(posY > 70 / 3.0){// top left junction
                    //(-47, 47)
                    junctionX = -47;
                    junctionY = 47;

                }
                if(posY < 70 / 3.0){// bottom left junction
                    //(-47, -47)
                    junctionX = -47;
                    junctionY = -47;

                }
            }
            telemetry.addData("JunctionX : ", junctionX);
            telemetry.addData("JunctionY : ", junctionY);






//------------------------------------------------------------------------------------------------------------------



            robotRadius = 7;//from approx center to corner of extrusion
            slowAreaRadius = 5.0 + (robotRadius);//area around the junction to slow down the robot
            stopAreaRadius = 4.0 + (robotRadius);//area around the junction the robot cannot enter
            junctionDistanceX = posX - junctionX;
            junctionDistanceY = posY - junctionY;


            //set nudge constant
            // how much the robot is nudged away from the junction robot just stops at the edge if set to 1
            nudgeConstant = 1;
            //no longer being used
            /*
            // clip range from 1 to 10 to prevent robot from zooming away
            nudgeConstant = Range.clip(nudgeConstant, 1, 10);
            */


            //set deadzone value where robot will just sit still instead of moving out of circle
            junctionDeadzone = 0.05;





            //If guide button is pressed, disable guided movement
            if(gamepad1.guide){
                currGuidedMovementMode = guidedMovement.inactive;
            }
            telemetry.addData("Guided Movement: ", currGuidedMovementMode);


            if(currGuidedMovementMode != guidedMovement.inactive) {
                //if guided movement is inactive then
                if ((posX < (slowAreaRadius + junctionX) && posX > (-slowAreaRadius + junctionX)) && (posY < (slowAreaRadius + junctionY) && posY > (-slowAreaRadius + junctionY))) {//if robot is within a 3x3 box around ground junction
                    telemetry.addData("Stop area activated!: ", true);
                    // Stop and reverse robot away from ground junction area if too close
                    if (posX >= junctionX) {//means is on right side of junction
                        // if robot is on the right side and above it (Quadrant 1 if centered around junction)
                        //find vector magnitudes perpendicular to current movement vector and pushes robot that direction
                        telemetry.addData("On right side of junction", true);
                        if (posY >= junctionY) {//if robot is on right side and above it
                            if (((movementHeading >= 180 && movementHeading <= 270)) || junctionDistanceX < stopAreaRadius || junctionDistanceY < stopAreaRadius) {// if robot is in Q 1
                                y += ((stopAreaRadius / junctionDistanceY) * y);// nudge robot up
                                x += ((stopAreaRadius / junctionDistanceX) * x);//nudge robot right
                                telemetry.addData("nudged UP and RIGHT", true);
                                if (Math.abs(x) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    x = 0;
                                }
                                if (Math.abs(y) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    y = 0;
                                }
                            }
                        } else if (posY < junctionY) {// if robot is in quadrant 4
                            if ((movementHeading >= 90 && movementHeading <= 180) || junctionDistanceX < stopAreaRadius || junctionDistanceY < stopAreaRadius) {// if robot is in Q
                                y += -((stopAreaRadius / junctionDistanceY) * y);//nudge robot down
                                x += ((stopAreaRadius / junctionDistanceX) * x);
                                if (Math.abs(x) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    x = 0;
                                }
                                if (Math.abs(y) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    y = 0;
                                }
                                telemetry.addData("nudged UP and DOWN", true);
                            }
                        }


                    } else if (posX < junctionX) {//means is on left side of junction

                        telemetry.addData("On left side of junction", true);
                        if (posY >= junctionY) {//if robot is on left side and above it
                            if ((movementHeading >= 270 && movementHeading <= 360) || movementHeading == 0 || junctionDistanceX < stopAreaRadius || junctionDistanceY < stopAreaRadius) {// takes into account rollback to zero
                                y += ((stopAreaRadius / junctionDistanceY) * y);// nudge robot up
                                x += -((stopAreaRadius / junctionDistanceX) * x); // nudge robot to the left
                                telemetry.addData("nudged UP and LEFT", true);
                                if (Math.abs(x) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    x = 0;
                                }
                                if (Math.abs(y) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    y = 0;
                                }
                            }
                        } else if (posY < junctionY) {// if robot is in quadrant 3
                            if ((movementHeading >= 0 && movementHeading <= 180) || movementHeading == 360 || junctionDistanceX < stopAreaRadius || junctionDistanceY < stopAreaRadius) {
                                y += -((stopAreaRadius / junctionDistanceY) * y);//nudge robot down
                                x += -((stopAreaRadius / junctionDistanceX) * x); // nudge robot to the left
                                telemetry.addData("nudged DOWN and LEFT", true);
                                if (Math.abs(x) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    x = 0;
                                }
                                if (Math.abs(y) <= junctionDeadzone && Math.abs(x) >= junctionDeadzone) {//if robot is being told to move slowly, prevent it from moving in that direction
                                    y = 0;
                                }
                            }
                        }

                    }

                }
            }



                /*} else {
                    telemetry.addData("is not in stop area: ", false);

                    //frontLeftPower *= ((posX - slowAreaRadius) / (slowAreaRadius + stopAreaRadius));
                    // frontRightPower *= ((posX - slowAreaRadius) / (slowAreaRadius + stopAreaRadius));
                    // backLeftPower *= ((posX - slowAreaRadius) / (slowAreaRadius + stopAreaRadius));
                    // backRightPower *= ((posX - slowAreaRadius) / (slowAreaRadius + stopAreaRadius));
                }*/


            /*} else {
                telemetry.addData("is not in slow area: ", false);
            }*/





            // Calculate motor power

            frontLeftPower = y + x + rx;
            frontRightPower = y - x - rx;
            backLeftPower = y - x + rx;
            backRightPower = y + x - rx;





            // Make sure driving power is -1 to 1 and set max/min values
            frontLeftPower = Range.clip(frontLeftPower, -1, 1);
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            backLeftPower = Range.clip(backLeftPower, -1, 1);
            backRightPower = Range.clip(backRightPower, -1, 1);

            // Set all the drive motors power
            robot.setDrivePower(frontLeftPower * -0.6, frontRightPower * 0.6, backRightPower * 0.6, backLeftPower * -0.6);








            //**********************************************************************************************************************
            // FlashFreeze

            // Toggle for FlashFreeze
            /*
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
            */

/*
            // Read pose
            //Pose2d poseEstimate = drive.getPoseEstimate();

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
*/
            robot.updateLightsTele(lowSpeedActivated, superLowSpeedActivated);

            // Update everything. Odometry. Etc.
            //drive.update();
/*
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            */


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

            /*

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
*/
            // Update telemetry at end
            telemetry.update();
        }
    }
}