/*
Premove Joystick Nudge program
Authors:Nathan, Chethas
Start Date: 9/10/22
End Date:[IN PROGRESS]
Description: Allows you to move robot using the dpad to premove and queue multiple inputs and have the robot execute them in the order
they were inputted.  Moving one of joysticks overrides the premove function and clears all inputted commands.
The X Y A B right bumper and left bumper buttons
allow you to nudge the robot in order to move the robot back to the middle of the tile to premove again.





Checkpoint 9/24/22: Rewrote nudge dpad and joystick functions.  Fixed Dpad, joystick and nudge functions
 interfering each other and also fixed joystick functions.


*/
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name="premoveNoWhileLoops", group="testing")

public class premoveNoWhileLoops extends OpMode {

    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();


    //boolean newcommand =true;
    //double ctime=0;


    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;

    //Set up variables for drivebase and freeze frame
    boolean lowSpeedActivated = false;
    boolean driverAButtonDown = false;
    boolean bumperPressed = false;

    //Swap drive modes
    enum DriveMode{
        Dpad, Joystick
    }
    DriveMode currstateDrive = DriveMode.Dpad;

    enum SwitchDriveState{
        Active, Inactive
    }
    SwitchDriveState currstateSwitchDriveState = SwitchDriveState.Active;

    //Nudge enabled / disabled
    boolean Nudge = false;


    //Adds moves to the queue
    enum Move {
        Front, Back, Left, Right, TLeft, TRight
    }

    //ArrayList<Move> order = new ArrayList<Move>();
    Queue<Move> order = new LinkedList<>();

    /*
    Active: can press the button and make a premove
    Inactive: need to wait until the button is released before you can premove a specific button again
    If you want to premove front + left, you can press the left button before you release the front button
    */
    enum Left{
        Active, Inactive
    }
    Left currStateL = Left.Active;

    enum Right{
        Active, Inactive
    }
    Right currStateR = Right.Active;

    enum Front{
        Active, Inactive
    }
    Front currStateF = Front.Active;

    enum Back{
        Active, Inactive
    }
    Back currStateB = Back.Active;

    enum TRight{
        Active, Inactive
    }
    TRight currStateTR = TRight.Active;

    enum TLeft{
        Active, Inactive
    }
    TLeft currStateTL = TLeft.Active;


//exponential drive variables
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }
    //-----------------------No while loop variables------------------------------------------------------------

    boolean telegodistanceSLoop = false;
    boolean RotateDEGLoop = false;
    int RotateDEGticks;
    boolean strafeDistanceCM3SLoop = false;















//----------------------------------------------------------------------------------------------
    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        //  robot.lifter.setMode(DcMotor.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

        //---------------------------------------------------------
//OLD USELESS gamepad1.dpad_up && gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_right && currstateDrive == DriveMode.Dpad


        // Drivemode enum states
        if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0) && currstateDrive == DriveMode.Dpad && currstateSwitchDriveState == SwitchDriveState.Active){
            currstateDrive = DriveMode.Joystick;
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currstateSwitchDriveState = SwitchDriveState.Inactive;
            order.clear();
            telemetry.addData("DriveMode", "Joystick");
            telemetry.update();
        }
        else if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y ==0 && gamepad1.right_stick_x ==0 && gamepad1.right_stick_y ==0 && currstateDrive == DriveMode.Joystick && currstateSwitchDriveState == SwitchDriveState.Active){
            currstateDrive = DriveMode.Dpad;
            currstateSwitchDriveState = SwitchDriveState.Inactive;
            telemetry.addData("DriveMode", "Dpad");
            telemetry.update();
        }

        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y ==0 && gamepad1.right_stick_x ==0 && gamepad1.right_stick_y ==0 && currstateSwitchDriveState == SwitchDriveState.Inactive){
            currstateSwitchDriveState = SwitchDriveState.Active;
        }

        //Joystick Controls -------------------------------------------------------------------
        if (currstateDrive == DriveMode.Joystick) {
            // controller variables
            double y = exponential(gamepad1.left_stick_y, 1);
            double x = exponential(-gamepad1.left_stick_x, 1); //* strafingConstant; // coefficient counteracts imperfect strafing
            double rx = exponential(-gamepad1.right_stick_x, 1);

            double ry2 = gamepad2.left_stick_y;

            // Toggle for FlashFreeze
            if (gamepad1.left_bumper) {
                if (!driverAButtonDown) {
                    driverAButtonDown = true;
                    lowSpeedActivated = !lowSpeedActivated;

                }
            } else {
                driverAButtonDown = false;
            }

            // REDUCED TURNING SPEED
            if (lowSpeedActivated) {
                rx = rx/1.5;
            }

            // Convert controller variables into motor powers in holonomic drive
        /*
        if (lowSpeedActivated) { // if freezframe mode is on (slo mo), then reduce turning speed even more
            frontLeftPower = y + x + rx / 1.5;
            frontRightPower = y - x - rx / 1.5;
            backLeftPower = y - x + rx / 1.5;
            backRightPower = y + x - rx / 1.5;
        } else {
        */

            frontLeftPower = y + x + rx;
            frontRightPower = y - x - rx;
            backLeftPower = y - x + rx;
            backRightPower = y + x - rx;
            //}



            // Make sure driving power is -1 to 1 and set max/min values
            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);


            // Reducing power for each drive motor to one third of its original power for flash freeze
            if (lowSpeedActivated) { // was divison by 1.5 - 2/3, now times .8
                frontLeftPower *= .4;
                frontRightPower *= .4;
                backLeftPower *= .4;
                backRightPower *= .4;
            }


            // Set all the drive motors power
            robot.setDrivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);
//        robot.frontLeftMotor.setPower(frontLeftPower);
//        robot.frontRightMotor.setPower(frontRightPower);
//        robot.backLeftMotor.setPower(backLeftPower);
//        robot.backRightMotor.setPower(backRightPower);
            telemetry.update();









            //OLD JOYSTICK CONTROLS______________________________________________________________________________________
            /*
            // controller variables
            double y = exponential(-gamepad1.left_stick_y, 1);
            double x = exponential(-gamepad1.left_stick_x, 1); //* strafingConstant; // coefficient counteracts imperfect strafing
            double rx = exponential(-gamepad1.right_stick_x, 1);


            frontLeftPower = y + x + rx;
            frontRightPower = y - x - rx;
            backLeftPower = y - x + rx;
            backRightPower = y + x - rx;
            //}


            // Set all the drive motors power
            robot.setDrivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);

*/
        }
/*
//old joystick
if (currstateDrive == DriveMode.Joystick){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * strafingConstant; // coefficient counteracts imperfect strafing
            double rx = gamepad1.right_stick_x;


            frontLeftPower = y + x + rx;
            frontRightPower = y - x - rx;
            backLeftPower = -y - x + rx;
            backRightPower = -y + x - rx;
            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);

            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.backRightMotor.setPower(backRightPower);

        }*/

        //----------------------------------------------------------------------------------------


        //Add commands to the queue and also button debounce
        if (gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down && currStateL == Left.Active) {
            order.add(Move.Left);
            currStateL = Left.Inactive;
        }
        if (gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad1.dpad_down && currStateR == Right.Active){
            order.add(Move.Right);
            currStateR = Right.Inactive;
        }
        if (gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left && currStateF == Front.Active) {
            order.add(Move.Front);
            currStateF = Front.Inactive;
        }
        if (gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left && currStateB == Back.Active) {
            order.add(Move.Back);
            currStateB = Back.Inactive;
        }
        /*
        if (gamepad1.right_bumper && currStateTR == TRight.Active){
            order.add(Move.TRight);

        if (gamepad1.left_bumper && currStateTL == TLeft.Active){
            order.add(Move.TLeft);
        }
*/
        if (!gamepad1.dpad_left && currStateL == Left.Inactive) {
            currStateL = Left.Active;
        }
        if (!gamepad1.dpad_right && currStateR == Right.Inactive){
            currStateR = Right.Active;
        }
        if (!gamepad1.dpad_up && currStateF == Front.Inactive) {
            currStateF = Front.Active;
        }
        if (!gamepad1.dpad_down && currStateB == Back.Inactive) {
            currStateB = Back.Active;
        }
        /*
        if (!gamepad1.left_bumper && currStateTL == TLeft.Inactive) {
            currStateTL = TLeft.Active;
        }
        if (!gamepad1.left_bumper && currStateTR == TRight.Inactive) {
            currStateTR = TRight.Active;
        }
*/
/*
            if (order.size() != 0){
                Move nextCommand = order.peek();
                if (nextCommand == Move.Left) {
                    robot.strafeDistanceCM3(-60, 0.4, false);
                    order.remove();
                }
                if (nextCommand == Move.Right){
                    robot.strafeDistanceCM3(60, 0.4, false);
                    order.remove();
                }
                if (nextCommand == Move.Front){
                    robot.teleGoDistance(60, 0.4, false);
                    order.remove();
                }
                if (nextCommand == Move.Back){
                    robot.teleGoDistance(-60, 0.4, false);
                    order.remove();
                }
            }

            */

        //if none of the dpad buttons are pressed execute the commands
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && currstateDrive == DriveMode.Dpad) {

            if (order.peek() != null && currstateDrive == DriveMode.Dpad) {
                if (order.peek() == Move.Left && currstateDrive == DriveMode.Dpad) {
                    strafeDistanceCM3S(60, 0.4, false);
                    order.poll();
                    telemetry.addData("Movement", "Left");
                } else if (order.peek() == Move.Right && currstateDrive == DriveMode.Dpad) {
                    strafeDistanceCM3S(-60, 0.4, false);
                    order.poll();
                    telemetry.addData("Movement", "Right");
                } else if (order.peek() == Move.Front && currstateDrive == DriveMode.Dpad) {
                    teleGoDistanceS(60, 0.4, false);
                    order.poll();
                    telemetry.addData("Movement", "Front");
                } else if (order.peek() == Move.Back && currstateDrive == DriveMode.Dpad) {
                    teleGoDistanceS(-60, 0.4, false);
                    order.poll();
                    telemetry.addData("Movement", "Back");
                }else{
                    order.clear();
                }
                telemetry.update();
            }

        }
        if (currstateDrive != DriveMode.Dpad){
            order.clear();
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        if (telegodistanceSLoop && (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {


            //Stop motors if on Joystick mode
            //teleGoDistanceS

            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //break;
                telegodistanceSLoop = false;
            }


            //check for button inputs while the robot is driving
            if (gamepad1.dpad_left && currStateL == Left.Active) {
                order.add(Move.Left);
                currStateL = Left.Inactive;
            }
            if (gamepad1.dpad_right && currStateR == Right.Active){
                order.add(Move.Right);
                currStateR = Right.Inactive;
            }
            if (gamepad1.dpad_up && currStateF == Front.Active) {
                order.add(Move.Front);
                currStateF = Front.Inactive;
            }
            if (gamepad1.dpad_down && currStateB == Back.Active) {
                order.add(Move.Back);
                currStateB = Back.Inactive;
            }
           /* if (gamepad1.right_bumper && currStateTR == TRight.Active){
                order.add(Move.TRight);
            }
            if (gamepad1.left_bumper && currStateTL == TLeft.Active){
                order.add(Move.TLeft);
            }*/

            if (!gamepad1.dpad_left && currStateL == Left.Inactive) {
                currStateL = Left.Active;
            }
            if (!gamepad1.dpad_right && currStateR == Right.Inactive){
                currStateR = Right.Active;
            }
            if (!gamepad1.dpad_up && currStateF == Front.Inactive) {
                currStateF = Front.Active;
            }
            if (!gamepad1.dpad_down && currStateB == Back.Inactive) {
                currStateB = Back.Active;
            }
           /* if (!gamepad1.left_bumper && currStateTL == TLeft.Inactive) {
                currStateTL = TLeft.Active;
            }
            if (!gamepad1.left_bumper && currStateTR == TRight.Inactive) {
                currStateTR = TRight.Active;
            }*/
            if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0) && currstateDrive == DriveMode.Dpad && currstateSwitchDriveState == SwitchDriveState.Active){
                currstateDrive = DriveMode.Joystick;
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //break;
                telegodistanceSLoop = false;
            }
            telegodistanceSLoop = false;
        }

        if(RotateDEGLoop && (Math.abs(robot.frontLeftMotor.getCurrentPosition()) < RotateDEGticks && Math.abs(robot.frontRightMotor.getCurrentPosition()) < RotateDEGticks && Math.abs(robot.backLeftMotor.getCurrentPosition()) < RotateDEGticks && Math.abs(robot.backRightMotor.getCurrentPosition()) < RotateDEGticks)) {
            //Stop motors if on Joystick mode
            //RotateDEG
            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //break;
                RotateDEGLoop = false;
            }
        }


        if(strafeDistanceCM3SLoop && (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {
            //Stop motors if on Joystick mode
            //strafeDistanceCM3S
            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();

            }

            //check for button inputs while the robot is driving
            if (gamepad1.dpad_left && currStateL == Left.Active) {
                order.add(Move.Left);
                currStateL = Left.Inactive;
            }
            if (gamepad1.dpad_right && currStateR == Right.Active){
                order.add(Move.Right);
                currStateR = Right.Inactive;
            }
            if (gamepad1.dpad_up && currStateF == Front.Active) {
                order.add(Move.Front);
                currStateF = Front.Inactive;
            }
            if (gamepad1.dpad_down && currStateB == Back.Active) {
                order.add(Move.Back);
                currStateB = Back.Inactive;
            }


            if (!gamepad1.dpad_left && currStateL == Left.Inactive) {
                currStateL = Left.Active;
            }
            if (!gamepad1.dpad_right && currStateR == Right.Inactive){
                currStateR = Right.Active;
            }
            if (!gamepad1.dpad_up && currStateF == Front.Inactive) {
                currStateF = Front.Active;
            }
            if (!gamepad1.dpad_down && currStateB == Back.Inactive) {
                currStateB = Back.Active;
            }


            if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0) && currstateDrive == DriveMode.Dpad && currstateSwitchDriveState == SwitchDriveState.Active){
                currstateDrive = DriveMode.Joystick;
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                strafeDistanceCM3SLoop = false;
            }

        }



    }//end of main loop

    //methods

    public void teleGoDistanceS(int centimeters, double power, boolean handoff) {
        //****************************************************************************
        telegodistanceSLoop = true;
        //NOWHILELOOP

        //double conversion_factor = 31.3;  old conversion factor using 3x3x3 cartridges on the drive motor
        double conversion_factor = 17.59;  // new conversion factor using 4x5 gear cartridges
        //This method is used for TeleOp
        //was 31.3 for 3 3:1 cartridges

        boolean backwards = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);

        robot.resetDriveEncoders();

        if (backwards) {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;
        }
        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        if (backwards) {
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(-power);
        } else {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(power);
            robot.backLeftMotor.setPower(power);
        }


        // keep looping while we are still active, and there is time left, and all motors are running.
        /*while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {


            //Stop motors if on Joystick mode
            //teleGoDistanceS

            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }


            //check for button inputs while the robot is driving
            if (gamepad1.dpad_left && currStateL == Left.Active) {
                order.add(Move.Left);
                currStateL = Left.Inactive;
            }
            if (gamepad1.dpad_right && currStateR == Right.Active){
                order.add(Move.Right);
                currStateR = Right.Inactive;
            }
            if (gamepad1.dpad_up && currStateF == Front.Active) {
                order.add(Move.Front);
                currStateF = Front.Inactive;
            }
            if (gamepad1.dpad_down && currStateB == Back.Active) {
                order.add(Move.Back);
                currStateB = Back.Inactive;
            }


            if (!gamepad1.dpad_left && currStateL == Left.Inactive) {
                currStateL = Left.Active;
            }
            if (!gamepad1.dpad_right && currStateR == Right.Inactive){
                currStateR = Right.Active;
            }
            if (!gamepad1.dpad_up && currStateF == Front.Inactive) {
                currStateF = Front.Active;
            }
            if (!gamepad1.dpad_down && currStateB == Back.Inactive) {
                currStateB = Back.Active;
            }

            if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0) && currstateDrive == DriveMode.Dpad && currstateSwitchDriveState == SwitchDriveState.Active){
                currstateDrive = DriveMode.Joystick;
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }

        }*/

        if (!handoff) robot.stopDriveMotors();
        // startDriveEncoders();

    }

    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode, boolean TelemetryOn) {
        RotateDEGLoop = true;

        final double conversionFactor = 9.6; // for outreach robot: 8.46, for FreightFrenzy robot: 9.6

        if (degrees < 0 && power > 0) power = -power;

        int ticks = (int) Math.abs(Math.round(degrees * conversionFactor));
        RotateDEGticks = ticks;


        //if (TelemetryOn) {
        //    systemTools.telemetry.addData("Status", "Resetting Encoders");
        //    systemTools.telemetry.update();
        //}
//
        robot.resetDriveEncoders();

        //if (TelemetryOn) {
        //    systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //            robot.frontLeftMotor.getCurrentPosition(),
        //            robot.frontRightMotor.getCurrentPosition(),
        //            robot.backLeftMotor.getCurrentPosition(),
        //            robot.backRightMotor.getCurrentPosition());
        //    systemTools.telemetry.update();
        //}
        //Conversions to rotate
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = robot.backRightMotor.getCurrentPosition() - ticks;

        robot.startDriveEncoders();
        //Starts to rotate
        // setDrivePower(power, -power, -power, power);
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);


        /*while (linearOpMode.opModeIsActive() &&
                (Math.abs(robot.frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(robot.frontRightMotor.getCurrentPosition()) < ticks && Math.abs(robot.backLeftMotor.getCurrentPosition()) < ticks && Math.abs(robot.backRightMotor.getCurrentPosition()) < ticks)) {
            //Stop motors if on Joystick mode
            //RotateDEG
            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }
        }*/

        robot.stopDriveMotors();

        robot.startDriveEncoders();

        //if (TelemetryOn) {
        //    systemTools.telemetry.addData("Path", "Complete");
        //    systemTools.telemetry.addData("counts", ticks);
        //    systemTools.telemetry.update();
        //}

    }

    //methods

    public void strafeDistanceCM3S(int centimeters, double power, boolean handoff) {
        strafeDistanceCM3SLoop = true;
        //****************************************************************************
        //   This method is used for strafing a controlled distance in Teleop mode
        //
        //double conversion_factor = 31.3;  old conversion factor using 3x3x3 cartridges on the drive motor
        double conversion_factor = 17.59;  // new conversion factor using 4x5 gear cartridges
        //This method is used for TeleOp
        //was 31.3 for 3 3:1 cartridges

        boolean left = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);

        robot.resetDriveEncoders();

        if (left) {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;
        }
        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        if (left) {
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
        } else {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(power);
            robot.backLeftMotor.setPower(-power);
        }



        // keep looping while we are still active, and there is time left, and all motors are running.

        /*while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            //Stop motors if on Joystick mode
            //strafeDistanceCM3S
            if (currstateDrive == DriveMode.Joystick && (gamepad1.right_stick_x !=0 || gamepad1.right_stick_y != 0|| gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)){
                robot.stopDriveMotors();
                order.clear();
                return;
            }

            //check for button inputs while the robot is driving
            if (gamepad1.dpad_left && currStateL == Left.Active) {
                order.add(Move.Left);
                currStateL = Left.Inactive;
            }
            if (gamepad1.dpad_right && currStateR == Right.Active){
                order.add(Move.Right);
                currStateR = Right.Inactive;
            }
            if (gamepad1.dpad_up && currStateF == Front.Active) {
                order.add(Move.Front);
                currStateF = Front.Inactive;
            }
            if (gamepad1.dpad_down && currStateB == Back.Active) {
                order.add(Move.Back);
                currStateB = Back.Inactive;
            }


            if (!gamepad1.dpad_left && currStateL == Left.Inactive) {
                currStateL = Left.Active;
            }
            if (!gamepad1.dpad_right && currStateR == Right.Inactive){
                currStateR = Right.Active;
            }
            if (!gamepad1.dpad_up && currStateF == Front.Inactive) {
                currStateF = Front.Active;
            }
            if (!gamepad1.dpad_down && currStateB == Back.Inactive) {
                currStateB = Back.Active;
            }


            if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0) && currstateDrive == DriveMode.Dpad && currstateSwitchDriveState == SwitchDriveState.Active){
                currstateDrive = DriveMode.Joystick;
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }

        }*/

        if (!handoff) robot.stopDriveMotors();
        // startDriveEncoders();

    }
    public void stop() {
        robot.stopAllMotors();
        order.clear();
    }

}