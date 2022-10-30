package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        Home, Low, Middle, High, Manual
    }

    // Setup booleans for state machines
    boolean lowSpeedActivated = false;
    boolean leftBumperDown = false;


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
            frontLeftPower *= .4;
            frontRightPower *= .4;
            backLeftPower *= .4;
            backRightPower *= .4;
        }

        //******************************************************************************************

        // Set all the drive motors power
        robot.setDrivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);


        //******************************************************************************************

        // Lifter implementation

        /* TODO:
        preset buttons for location
        nudge using bumpers
        joystick that controls power
         */

        robot.lifter.setPower(lifterPower);

        telemetry.addData("Lifter Power: ", lifterPower);
        telemetry.addData("Lifter Ticks: ", robot.lifter.getCurrentPosition());

        // Update telemetry at end
        telemetry.update();

    }

    public void stop() {
        robot.stopAllMotors();
    }

}
