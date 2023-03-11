package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@Disabled
@TeleOp(name="servoTest", group="Iterative Opmode")

public class DriveBase extends OpMode {

    RobotPowerPlay robot = new RobotPowerPlay();
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    boolean intakeUp = true;
    boolean movingintake = false;
    boolean intakePressed = false;
    double initialin;
    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;

    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // controller variables
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * strafingConstant; // coefficient counteracts imperfect strafing
        double rx = gamepad1.right_stick_x;


        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = -y - x + rx;
        backRightPower = -y + x - rx;
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower   = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower   = Range.clip(backRightPower, -1.0, 1.0);

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);
        if (!movingintake) {
            if (gamepad1.left_bumper) {
                intakePressed = true;
                movingintake = true;
                initialin = getRuntime();
            }
        }
        if (intakePressed && movingintake) {
            if (!intakeUp) {
                robot.intake.setPosition(0);
            }
        else if (intakeUp) {
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

    }

    public void stop() {
        robot.stopAllMotors();
    }

}
