package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

import java.lang.reflect.Array;
import java.util.*;

@TeleOp(name="premoveAwakened", group="testing")

public class premoveAwakened extends OpMode {

    RobotPowerPlay robot = new RobotPowerPlay();

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    //exponential drive variables
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }

    enum Move{
        Front, Back, Left, Right, TLeft, TRight
    }

    ArrayList<Move> order = new ArrayList<>();

    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        //  robot.lifter.setMode(DcMotor.ZeroPowerBehavior.BRAKE);    // needed for 20:1 motor
    }

    @Override
    public void start() {
        runtime.reset();
        boolean premoving = false;
    }

    @Override
    public void loop() {
        double y = exponential(gamepad1.left_stick_y, 1);
        double x = exponential(-gamepad1.left_stick_x, 1); //* strafingConstant; // coefficient counteracts imperfect strafing
        double rx = exponential(-gamepad1.right_stick_x, 1);


    }

    public void stop() {
        robot.stopAllMotors();
    }
}