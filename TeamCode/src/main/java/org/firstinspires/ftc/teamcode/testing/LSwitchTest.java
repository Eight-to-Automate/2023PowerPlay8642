package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@Autonomous(name="LSwitchTest", group="Motion")
public class LSwitchTest extends LinearOpMode {
    RobotPowerPlay robot = new RobotPowerPlay();
    private ElapsedTime runtime = new ElapsedTime();

    // new gripper function,, robot class has old gripper (robot.intake)
    public void intake(boolean close) {
        if (close) {
            robot.intake.setPosition(1); //true = close = 0.9 (old)
        } else {
            robot.intake.setPosition(0.42); //false = open = 0.1 (old)
        }
    }

    @Override
    public void runOpMode() {
        waitForStart();
        robot.initAuto(hardwareMap, this);
        intake(false);
        robot.lifterCalibration(this);
        telemetry.addData("lifter ticks", robot.lifter.getCurrentPosition());
        telemetry.update();
        robot.wait(5000, this);
    }
}
