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
    @Override
    public void runOpMode() {
        waitForStart();
        robot.initAuto(hardwareMap, this);
        robot.lifterCalibration(this);
    }
}
