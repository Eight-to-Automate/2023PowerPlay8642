package org.firstinspires.ftc.teamcode.Autonomous.Meet0;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;
@Disabled
@Autonomous(name="Meet0Left", group="Motion")
public class Meet0Left extends LinearOpMode {
    RobotPowerPlay robot = new RobotPowerPlay();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        waitForStart();
        robot.initAuto(hardwareMap, this);
        //robot.initVuforia();
        //robot.initTfod();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        robot.GoDistance(-62, 0.5,false,this );
        robot.Strafe(-60, 0.5, this, false);
    }
}
