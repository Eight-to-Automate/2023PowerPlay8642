package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@Disabled
@Autonomous(name="SleeveTest", group="Motion")
public class SleeveTest extends LinearOpMode {
    RobotPowerPlay robot = new RobotPowerPlay();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.initAutoTester(hardwareMap, this);
        robot.initVuforia();
        robot.initTfod();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //robot.initAuto(hardwareMap, this);
        //robot.initVuforia();
        //robot.initTfod();

        //int route = robot.getRoute(this);

        Recognition rec = robot.detectFrame(this);
        int route = robot.getRoute(this, rec);
        telemetry.addData("route:", route);
        telemetry.update();


        while (opModeIsActive()) {}
    }
}
