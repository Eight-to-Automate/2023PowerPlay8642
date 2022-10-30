package org.firstinspires.ftc.teamcode.Autonomous.Meet0;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@Disabled
@Autonomous(name="Meet1BlueRight", group="Motion")
public class Meet1BlueRight extends LinearOpMode {
    RobotPowerPlay robot = new RobotPowerPlay();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        //Init required tools, robot and vuforia
        robot.initAuto(hardwareMap, this);
        //robot.initVuforia();
        //robot.initTfod();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //int route = robot.getRoute(this);
        telemetry.update();

        while (!gamepad1.a) {}
/*
        robot.GoDistance(60, 0.5,false,this );

        if (route == 1) {
            robot.Strafe(-60, 0.5, this, false);
        } else if (route == 3) {
            robot.Strafe(60, 0.5, this, false);
        }
*/

    }
}
