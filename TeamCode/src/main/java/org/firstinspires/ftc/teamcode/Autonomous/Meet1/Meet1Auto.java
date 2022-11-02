package org.firstinspires.ftc.teamcode.Autonomous.Meet1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;

@Disabled
@Autonomous(name="Meet1Auto", group="Motion")
public class Meet1Auto extends LinearOpMode {

    RobotPowerPlay robot = new RobotPowerPlay();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.initAuto(hardwareMap, this);
        robot.initVuforia();
        robot.initTfod();
        robot.intake(true);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // route info

        int route;

        try {
            Recognition rec = robot.detectFrame(this);
            route = robot.getRoute(this, rec);
        } catch (IndexOutOfBoundsException E) {
            telemetry.addData("error", "The robot has not detected anything, assuming route 2");
            route = 2;
        }

        telemetry.addData("route:", route);
        telemetry.update();

        double topLifterTicks = 100;
        double testingPower = 0.2;
        double tileDistance = 48;

        if (route == 1) {
            robot.GoDistance(tileDistance, testingPower,  false, this);// drive 1 tile forward
            robot.Strafe(106, testingPower, this, false);// strafe 1.5 tiles right
            //robot.lifterA(topLifterTicks, 0.5, this);// raise lifter fully
            robot.GoDistance(5, testingPower, false, this);// drive forward small amount
            // release grabber
            // close grabber
            robot.GoDistance(-5, testingPower, false, this); // back up same small amount
        } else if (route == 2) {
            robot.GoDistance(tileDistance, testingPower,  false, this);// drive 1 tile forward
            robot.Strafe(106, testingPower, this, false);// strafe 1.5 tiles right
            //robot.lifterA(topLifterTicks, 0.5, this);// raise lifter fully
            robot.GoDistance(5, testingPower, false, this);// drive forward small amount
            // release grabber
            // close grabber
            robot.GoDistance(-5, testingPower, false, this); // back up same small amount
            robot.Strafe(-30, testingPower, this, false); // strafe .5 to left
            robot.GoDistance(-tileDistance, testingPower, false, this);// back up 1 tile
        } else if (route == 3) {
            robot.GoDistance(tileDistance, testingPower,  false, this);// drive 1 tile forward
            robot.Strafe(106, testingPower, this, false);// strafe 1.5 tiles right
            //robot.lifterA(topLifterTicks, 0.5, this);// raise lifter fully
            robot.GoDistance(5, testingPower, false, this);// drive forward small amount
            // release grabber
            // close grabber
            robot.GoDistance(-5, testingPower, false, this); // back up same small amount
            robot.Strafe(-30, testingPower, this, false); // strafe .5 to left
            robot.GoDistance(-tileDistance, testingPower, false, this); // back up 2 tiles
        }

    }
}
