/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.Autonomous.Meet2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.testing.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Meet2LeftAuto", group = "motion")
public class Meet2LeftAuto extends LinearOpMode{
    RobotPowerPlay robot = new RobotPowerPlay();

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 6;
    int MIDDLE = 4;
    int RIGHT = 2;
    int route = 0;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.initAuto(hardwareMap, this);
        robot.initVuforia();
        robot.initTfod();
        robot.intake(true); // closes

        double topLifterTicks = 100;
        double testingPower = 0.2;
        double tileDistance = 60;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT){
            route = 1;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
        }
        else if (tagOfInterest == null || tagOfInterest.id == MIDDLE){
            route = 2;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
        }
        else {
            route = 3;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
        }
        robot.intake.setPosition(0);// close grabber  added 11-1-22
        //robot.wait(1000, this );
        robot.asynchLift(-400,  1, this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(400, this);

        robot.GoDistance(-126, 0.3, false, this);
        robot.Strafe(6, 0.3, this, false);
        robot.Strafe(-103, 0.3, this,false);
        robot.asynchLift(-2180,  1, this);
        robot.wait(500, this);  //changed from 1500
        robot.GoDistance(9, 0.2, false, this);
        robot.intake.setPosition(1);
        robot.wait(1500,this);
        robot.GoDistance(-8, 0.2, false, this);
        robot.asynchLift(2050, 0.7, this);
        robot.wait(1000, this);
        robot.Strafe(-32, 0.3, this,false);
        robot.GoDistance(3 * tileDistance + 8, 0.4, false, this);
        robot.intake.setPosition(0);
        robot.wait(1000,this);
        robot.GoDistance(-3,0.2,false,this);
        robot.asynchLift(-500,0.7,this);
        robot.wait(500, this);
        robot.GoDistance(-tileDistance - 8,0.2,false,this);
        robot.Strafe(36, 0.3, this, false);
        robot.GoDistance(11, 0.3, false, this);
        robot.intake.setPosition(1);
        robot.wait(1000,this);
        robot.GoDistance(-11, 0.3, false,this);


        if (route == 3){
            robot.Strafe(40, 0.4, this, false); //was 35 11-1-22
            robot.Strafe(-9, 0.4, this, false);
            robot.GoDistance(tileDistance,0.4,false,this);
        }
        if (route == 1){
            robot.Strafe(40, 0.4, this, false); //was 35 11-1-22
            robot.Strafe(-9, 0.4, this, false);
            robot.GoDistance(-tileDistance,0.4,false,this);
        }
        /*
        robot.lifter.setTargetPosition(0);
        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifter.setPower(0.6);s
        robot.wait(2000, this);
        robot.lifter.setPower(0);*/
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}