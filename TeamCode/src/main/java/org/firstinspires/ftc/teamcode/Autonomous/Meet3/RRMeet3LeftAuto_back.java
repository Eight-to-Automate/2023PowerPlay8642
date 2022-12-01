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


package org.firstinspires.ftc.teamcode.Autonomous.Meet3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="RRMeet3LeftAuto_back", group = "motion")
public class RRMeet3LeftAuto_back extends LinearOpMode{
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

    // positions for localization
    Pose2d startPos1 = new Pose2d(35.625,-62.75, Math.toRadians(180));
    Vector2d wallAlign1 = new Vector2d(13.5, -65.1875);
    Vector2d highJunction = new Vector2d(13.5, -25.1875);
    Vector2d getHighJunctionClose = new Vector2d(4.5, -25.1875);
    Vector2d zone1 = new Vector2d(13.5, -35.1875);
    Vector2d zone2 = new Vector2d(24,-35.1875 );
    Vector2d zone3 = new Vector2d(24, -35.1875);

    TrajectorySequence end;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
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

        //****************************************************************************************
        //TRAJECTORIES

        drive.setPoseEstimate(startPos1);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPos1)
                .forward(17)

                .addDisplacementMarker(17, () -> {
                    robot.asynchLift(robot.lifterLevelOne, 1, this);
                })
                .strafeRight(47)
                .forward(5)

                .build();
               // .splineToConstantHeading(highJunction, Math.toRadians(180))
                //robot.intake(false);

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPos1)
                .back(5)
                .addTemporalMarker(1, () ->{
                    robot.asynchLift(530, 0.8, this);
                })
                .strafeRight(13)
                .forward(3*24-4)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPos1)
                .strafeRight(13)
                .forward(3*24-4)

                .build();

        /*
                .forward(9,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL , DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addSpatialMarker(getHighJunctionClose, () -> {
                    robot.intake(false);
                })
                .back(-9,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL , DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .strafeLeft(14.17)*/



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

        /*
        if (route == 1) {
            TrajectorySequence end = drive.trajectorySequenceBuilder(traj1.end())
                    .splineTo(zone1, Math.toRadians(180))
                    .build();
        } else if (route == 2) {
            TrajectorySequence end = drive.trajectorySequenceBuilder(traj1.end())
                    .splineTo(zone2, Math.toRadians(180))
                    .build();
        } else {
            TrajectorySequence end = drive.trajectorySequenceBuilder(traj1.end())
                    .splineTo(zone3, Math.toRadians(180))
                    .build();
        }
         */

        //robot.intake(true);
        double timeout = getRuntime();
        double cp=robot.lifter.getCurrentPosition();
        robot.wait(400, this);
        robot.absoluteasynchLift(-400,1,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(300, this);

        /*if((cp-150)>robot.lifter.getCurrentPosition()){
            robot.lifter.setTargetPosition(robot.lifter.getCurrentPosition());
        }*/

        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(end);
        robot.intake(false);
        robot.wait(500, this);

        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(end);




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