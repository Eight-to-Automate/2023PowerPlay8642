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


package org.firstinspires.ftc.teamcode.Autonomous.KentState;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous(name="KentLeftEasy", group = "motion")
public class KentLeftEasy extends LinearOpMode{
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

    //******************************************************************************************
    // COORDINATES

    Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));

    Vector2d stack = new Vector2d(-62.75, -12);

    //Pose2d forward1 = new Pose2d(-35.7, -7.5, Math.toRadians(90));

    Pose2d firstBackToPole = new Pose2d(-35.7,-12, Math.toRadians(45));

    Vector2d forward1 = new Vector2d(-35.7, -7);

    Pose2d firstBackFromPole = new Pose2d(-35.7, -12, Math.toRadians(180-0.000001));
    //Pose2d highJunctionDiagonal = new Pose2d(-23.7, 0.5, Math.toRadians(45));

    // back from high junction diagonal
    Vector2d back = new Vector2d(-35.7, -12);
    Pose2d backpose = new Pose2d(-35.7, -12);

    Vector2d highJunctionDiagonal = new Vector2d(-35.7 + 7.5, -12 + 7.5);
    Vector2d smallJunctionDiagonal = new Vector2d(-35.7 - 8.485, -12 - 8.485);

    //******************************************************************************************

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot.intake(true); // closes gripper

        robot.wait(400, this);
        robot.absoluteasynchLift(-150,0.6,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(300, this);

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

        drive.setPoseEstimate(startPos1);

        telemetry.setMsTransmissionInterval(50);

        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // TRAJECTORIES


        TrajectorySequence firstForwardAndBack = drive.trajectorySequenceBuilder(startPos1)
                .lineTo(forward1,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .splineToLinearHeading(backpose, Math.toRadians(45), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9))
                /*
                .lineTo(back,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9))
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(-45))

                 */
                .build();


        TrajectorySequence drop1 = drive.trajectorySequenceBuilder(firstForwardAndBack.end())
                .waitSeconds(0.25)
                .lineTo(highJunctionDiagonal,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();


        TrajectorySequence toStack1a = drive.trajectorySequenceBuilder(drop1.end())
                //.waitSeconds(0.1)
                .lineTo(back,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();

        TrajectorySequence toStack1b = drive.trajectorySequenceBuilder(toStack1a.end())
                .waitSeconds(0.25)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(135))
                .lineTo(stack)
                .build();

        TrajectorySequence backSmall1 = drive.trajectorySequenceBuilder(toStack1b.end())
                .back(1.25,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))
                .build();

        TrajectorySequence handle2 = drive.trajectorySequenceBuilder(backSmall1.end())
                .splineToLinearHeading(backpose, Math.toRadians(45),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                /*
                .lineTo(back,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(-135))

                 */
                .build();

        TrajectorySequence drop2 = drive.trajectorySequenceBuilder(handle2.end())
                .waitSeconds(0.25)
                .lineTo(highJunctionDiagonal,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();

        TrajectorySequence toStack2a = drive.trajectorySequenceBuilder(drop2.end())
                //.waitSeconds(0.1)
                .lineTo(back,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();

        TrajectorySequence toStack2b = drive.trajectorySequenceBuilder(toStack2a.end())
                .waitSeconds(0.25)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(135))
                .lineTo(stack)
                .build();

        //backs up from stack after picking up cone 3
        TrajectorySequence backSmall2 = drive.trajectorySequenceBuilder(toStack2b.end())
                .back(1.25,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))
                .build();

        TrajectorySequence handle3 = drive.trajectorySequenceBuilder(backSmall2.end())
                .splineToLinearHeading(backpose, Math.toRadians(135),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                /*.lineTo(back,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(45))

                 */
                .build();

        /*
        TrajectorySequence drop3 = drive.trajectorySequenceBuilder(handle3.end())
                .waitSeconds(0.25)
                .lineTo(highJunctionDiagonal,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();
         */

        TrajectorySequence drop3 = drive.trajectorySequenceBuilder(handle3.end())
                .waitSeconds(0.25)
                .lineTo(smallJunctionDiagonal,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                .build();

        TrajectorySequence park;


        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
            park = drive.trajectorySequenceBuilder(drop3.end())
                    .splineTo(back, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                    .strafeLeft(22)
                    .build();
        }
        else if (tagOfInterest == null || tagOfInterest.id == MIDDLE){
            route = 2;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(drop3.end())
                    .splineTo(back, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                    .build();
        }
        else {
            route = 3;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(drop3.end())
                    .splineTo(back, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.9))
                    .strafeRight(24.5)
                    .build();
        }

        //***********************************************************************************************


        drive.followTrajectorySequence(firstForwardAndBack);

        // lift the first time
        robot.absoluteasynchLift(robot.lifterY, 1, this);
        drive.followTrajectorySequence(drop1);

        // drop cone (w/ redundancy as a fail-safe)
        robot.intake(false);
        robot.intake(false);

        drive.followTrajectorySequence(toStack1a);
        robot.absoluteasynchLift(-390, 0.9, this);
        drive.followTrajectorySequence(toStack1b);

        //intake the 2nd cone (we are scoring) from the stack
        robot.intake(true);
        robot.intake(true);
        drive.followTrajectorySequence(backSmall1);
        robot.absoluteasynchLift(-1320, 0.9, this);
        drive.followTrajectorySequence(handle2);

        robot.absoluteasynchLift(robot.lifterY, 1, this);
        drive.followTrajectorySequence(drop2);
        robot.intake(false);
        robot.intake(false);

        drive.followTrajectorySequence(toStack2a);
        robot.absoluteasynchLift(-390, 0.9, this);
        drive.followTrajectorySequence(toStack2b);

        // collect third cone
        robot.intake(true);
        robot.intake(true);
        drive.followTrajectorySequence(backSmall2);
        robot.absoluteasynchLift(robot.lifterA, 0.9, this);
        drive.followTrajectorySequence(handle3);

       // robot.absoluteasynchLift(robot.lifterA, 1, this);
        drive.followTrajectorySequence(drop3);
        robot.intake(false);
        robot.intake(false);

        drive.followTrajectorySequence(park);

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