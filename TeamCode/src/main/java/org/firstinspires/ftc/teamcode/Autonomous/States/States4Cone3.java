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

//states left but I fixed the trajectories to be more smooth
// works except for third cone

package org.firstinspires.ftc.teamcode.Autonomous.States;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
//changed all the speeds
@Autonomous(name="States4Cone3", group = "motion")
public class States4Cone3 extends LinearOpMode{
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
    Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));
    Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(90));

    Pose2d pushToDrop1 = new Pose2d(-35.7,-9.5,Math.toRadians(90)); // 03/04/23 was -24
    Pose2d drop1 = new Pose2d(-25,-6.5,Math.toRadians(90));//  was -6, 03/04/23 was -24

    Pose2d stackPos = new Pose2d(-62.5, -12, Math.toRadians(180));

    Pose2d scorePos = new Pose2d(-26,-6, Math.toRadians(70));
    Pose2d scoreBack = new Pose2d(-26 - 4 * Math.cos(Math.toRadians(70)), -6 - 4 * Math.sin(Math.toRadians(70)), Math.toRadians(70));

    Pose2d prePark = new Pose2d(-28.55, -13, Math.toRadians(70));
    Pose2d zone1 = new Pose2d(-58, -13, Math.toRadians(90));
    Pose2d zone2 = new Pose2d(-35.7, -13, Math.toRadians(90));
    Pose2d zone3 = new Pose2d(-12, -13, Math.toRadians(90));


    public  double NEW_P = 10;//13; //15
    public double NEW_I = 3;//3; //3
    public  double NEW_D =0.2;// 1.5; //1.5
    public  double NEW_F =12.56;// 14;  //was 12.6

    // new gripper function,, robot class has old gripper (robot.intake)
    public void intake(boolean close) {
        if (close) {
            robot.intake.setPosition(1); //true = close = 0.9 (old)
        } else {
            robot.intake.setPosition(0.315); //false = open = 0.1 (old)
        }
    }


    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        // robot.initVuforia();
        // robot.initTfod();
        RobotPowerPlay.setGripperType(3);

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        // PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        robot.lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);

        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot.closeIntake(); // closes gripper = true = 0.9

        robot.wait(400, this);
        robot.absoluteasynchLift(-225,0.6,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(700, this);


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

        TrajectorySequence score1 = drive.trajectorySequenceBuilder(startPos1)

                .addTemporalMarker(2.2, () -> {
                    robot.absoluteasynchLift(robot.lifterY, 1, this);
                })
                .splineToLinearHeading(conePush, Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))//first forward movement
                .setReversed(true)

                .splineToLinearHeading(pushToDrop1, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))  //Originally 90 spline from first forwards movement to high goal score pos
                .splineToLinearHeading(drop1, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))  //-7.8
                .setReversed(false)
                .build();


        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(score1.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.fiveStack, 1, this);
                })
                .back(1.5,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(180)), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(stackPos, Math.toRadians(180),   SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence score2 = drive.trajectorySequenceBuilder(toStack1.end())
                .addTemporalMarker(0.1, () -> {
                    robot.absoluteasynchLift(robot.lifterY, 1, this);
                })
                .setReversed(true)
                .lineTo(new Vector2d(-40, -12 ), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5))
                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(scorePos, Math.toRadians(70), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setReversed(false)
                .build();

//        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(score2.end())
//                .setReversed(true)
//                .addTemporalMarker(1, () -> {
//                    robot.absoluteasynchLift(robot.stackPos, 1, this);
//                })
//                //was 1.5 then 2
//                .back(3,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(180)), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(-62.5, -12, Math.toRadians(180)), Math.toRadians(180),   SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(score2.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.fourStack, 1, this);
                })
                //was 1.5 then 2
                .setReversed(true)
                .lineToLinearHeading(scoreBack)
                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(180)), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(stackPos, Math.toRadians(180),   SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence score3 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0.1, () -> {
                    robot.absoluteasynchLift(robot.lifterY, 1, this);
                })
                .setReversed(true)
                .lineTo(new Vector2d(-40, -12 ), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5))
                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(scorePos, Math.toRadians(70), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(score3.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.threeStack, 1, this);
                })
                .setReversed(true)
                .lineToLinearHeading(scoreBack)
                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(180)), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(stackPos, Math.toRadians(180),   SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence score4 = drive.trajectorySequenceBuilder(toStack3.end())
                .addTemporalMarker(0.1, () -> {
                    robot.absoluteasynchLift(robot.lifterY, 1, this);
                })
                .setReversed(true)
                .lineTo(new Vector2d(-40, -12 ), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(scorePos, Math.toRadians(70), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence park;

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
            park = drive.trajectorySequenceBuilder(score4.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(false)
//                    .back(3)
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(90)), Math.toRadians(210)) //zone 1

                    .lineToLinearHeading(prePark)
                    .lineToLinearHeading(zone1)
                    .build();


        }
        else if (tagOfInterest == null || tagOfInterest.id == MIDDLE){
            route = 2;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(score4.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(false)
//                    .back(6)
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)), Math.toRadians(180)) //zone 2

                    .lineToLinearHeading(prePark)
                    .lineToLinearHeading(zone2)
                    .build();
        }
        else {
            route = 3;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(score4.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(false)
//                    .back(2.7)
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-12, -14.5, Math.toRadians(90)), Math.toRadians(90)) //zone 3

                    .lineToLinearHeading(prePark)
                    .lineToLinearHeading(zone3)
                    .build();
        }



        drive.followTrajectorySequence(score1);
        robot.openIntake(); robot.openIntake();//intake(false); intake(false);  //drop the first cone

        drive.followTrajectorySequence(toStack1);
        robot.closeIntake(); robot.closeIntake();//intake(true); intake(true);
        robot.wait(200, this);


        //drive.followTrajectorySequence(backSmall1);
        robot.absoluteasynchLift(robot.fourStack - 1000, 0.9, this);
        drive.followTrajectorySequence(score2);
        robot.openIntake(); robot.openIntake();//intake(false); intake(false);  //drop the second cone

        drive.followTrajectorySequence(toStack2);
        robot.closeIntake(); robot.closeIntake();//intake(true); intake(true);
        robot.wait(200, this);


        //drive.followTrajectorySequence(backSmall2);
        robot.absoluteasynchLift(robot.fourStack - 1000, 0.9, this);
        drive.followTrajectorySequence(score3);
        robot.openIntake(); robot.openIntake();//intake(false); intake(false);  //drop the second cone

        drive.followTrajectorySequence(toStack3);
        robot.closeIntake(); robot.closeIntake();//intake(true); intake(true);
        robot.wait(200, this);

        //drive.followTrajectorySequence(backSmall3);
        robot.absoluteasynchLift(robot.fourStack - 1000, 0.9, this);
        drive.followTrajectorySequence(score4);
        robot.openIntake(); robot.openIntake();//intake(false); intake(false);  //drop the second cone

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