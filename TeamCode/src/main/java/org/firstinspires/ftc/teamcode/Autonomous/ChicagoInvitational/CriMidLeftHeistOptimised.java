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


package org.firstinspires.ftc.teamcode.Autonomous.ChicagoInvitational;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="CriMidLeftHeistOptimised", group = "motion")
public class CriMidLeftHeistOptimised extends LinearOpMode{
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
    Pose2d startPos1 = new Pose2d(35.7,-62.7 - 24, Math.toRadians(90));
    Pose2d conePush = new Pose2d(35.7,-7.5, Math.toRadians(90));
    Pose2d drop1 = new Pose2d(23,-5.5,Math.toRadians(90));//  was -6

    //    Pose2d drop2 = new Pose2d(-4.9, -20.11, Math.toRadians(-45));
    Pose2d drop2 = new Pose2d(-1, -18.5, Math.toRadians(-90));
    Pose2d stall = new Pose2d(-13,-7,Math.toRadians(-90));

    Pose2d LeftTurnTransform = new Pose2d(24,-12.5, Math.toRadians(180));
    Pose2d TransformPosition = new Pose2d(-12, -12.5, Math.toRadians(180));
//    Vector2d TRANSFORMER = new Vector2d(-38.5, 0);

    Pose2d TRANSFORMER = new Pose2d(-38.5, 0, Math.toRadians(180));
    Vector2d TRANSFORMERVECTOR = new Vector2d(-38.5, 0);
    Pose2d zone1 = new Pose2d(-12,-13, Math.toRadians(-90));
    Pose2d zone2 = new Pose2d(12,-13, Math.toRadians(-90));
    Pose2d zone3 = new Pose2d(36,-13, Math.toRadians(-90));

    Pose2d laterDropsFirstHalf = new Pose2d(30, -12, Math.toRadians(110));
    Pose2d pushToDrop1 = new Pose2d(24,-7.5-2,Math.toRadians(90));

    Pose2d cyclePose = new Pose2d(-35.75 + 24, -14, Math.toRadians(0));
    Pose2d transformerPos = new Pose2d(-62.5 + 12, 12, Math.toRadians(0));

    Pose2d CMLeftTurnTransform = new Pose2d(24,-12.5, Math.toRadians(180));
    Pose2d CMTransformPosition = new Pose2d(-12, -12.5, Math.toRadians(180));

    Vector2d laterScoresFirstLineTo = new Vector2d(40, -12 );
    //    Pose2d laterDropsSecondHalf = new Pose2d(-26, -6, Math.toRadians(70));
    Pose2d laterDropsSecondHalf = new Pose2d(26, -6.5, Math.toRadians(110));  //for testing 2/19/23
    Vector2d stack2LineTo1 = new Vector2d(30, -12);
    Pose2d stack2FirstSpline = new Pose2d(39, -12, Math.toRadians(0));

    //lifting motor new PIDF values
    public  double NEW_P = 10;//13; //15
    public double NEW_I = 3;//3; //3
    public  double NEW_D =0.2;// 1.5; //1.5
    public  double NEW_F =12.56;// 14;  //was 12.6

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        // robot.initVuforia();
        // robot.initTfod();
        RobotPowerPlay.setGripperType(3);

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        robot.lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot.closeIntake();//robot.intake2(true); // closes gripper = true = 0.9

        robot.wait(1000, this);
        robot.absoluteasynchLift(-225,0.6,this); //raise lifter slightly -> prevent cone scraping against ground
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

        //telemetry.setMsTransmissionInterval(50);

        drive.setPoseEstimate(startPos1);

        TrajectorySequence score1 = drive.trajectorySequenceBuilder(startPos1)
                .addTemporalMarker(3, () -> {
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

        TrajectorySequence toTransformer = drive.trajectorySequenceBuilder(score1.end())
                .addTemporalMarker(1.5, () -> {
                    robot.absoluteasynchLift(15, 1, this);
                })
                .back(7.5)
//                .splineToLinearHeading(CMLeftTurnTransform, Math.toRadians(180),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToLinearHeading(CMTransformPosition, Math.toRadians(180),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToConstantHeading(TRANSFORMER, Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                //.turn(Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(1,-13,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                                .splineToLinearHeading(CMTransformPosition, Math.toRadians(180),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToLinearHeading(TRANSFORMER, Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))

                //Optimized Spline to transformer traj
                .lineToSplineHeading(new Pose2d(1,-13,Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .splineToConstantHeading(TRANSFORMERVECTOR, Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();

        TrajectorySequence secureTransformer = drive.trajectorySequenceBuilder(toTransformer.end())
//                .lineTo(new Vector2d(-30,0),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(-90)),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//
//                .lineTo(new Vector2d(24, -12))
//                .splineToLinearHeading(new Pose2d(35,-24,Math.toRadians(-90)),Math.toRadians(-90),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToLinearHeading(new Pose2d(35,-58,Math.toRadians(-90)),Math.toRadians(-90),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))



//                .lineTo(new Vector2d(-18, 0),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToSplineHeading(new Pose2d(-12, -6, Math.toRadians(-90)),Math.toRadians(-90),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .lineTo(new Vector2d(-12,-58),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                //.waitSeconds(0.5)

                //Optimized spline to heist spot
                .lineTo(new Vector2d(-30,0), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .splineToSplineHeading(new Pose2d(-12,-12,Math.toRadians(-90)),Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .lineTo(new Vector2d(24, -12), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .splineToLinearHeading(new Pose2d(35,-24,Math.toRadians(-90)),Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .splineToLinearHeading(new Pose2d(35,-58-24,Math.toRadians(-90)),Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))

                .build();
//        TrajectorySequence returnToPark = drive.trajectorySequenceBuilder(secureTransformer.end())
//                //NO LONGER IN USE DO NOT USE
//                .lineTo(new Vector2d(35,-12),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//
//
//
////                .lineTo(new Vector2d(-12,-12),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .build();

//        TrajectorySequence toStall = drive.trajectorySequenceBuilder(backSmall1.end())
//                .addTemporalMarker(2, () -> {
//                    robot.absoluteasynchLift(robot.lifterY, 1, this);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(stall, Math.toRadians(-90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .build();
//
//        TrajectorySequence score2 = drive.trajectorySequenceBuilder(toStall.end())
//                .setReversed(true)//old false
//                .waitSeconds(8)
////                .addTemporalMarker(2, () -> {
////                    robot.absoluteasynchLift(robot.lifterY, 1, this);
////                })
////                .splineToSplineHeading(drop2, Math.toRadians(-45),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .lineTo(new Vector2d(-6,-12),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .splineToLinearHeading(drop2, Math.toRadians(-90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
//                .build();

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
            park = drive.trajectorySequenceBuilder(secureTransformer.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(-90)), Math.toRadians(180))
//
                    //.back(0.5)
                    //.lineToLinearHeading(zone1,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))

//                    .lineTo(new Vector2d(0, -15))
//                    .splineToSplineHeading(zone1, Math.toRadians(-90))


                    //Optimized Parking Traj zone 1
                    .lineTo(new Vector2d(35,-24),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                    .splineToLinearHeading(zone1, Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                    .build();
        }
        else if (tagOfInterest == null || tagOfInterest.id == MIDDLE){
            route = 2;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(secureTransformer.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(270))
//                    .strafeLeft(24)

                    //.back(5)
                    //.lineToLinearHeading(zone2,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    //        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))

//                    .lineTo(new Vector2d(0, -15))
//                    .splineToSplineHeading(zone2, Math.toRadians(-90))


                    //Optimized Parking Traj zone 2
                    .lineTo(new Vector2d(35,-22),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                    .splineToLinearHeading(zone2, Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                    .build();
        }
        else {
            route = 3;
            telemetry.addLine("Route = " + route);
            telemetry.update();
            //trajectory
            park = drive.trajectorySequenceBuilder(secureTransformer.end())
                    .addTemporalMarker(0.75, () -> {
                        robot.absoluteasynchLift(robot.threeStack, 1, this);
                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(270))
//                    .strafeLeft(-48)

                    //.back(5)
                    //.lineToLinearHeading(zone3,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))

//                    .lineTo(new Vector2d(0, -15))
//                    .splineToSplineHeading(zone3, Math.toRadians(-90))

                    //Optimized Parking Traj zone 3
                    .lineToLinearHeading(zone3)
                    .build();
        }

        drive.followTrajectorySequence(score1);
        robot.openIntake(); robot.openIntake();//robot.intake2(false); robot.intake2(false);  //drop the first cone

        drive.followTrajectorySequence(toTransformer);
        robot.closeIntake(); robot.closeIntake();

        drive.followTrajectorySequence(secureTransformer);
        //robot.absoluteasynchLift(robot.fourStack - 1000, 0.9, this);

        //drive.followTrajectorySequence(toStall);
        //drive.followTrajectorySequence(score2);
        robot.openIntake(); robot.openIntake();
        //drive.followTrajectorySequence(returnToPark);
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