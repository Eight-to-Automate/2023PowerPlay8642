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


package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
@Disabled
@Autonomous(name="SplineDropTest3", group = "motion")
public class SplineDropTest3 extends LinearOpMode{
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
    Pose2d drop1 = new Pose2d(-24,-6.5,Math.toRadians(90));//  was -6
    Pose2d cyclePose = new Pose2d(-35.75, -14, Math.toRadians(180));
    Pose2d stackPos = new Pose2d(-63, -12, Math.toRadians(180));
    Vector2d preturn = new Vector2d(-35.7, -12);
    Pose2d backpose = new Pose2d(-35.7, -12);


    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        // robot.initVuforia();
        // robot.initTfod();

        robot.intake(true); // closes gripper

        robot.wait(400, this);
        robot.absoluteasynchLift(-220,1,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(300, this);

       // telemetry.setMsTransmissionInterval(50);

        //drive.setPoseEstimate(cyclePose);
        drive.setPoseEstimate(startPos1);

       // telemetry.setMsTransmissionInterval(50);

        TrajectorySequence score1 = drive.trajectorySequenceBuilder(startPos1)
               .addTemporalMarker(2.2, () -> {
                    robot.absoluteasynchLift(robot.lifterLevelThree, 1, this);
                })
                .splineToLinearHeading(conePush, Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .setReversed(true)
                .splineToLinearHeading(drop1, Math.toRadians(90),  SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .setReversed(false)
                .build();

        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(score1.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.stackPosAuto, 1, this);
                })
                .splineToSplineHeading(cyclePose, Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .splineToSplineHeading(stackPos, Math.toRadians(180),   SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();

        TrajectorySequence backSmall1 = drive.trajectorySequenceBuilder(toStack1.end())
                .back(1.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))
                //   .strafeLeft(3,
                //         SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.1))
                .build();

        /*
        TrajectorySequence score2 = drive.trajectorySequenceBuilder(toStack1.end())
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.stackPosAuto, 0.8, this);
                })
                .waitSeconds(.1)
                .back(1,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))

                .waitSeconds(0.6)
                //.splineToConstantHeading(preturn, Math.toRadians(45),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();

         */

        TrajectorySequence score2 = drive.trajectorySequenceBuilder(backSmall1.end())
                .addTemporalMarker(2.5, () -> {
                    robot.absoluteasynchLift(robot.lifterLevelThree, 1, this);
                })
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-20, -7.5, Math.toRadians(135)))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(score2.end())
                //.setReversed(true)
                .addTemporalMarker(1.5, () -> {
                    robot.absoluteasynchLift(robot.stackPos, 1, this);
                })
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .lineToLinearHeading(stackPos)
                .build();

        TrajectorySequence backSmall2 = drive.trajectorySequenceBuilder(toStack2.end())
                .back(1.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))
                //   .strafeLeft(3,
                //         SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.1))
                .build();

        TrajectorySequence score3 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(2.5, () -> {
                    robot.absoluteasynchLift(robot.lifterLevelThree, 1, this);
                })
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-20, -7.5, Math.toRadians(135)))
                .build();

        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(score2.end())
                //.setReversed(true)
                .addTemporalMarker(1.5, () -> {
                    robot.absoluteasynchLift(robot.stackPos, 1, this);
                })
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .lineToLinearHeading(stackPos)
                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
           sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        //robot.intake(true);
        drive.followTrajectorySequence(score1);
        robot.intake(false); robot.intake(false);  //drop the first cone
        drive.followTrajectorySequence(toStack1);
        robot.intake(true);
        drive.followTrajectorySequence(backSmall1);
        robot.absoluteasynchLift(robot.lifterLevelOne, 1, this);
        drive.followTrajectorySequence(score2);
        robot.intake(false); robot.intake(false);
        drive.followTrajectorySequence(toStack2);
        robot.intake(true);
        drive.followTrajectorySequence(backSmall2);
        robot.absoluteasynchLift(robot.lifterLevelOne, 1, this);
        drive.followTrajectorySequence(score3);
        robot.intake(false); robot.intake(false);
        drive.followTrajectorySequence(toStack3);


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