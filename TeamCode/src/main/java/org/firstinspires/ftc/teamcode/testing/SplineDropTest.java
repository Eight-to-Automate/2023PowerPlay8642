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

@Autonomous(name="SplineDropTest", group = "motion")
public class SplineDropTest extends LinearOpMode{
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
    // Vector2d forward1 = new Vector2d(-36, -3.5);
    Vector2d forward2 = new Vector2d(-35.7, -7.5);// was -35.75. -9.5
    Vector2d highJunction = new Vector2d(-23.5, -14.5);     //was -23.5, -14.5 meet 3
    Pose2d highJunctionH = new Pose2d(-3.75, -15.5, Math.toRadians(90));
    //Vector2d getHighJunctionClose = new Vector2d(4.5, -25.1875);
    Vector2d stack = new Vector2d(-62.75, -10.5);  // was 62.75, 11.75
    Pose2d stackh = new Pose2d(-63, -12, Math.toRadians(180));

    Pose2d cyclePose = new Pose2d(-35.75, -24, Math.toRadians(90));
    //Pose2d cyclePose = new Pose2d(-35.75, -48, Math.toRadians(90));
    Pose2d highJunctionHeading = new Pose2d(-24, -6, Math.toRadians(90));


    Vector2d zone1 = new Vector2d(-59, -12);
    Vector2d zone2 = new Vector2d(-36,-12);
    Vector2d zone3 = new Vector2d(-12, -12);

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
       // robot.initVuforia();
       // robot.initTfod();

        robot.intake(true); // closes gripper

        robot.wait(400, this);
        robot.absoluteasynchLift(-150,0.6,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(300, this);

        telemetry.setMsTransmissionInterval(50);

        //drive.setPoseEstimate(cyclePose);
        drive.setPoseEstimate(startPos1);
/*
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPos1)
                .lineTo(forward2,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), // max velocity was 0.6 with feed forward
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9)) // added Max accel * 0.8 for championship Drive speed was 0.85
                .lineTo(highJunction,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
               .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(7,         // was 5.7 - 0.45-.25 for meet 3
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(6.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .strafeLeft(12)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL * 1, DriveConstants.MAX_ANG_ACCEL)
                .turn(Math.toRadians(90))
                //.lineToLinearHeading(stackh,
                //        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(stack,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence backSmall = drive.trajectorySequenceBuilder(traj4.end())
                .back(1,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4))
                //   .strafeLeft(3,
                //         SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.1))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(backSmall.end())
                .back(25)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL * 1, DriveConstants.MAX_ANG_ACCEL) // max angle velocity was 0.7
                .turn(Math.toRadians(-90))
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), //max vel was 0.2
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .lineTo(highJunction)
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .forward(6,   // was 5.5 meet 3.
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .build();
*/
        TrajectorySequence spline1 = drive.trajectorySequenceBuilder(startPos1)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.lifterLevelThree, 1, this);
                })
                .splineToLinearHeading(cyclePose, Math.toRadians(90))
                .splineToLinearHeading(highJunctionHeading, Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL*0.8, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.8))
                .build();



        // TrajectorySequence sanityTest = drive.trajectorySequenceBuilder(new Pose2d(-36,-63.5, Math.toRadians(90)))
        //         .lineTo(new Vector2d(-36,0))
        //         .build();


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





        drive.followTrajectorySequence(spline1);





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