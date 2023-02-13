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

import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="cycletest", group = "motion")
public class cycletest extends LinearOpMode{
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

    Pose2d drop1 = new Pose2d(-24,-6.5,Math.toRadians(90));
    Pose2d cyclePose = new Pose2d(-35.75, -23, Math.toRadians(90));
    Pose2d stackPos = new Pose2d(-63, -12, Math.toRadians(180));
    Vector2d preturn = new Vector2d(-35.7, -12);
    Pose2d backpose = new Pose2d(-35.7, -12);

    Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));//Splinedroptest4
    Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(90));
    Pose2d highJunctionHeading = new Pose2d(-24, -6, Math.toRadians(90));
    Pose2d highJunctionHeading2 = new Pose2d(-28, -4, Math.toRadians(95));

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        // robot.initVuforia();
        // robot.initTfod();

    //    robot.intake(true); // closes gripper

   //     robot.wait(400, this);
     //   robot.absoluteasynchLift(-220,1,this); //raise lifter slightly -> prevent cone scraping against ground
     //   robot.wait(300, this);

       // telemetry.setMsTransmissionInterval(50);

        //drive.setPoseEstimate(cyclePose);
        drive.setPoseEstimate(stackPos);

       // telemetry.setMsTransmissionInterval(50);

        TrajectorySequence maxSus = drive.trajectorySequenceBuilder(stackPos)
                .addTemporalMarker(1, () -> {
                    robot.absoluteasynchLift(robot.lifterLevelThree, 1, this);
                })
                .setReversed(true)
                .lineTo(new Vector2d(-40, -12 ))
                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-26, -6, Math.toRadians(70)), Math.toRadians(70))
                .setReversed(false)
                .lineTo(new Vector2d(-30, -12))
                .splineToSplineHeading(new Pose2d(-39, -12, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(stackPos, Math.toRadians(180))
//                .splineToSplineHeading(cyclePose, Math.toRadians(270))
//                .splineToLinearHeading(highJunctionHeading, Math.toRadians(0))
                .build();


        //_____________________________________________________________________________


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

        drive.followTrajectorySequence(maxSus);

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