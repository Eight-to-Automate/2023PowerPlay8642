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

package org.firstinspires.ftc.teamcode.testing;

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

//changed all the speeds
@Disabled
@Autonomous(name="Straight Movement", group = "motion")
public class StraightMovement extends LinearOpMode{
    RobotPowerPlay robot = new RobotPowerPlay();

    private ElapsedTime runtime = new ElapsedTime();

    static final double FEET_PER_METER = 3.28084;



    // positions for localization
    Pose2d startPos1 = new Pose2d(35.7,-62.7, Math.toRadians(90));
    Pose2d conePush = new Pose2d(35.7,-7.5, Math.toRadians(90));

    Pose2d pushToDrop1 = new Pose2d(35.7,-9.5,Math.toRadians(90)); // 03/04/23 was -24
    Pose2d drop1 = new Pose2d(25,-6.5,Math.toRadians(90));//  was -6, 03/04/23 was -24

    Pose2d stackPos = new Pose2d(62.5, -12, Math.toRadians(0));

    Pose2d scorePos = new Pose2d(26,-6, Math.toRadians(180-70));
    Pose2d scoreBack = new Pose2d(26 + 4 * Math.cos(Math.toRadians(180 - 70)), -6 - 4 * Math.sin(Math.toRadians(180 - 70)), Math.toRadians(180 - 70));

    Pose2d prePark = new Pose2d(28.55, -13, Math.toRadians(180 - 70));
    Pose2d zone1 = new Pose2d(58, -13, Math.toRadians(90));
    Pose2d zone2 = new Pose2d(35.7, -13, Math.toRadians(90));
    Pose2d zone3 = new Pose2d(12, -13, Math.toRadians(90));


    public  double NEW_P = 13;
    public double NEW_I = 1.5;
    public  double NEW_D = 1.5;
    public  double NEW_F = 14;  //was 12.6



    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        // robot.initVuforia();
        // robot.initTfod();

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        // PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        robot.lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);

        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent

        drive.setPoseEstimate(startPos1);

        TrajectorySequence score1 = drive.trajectorySequenceBuilder(startPos1)
                .lineToLinearHeading(conePush,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                .build();

       waitForStart();

       drive.followTrajectorySequence(score1);


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