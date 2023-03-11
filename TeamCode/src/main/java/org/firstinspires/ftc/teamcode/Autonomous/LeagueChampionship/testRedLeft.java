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

package org.firstinspires.ftc.teamcode.Autonomous.LeagueChampionship;

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
import org.firstinspires.ftc.teamcode.pipelines.JunctionTopPipeline3;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

import java.util.ArrayList;

@Disabled
@Autonomous(name="testredleft", group = "motion")
public class testRedLeft extends LinearOpMode{
    RobotPowerPlay robot = new RobotPowerPlay();

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera aprilTagCam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    OpenCvCamera webcam2;
    JunctionTopPipeline3 junctionTopPipeline3;

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

    // positions for roadrunner localization
    Pose2d startPos1 = new Pose2d(-35.7,-62.75, Math.toRadians(90));
    // Vector2d forward1 = new Vector2d(-36, -3.5);
    Vector2d forward2 = new Vector2d(-35.7, -7.5);// was -35.75. -9.5
    Vector2d highJunction = new Vector2d(-23, -14.5);     //was -23.5, -14.5 meet 3
    Pose2d highJunctionH = new Pose2d(-3.75, -15.5, Math.toRadians(90));
    //Vector2d getHighJunctionClose = new Vector2d(4.5, -25.1875);
    Vector2d stack = new Vector2d(-63, -10.5);  // was 62.75, 11.75
    Pose2d stackh = new Pose2d(-63, -12, Math.toRadians(180));

    Vector2d zone1 = new Vector2d(-59, -12);
    Vector2d zone2 = new Vector2d(-36,-12);
    Vector2d zone3 = new Vector2d(-12, -12);

    double[] cords = {-1, -1};

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initAutoRR(hardwareMap, this);
        //robot.initVuforia();
        //robot.initTfod();

        robot.intake(true); // closes gripper

        robot.wait(400, this);
        robot.absoluteasynchLift(-150,0.5,this); //raise lifter slightly -> prevent cone scraping against ground
        robot.wait(300, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

        aprilTagCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        junctionTopPipeline3 = new JunctionTopPipeline3(false);


        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.setPipeline(junctionTopPipeline3);
                webcam2.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // open camera1
        aprilTagCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                aprilTagCam.setPipeline(aprilTagDetectionPipeline);
                aprilTagCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addLine("april tag camera opened");
                //telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        telemetry.setMsTransmissionInterval(50);

        drive.setPoseEstimate(startPos1);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPos1)
                .lineTo(forward2,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.85, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.absoluteasynchLift(robot.lifterY, 1, this);
                })
                .lineTo(highJunction,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(6.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                .build();
         */

/*
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

 */
        Pose2d myPose = new Pose2d(stack, Math.toRadians(90));
        TrajectorySequence backSmall = drive.trajectorySequenceBuilder(myPose)
                .back(2,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                //   .strafeLeft(3,
                //         SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.1))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(backSmall.end())
                .back(24)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL * 1, DriveConstants.MAX_ANG_ACCEL) // max angle velocity was 0.7
                .turn(Math.toRadians(-90))
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), //max vel was 0.2
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.8))
                .lineTo(highJunction)
                .build();
/*
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                .build();
 */

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
                        aprilTagCam.stopStreaming();
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

        // stops april tag 1camera
        //aprilTagCam.stopStreaming();

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

        drive.followTrajectorySequence(traj1);
        robot.wait(2250, this);

        //get centroid and perform calculations for camera
        Point center = junctionTopPipeline3.getCentroid();
        cords[0] = center.x; cords[1] = center.y;
        if(cords[0] == -1 || cords[1] == -1)
            telemetry.addLine("centroid not detected");
        //webcam2.stopStreaming();
        //webcam2.stopRecordingPipeline();

        double[] movement = getMovement(cords);
        TrajectorySequence realign;

        if (movement[1] > 0)
            realign = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                    .forward(5 + movement[1])
                    .strafeRight(movement[0])
                    .build();

        else {
            realign = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .forward(5 + movement[1])
                    .strafeLeft(-movement[0])
                    .build();
        }

        drive.followTrajectorySequence(realign);

        // drive.followTrajectorySequence(traj2);
        robot.intake(false);
        webcam2.stopStreaming();
        robot.wait(1000, this);

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(realign.end())
                .back(5.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                .build();
        drive.followTrajectorySequence(traj3);

        robot.absoluteasynchLift(robot.fourStack, 0.8, this);
        //robot.wait(900, this);
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

        drive.followTrajectorySequence(traj4);
        robot.intake(true);     // first cone
        robot.wait(300, this);
        drive.followTrajectorySequence(backSmall);
        robot.absoluteasynchLift(robot.fourStack - 1000, 1, this);
        TrajectorySequence realign2 = realign;
        robot.wait(300, this);
        drive.followTrajectorySequence(traj5);
        robot.absoluteasynchLift(robot.lifterY, 1, this);
        robot.wait(1000, this);
        drive.followTrajectorySequence(realign2);
        robot.intake(false);    // second cone
        robot.wait(200, this);

        TrajectorySequence end;

        if (route == 1) {
            end = drive.trajectorySequenceBuilder(realign2.end())
                    .back(3.8)
                    .strafeLeft(12+24)
                    .addTemporalMarker(1, ()->{
                        robot.absoluteasynchLift(-380, 0.8, this);
                    })
                    .build();
        } else if (route == 2) {
            end = drive.trajectorySequenceBuilder(realign2.end())
                    .back(3.8)
                    .strafeLeft(12)
                    .addTemporalMarker(1, ()->{
                        robot.absoluteasynchLift(-380, 0.8, this);
                    })
                    .setTurnConstraint(DriveConstants.MAX_ANG_VEL * 1, DriveConstants.MAX_ANG_ACCEL)
                    .turn(Math.toRadians(90))
                    .build();
        } else {
            end = drive.trajectorySequenceBuilder(realign2.end())
                    .back(3.8)
                    .strafeRight(12)
                    .addTemporalMarker(1, ()->{
                        robot.absoluteasynchLift(-380, 0.8, this);
                    })
                    .setTurnConstraint(DriveConstants.MAX_ANG_VEL * 1, DriveConstants.MAX_ANG_ACCEL)
                    .turn(Math.toRadians(90))
                    .build();
        }

        drive.followTrajectorySequence(end);
        Pose2d poseAutoEnd = drive.getPoseEstimate();

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

    double[] getMovement(double[] cords) {
        double x = (cords[0] - 640) / (150);
        if ( x < 0.2) x*=1.2;
        //double y = -(360 - cords[1]) / (720/5.25);
        double y = (360 - cords[1]) / 110;
        if(y>2) y*=.8;

        double[] movement = {x, y};

        if(Math.abs(x) > 3 || Math.abs(y) > 3){
            movement[0] = 0;
            movement[1] = 0;
        }

        return movement;
    }
}