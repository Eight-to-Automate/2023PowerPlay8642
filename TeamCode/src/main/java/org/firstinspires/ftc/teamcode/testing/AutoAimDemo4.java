/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.JunctionDetectC;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled

@Autonomous(name="AutoAimDemo4", group="Linear Opmode")

public class AutoAimDemo4 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotPowerPlay robot = new RobotPowerPlay();
    JunctionDetectC pipeline;
    OpenCvCamera camera;

    @Override
    public void runOpMode() {

        robot.initAutoRR(hardwareMap, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // *************************************************************************************************
        //Auto INIT
        robot.initAutoRR(hardwareMap, this);

        robot.intake(true);

        pipeline = new JunctionDetectC();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //FtcDashboard.getInstance().startCameraStream(camera, 15);

        waitForStart(); //******************************************************************************
        telemetry.addData("Pipeline Time", camera.getPipelineTimeMs());
        telemetry.addData("Overhead Time", camera.getOverheadTimeMs());
        telemetry.addData("Largest circle's radius," , pipeline.largest_radius);
        //double startT = getRuntime();

        double[] cords = {-1, -1};

        Point centroid;

        centroid = pipeline.getCentroid();
        cords[0] = centroid.x;
        cords[1] = centroid.y;



        if (cords[0] == -1 && cords[1] == -1) {
            telemetry.addLine("target not found");
        }

        double[] movement = getMovement(cords);

        telemetry.addData("x pixel position", cords[0]);
        telemetry.addData("y pixel position", cords[1]);
        telemetry.addData("Move in x", movement[0]);
        telemetry.addData("move in y", movement[1]);
        telemetry.addData("Pipeline Time", camera.getPipelineTimeMs());
        telemetry.addData("Overhead Time", camera.getOverheadTimeMs());
        telemetry.addData("Largest circle's radius," , pipeline.largest_radius);
        double start = getRuntime();
        Point curr;
        while (getRuntime() < start + 5000){
            curr = pipeline.getCentroid();
            if (curr!=null){
                telemetry.addData("CenterX = ", curr.x);
                telemetry.addData("CenterY = ", curr.y);
            }
            else telemetry.addLine("Haven't updated centroid yet / it is empty");
            telemetry.update();
        }



        TrajectorySequence realign;

        if (movement[1] > 0)
            realign = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                    .strafeRight(movement[0])
                    .forward(5 + movement[1])

                    .build();

        else {
            realign = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .strafeLeft(-movement[0])

                    .forward(5 + movement[1])
                    .build();
        }

        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));
        drive.followTrajectorySequence(realign);



        while (opModeIsActive()) {}

    }

    public double[] getMovement(double[] cords) {
        double x = (cords[0] - 640) / (150);
        if ( x < 0.2) x*=1.2;
        //double y = -(360 - cords[1]) / (720/5.25);
        double y = (360 - cords[1]) / 110;
        if(y>2) y*=.8;

        double[] movement = {x, y};

        return movement;
    }

}
