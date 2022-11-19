/*
 * Copyright (c) 2019 OpenFTC Team
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

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.ColorVals;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class ExampleDetector extends LinearOpMode
{
    OpenCvCamera camera;

    FtcDashboard dashboard;
    PowerPlayPipeline pipeline;



    @Override
    public void runOpMode() {
        pipeline = new PowerPlayPipeline(true, ColorVals.HUE_MIN, ColorVals.HUE_MAX, ColorVals.SATURATION_MIN, ColorVals.SATURATION_MAX, ColorVals.VALUE_MIN, ColorVals.VALUE_MAX);

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.addData("Detected Position", Arrays.toString(pipeline.getPosition()));
            telemetry.update();


            handleDashboard();
        }
    }

    private void handleDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Frame Count", camera.getFrameCount());
        packet.put("FPS", String.format("%.2f", camera.getFps()));
        packet.put("Total frame time ms", camera.getTotalFrameTimeMs());
        packet.put("Pipeline time ms", camera.getPipelineTimeMs());
        packet.put("Overhead time ms", camera.getOverheadTimeMs());
        packet.put("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
        packet.put("Detected Position", Arrays.toString(pipeline.getPosition()));
//        packet.put("hue min", ColorVals.HUE_MIN);
//        packet.put("hue max", ColorVals.HUE_MAX);
//        packet.put("saturation min", ColorVals.SATURATION_MIN);
//        packet.put("saturation max", ColorVals.SATURATION_MAX);
//        packet.put("value min", ColorVals.VALUE_MIN);
//        packet.put("value max", ColorVals.VALUE_MAX);
          //FtcDashboard.getInstance().startCameraStream(camera, 0);

        dashboard.sendTelemetryPacket(packet);
    }
}