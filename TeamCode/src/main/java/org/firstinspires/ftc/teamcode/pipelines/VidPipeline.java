package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import android.graphics.Bitmap;
import com.acmerobotics.dashboard.FtcDashboard;
import org.opencv.android.Utils;
import org.opencv.imgproc.Moments;
import java.util.ArrayList;

public class VidPipeline extends OpenCvPipeline{

    public Point centroid;
    public int largest_radius;

    public Mat processFrame(Mat input) {
/*
        Mat mat = new Mat();
        // Step 1 : mat turns into HSV values
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
 */

        Mat blurred = new Mat();
        // Step 2 : Bilateral blur
        int radius = 20;
        Imgproc.bilateralFilter(input, blurred, -1, radius, radius);

        Mat gray = new Mat();
        // Step 1 : mat turns gray
        Imgproc.cvtColor(blurred, gray, Imgproc.COLOR_RGB2GRAY);


        /*
        Scalar lowHSV = new Scalar(29, 62, 64);
        Scalar highHSV = new Scalar(150, 180, 180);
        Mat thresh = new Mat();
        //Get a black and white image
        Core.inRange(blurred, lowHSV, highHSV, thresh);

        /*
        Mat eroded = new Mat();
        // Step CV_erode0:
        Mat kernel = new Mat();
        Point anchor = new Point(-1,-1);
        Scalar borderValue = new Scalar(-1);

        Imgproc.erode(thresh, eroded, kernel, anchor, 4, Core.BORDER_CONSTANT, borderValue);

        kernel.release();
*/
        Mat circles = new Mat();
        Imgproc.HoughCircles(blurred, circles, Imgproc.HOUGH_GRADIENT, 1.5, 15, 300, 0.9, 50 , 1000);
        largest_radius = 0; Point largest_center = new Point(0,0);
        for (int i = 0; i < circles.cols(); i++ ) {
            double[] data = circles.get(0, i);
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            if (data[2] > largest_radius){
                largest_radius = (int) Math.round(data[2]);
                largest_center = center;
            }
        }

        //draw outline
        Imgproc.circle(circles, largest_center, largest_radius, new Scalar(0,0,255), 3, 8, 0 );
        centroid = largest_center;

       //**********************************************************************************************
        //CONTOUR DETECTION
/*
        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchey = new Mat();

        //finds contour
        Imgproc.findContours(eroded, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //sorts through and finds largest one

        int largestIndex = 0;
        int largest = contours.get(0).toArray().length;

        for (int i = 0; i < contours.size(); i++) {
            int currentSize = contours.get(i).toArray().length;
            if (currentSize > largest) {

                largest = currentSize;
                largestIndex = i;
            }
        }

        //Draw rectangle on largest contours

        MatOfPoint2f areaPoints = new MatOfPoint2f(contours.get(largestIndex).toArray());
        Rect rect = Imgproc.boundingRect(areaPoints);

        Imgproc.rectangle(input, rect, new Scalar(255, 0, 0));

        centerX = (rect.x + (rect.x + rect.width)) / 2;
        centerY = (rect.y + (rect.y + rect.width)) / 2;
 */
        gray.release();
        blurred.release();
        input.release();
        circles.copyTo(input);
        circles.release();
        //eroded.release();
        //mat.release();
        //thresh.release();
        //hierarchey.release();
        return input;
    }

}