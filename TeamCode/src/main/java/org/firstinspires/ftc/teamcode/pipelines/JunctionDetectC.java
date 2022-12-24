package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
// import org.opencv.core.MatOfPoint;
// import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
// import org.opencv.core.Rect;
import org.opencv.core.Scalar;
// import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// import android.graphics.Bitmap;
// import com.acmerobotics.dashboard.FtcDashboard;
// import org.opencv.android.Utils;
// import org.opencv.imgproc.Moments;
// import java.util.ArrayList;

// Hough Circles Pipeline
public class JunctionDetectC extends OpenCvPipeline{

    public Point centroid;
    public int largest_radius;
    Scalar lowHSV = new Scalar(70,0,0);
    Scalar highHSV = new Scalar(90,255,255);

    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Mat thresh = new Mat();
        Core.inRange(hsv, lowHSV, highHSV, hsv);

        Mat mask = new Mat();
        input.copyTo(mask, hsv);

        // Mat gray = new Mat();
        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_RGBA2GRAY);

        // Mat blur = new Mat();
        Imgproc.medianBlur(mask, mask, 1);

        Mat circles = new Mat();
        Imgproc.HoughCircles(mask, circles, Imgproc.HOUGH_GRADIENT, 1.5, 300, 300, 50, 55, 75);
        for (int i = 0; i < circles.cols(); i++ ) {
            double[] data = circles.get(0, i);
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(input, center, 1, new Scalar(0, 0, 255), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(data[2]);
            Imgproc.circle(input, center, radius, new Scalar(0,0,255), 3, 8, 0 );

            Imgproc.putText(input, Integer.toString(radius), center, 1, 2, new Scalar(255,255,255));
        }

        hsv.release();
        mask.release();
        circles.release();
        return input;
    }

}
