package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;

public class PowerPlayPipeline extends OpenCvPipeline
{
    private boolean enableDashboard;
    private FtcDashboard dashboard;

    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();
    Mat cameraMatrix;


    private int largestX, largestY;
    private double largestArea;

    //constants
    public static int pipelineStage = 0;
    public static double BLUR_RADIUS = 10;

    public static double HUE_MIN;
    public static double HUE_MAX;
    public static double SATURATION_MIN;
    public static double SATURATION_MAX;
    public static double VALUE_MIN;
    public static double VALUE_MAX;

    public static double MIN_CONTOUR_AREA = 200;
    public static String BLUR = "Box Blur";

    // store the top and bottom of the cone, min is top and max is bottom
    double minY;
    double maxY;
    int height;

    Point centroid = new Point(-1, -1);
    Point lowerConeBound = new Point(-1, -1);
    Point upperConeBound = new Point(-1, -1);

    enum Stage {
        blurOutput, hsvThresholdOutput, finalContourOutputMat
    }

    private Stage stageToRenderToViewport = Stage.finalContourOutputMat;

    private Stage[] stages = Stage.values();


    public PowerPlayPipeline(boolean enableDashboard, double hmin, double hmax, double smin, double smax, double vmin, double vmax) {
        this.enableDashboard = enableDashboard;

        if(enableDashboard)
            dashboard = FtcDashboard.getInstance();

        largestX = -1;
        largestY = -1;
        largestArea = -1;
        // store the top and bottom of the cone, min is top and max is bottom
        minY = -1;
        maxY = -1;
        height = -1;

        HUE_MIN = hmin;
        HUE_MAX = hmax;
        SATURATION_MIN = smin;
        SATURATION_MAX = smax;
        VALUE_MIN = vmin;
        VALUE_MAX = vmax;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Step Blur0 (stage 1):
        blurInput = input;
        BlurType blurType = BlurType.get(BLUR);
        double blurRadius = BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0  (stage 2):
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {HUE_MIN, HUE_MAX};
        double[] hsvThresholdSaturation = {SATURATION_MIN, SATURATION_MAX};
        double[] hsvThresholdValue = {VALUE_MIN, VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0 (stage 3):
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
        findContoursOutputMat = input.clone();
        for(int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // Finding largest contour (stage 4):
        finalContourOutputMat = input.clone();
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        int largestContourIndex = -1;
        for(int i = 0; i < findContoursOutput.size(); i++) {
            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > MIN_CONTOUR_AREA && contourArea > largestArea) {
                Moments p = Imgproc.moments(contour, false);

                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                largestContourIndex = i;
                largestX = x;
                largestY = y;
                largestArea = contourArea;
            }
        }

        if(largestContourIndex != -1)
            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);

        // isolate contour (stage 5)
        Mat contourIsolation = finalContourOutputMat;
        double[] hsvThresholdHue2 = {0, 0};
        double[] hsvThresholdSaturation2 = {0, 0};
        double[] hsvThresholdValue2 = {255, 255};
        hsvThreshold(finalContourOutputMat, hsvThresholdHue2, hsvThresholdSaturation2, hsvThresholdValue2, contourIsolation);

        // search for white pixels

        ArrayList<Point> contourPixels = new ArrayList<Point>();

        for (int y = 1; y < contourIsolation.rows(); y++) {

            for (int x = 1; x < contourIsolation.cols(); x++) {
                double[] pxCol = contourIsolation.get(x, y);

                if (pxCol != null) {
                    if (pxCol[0] == 255 && pxCol[1] == 255 && pxCol[2] == 255) {
                        contourPixels.add(new Point(x, y));
                    }
                }
            }
        }

        maxY = contourPixels.get(0).y;
        minY = maxY;


        for (Point r : contourPixels) {
            if (r.y < minY) {
                minY = r.y;
            }
            if (r.y > maxY) {
                maxY = r.y;
            }
        }

        height = (int)(minY - maxY);

       // establish significant points and lines for the camera stream
        Scalar white = new Scalar(0, 0, 0);
        Point centroid = new Point(largestX, largestY);
        Point lowerConeBound = new Point(largestX, maxY);
        Point upperConeBound = new Point(largestX, minY);

        Imgproc.line(finalContourOutputMat, lowerConeBound, upperConeBound, white);
        Imgproc.circle(finalContourOutputMat, centroid, 20, white, 20);

        handleDashboard();

        return finalContourOutputMat;
    }

    public Point getCentroid() {
        return centroid;
    }

    public int getHeight() {
        return height;
    }

    public Point getTop() {
        return upperConeBound;
    }

    public Point getBottom() {
        return lowerConeBound;
    }



    private void handleDashboard() {
        if(enableDashboard) {
            Mat toSend = null;
            switch(pipelineStage) {
                case 0:
                    toSend = blurInput;
                    break;
                case 1:
                    toSend = blurOutput;
                    break;
                case 2:
                    toSend = hsvThresholdOutput;
                    break;
                case 3:
                    toSend = findContoursOutputMat;
                    break;
                case 4:
                    toSend = finalContourOutputMat;
                    break;
            }
            sendMatToDashboard(toSend);
        }
    }

    private void sendMatToDashboard(Mat input) {
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);
        dashboard.sendImage(bitmap);
    }


    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    private void findContours(Mat input, boolean externalOnly,
                              ArrayList<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_NONE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }


}