package frc.robot.vision;

  
import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.Robot;
import java.awt.AWTException;
import java.awt.Color;


public class SkystoneDetector {
    enum SkystoneLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width; // width of the image
    SkystoneLocation location;
    private Point center = new Point(0, 0);
    private Robot robot;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public void SkystoneDetector(int width) {
        this.width = width;
    }

    public Mat processFrame(Mat input, int color) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no ball
        if (mat.empty()) {
            location = null;
            return input;
        }

        // We create a HSV range for both colors in an if statement to decide on which team we are to detect balls
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(150, 150, 0); // lower bound HSV for blue
        Scalar highHSV = new Scalar(180, 255, 255); // higher bound HSV for blue
        if (color != 0) { //red color array
            lowHSV = new Scalar(0, 150, 0); // lower bound HSV for red
            highHSV = new Scalar(10, 255, 255); // higher bound HSV for red
        }
        Mat thresh = new Mat();
        

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)gray.rows()/.0000001, // change this value to detect circles with different distances to each other
                100.0, 30.0, 20, 600); // change the last two parameters
                // (min_radius & max_radius) to detect larger circles
        Color blue = new Color(0, 0 , 255);
        Color red = new Color(204, 0, 0);
        int cx;
        int cy;


        //Draw Circles on image
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            this.center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(input, center, 1, new Scalar(0,100,100), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(input, center, radius, new Scalar(255,0,255), 3, 8, 0 );
        }
        cx = (int)this.center.x;
        cy = (int)this.center.y;

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        

        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = SkystoneLocation.LEFT;
        else if (!right) location = SkystoneLocation.RIGHT;
        // if both are true, then there's no Skystone in front.
        // since our team's camera can only detect two at a time
        // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE;

        return input; // return the mat with rectangles drawn
    }

    public SkystoneLocation getLocation() {
        return this.location;
    }

}
