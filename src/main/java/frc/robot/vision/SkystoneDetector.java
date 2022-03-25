package frc.robot.vision;

  
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector {
    /*enum SkystoneLocation {
    *    LEFT,
    *   RIGHT,
    *    NONE
    *}
    */
    private int width; // width of the image
    SkystoneLocation location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public void SkystoneDetector(int width) {this.width = width;}

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
            location = SkystoneLocation.NONE;
            return input;
        }

        // We create a HSV range for both colors in an if statement to decide on which team we are to detect balls
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        if (color == 0) { //Blue color array
            Scalar lowHSV = new Scalar(150, 150, 0); // lower bound HSV for blue
            Scalar highHSV = new Scalar(180, 255, 255); // higher bound HSV for blue
        }
        else{ //red color array
            Scalar lowHSV = new Scalar(0, 150, 0); // lower bound HSV for red
            Scalar highHSV = new Scalar(10, 255, 255); // higher bound HSV for red
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
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }
        Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            if (color == 0) { //blue circle
                Scalar color = new Scalar(0, 0, 255);
            }
            else{ //red circle
                Scalar color = new Scalar(255, 0, 0);
            }
            Imgproc.drawContours(drawing, contoursPolyList, i, color);
            Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        //if (!left) location = SkystoneLocation.LEFT;
        //else if (!right) location = SkystoneLocation.RIGHT;
        // if both are true, then there's no Skystone in front.
        // since our team's camera can only detect two at a time
        // we will need to scan the next 2 stones
        //else location = SkystoneLocation.NONE;

        return input; // return the mat with circles drawn
    }

    public SkystoneLocation getLocation() {
        return this.location;
    }

}
