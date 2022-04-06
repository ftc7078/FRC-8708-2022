package frc.robot.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class MyVisionThread extends Thread {
    UsbCamera m_camera;

    public void run() {
        
        if (m_camera == null ) {
            m_camera = new UsbCamera("RoboWebcam", 1);
        }
        
        

        CameraServer.startAutomaticCapture(m_camera);

        // Set the resolution
        m_camera.setResolution(320, 240);
        m_camera.setPixelFormat(PixelFormat.kYUYV);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Circle", 640, 480);

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();
        BallDetector ballFinder = new BallDetector();
     
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }
            Mat output = ballFinder.processFrame(mat,0);
            outputStream.putFrame(output);
        }
    }

}
