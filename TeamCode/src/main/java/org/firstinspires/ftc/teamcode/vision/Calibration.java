package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

public class Calibration {


    static Point[] dstPoints = new Point[]{
            new Point(0,3 ),
            new Point(-3,6 ),
            new Point(3,6 ),
            new Point(6,6 ),
            new Point(9,9 ),
            new Point(9, 12),
            new Point(-6,15 ),
            new Point(-12, 27),
            new Point(12,24 ),
            new Point(0,9 ),
            new Point(0,24 ),
            new Point(-3,21 ),
            new Point(-12,21),

    };
    static Point[] srcPoints = new Point[] {
            new Point(317, 474),
            new Point(207, 369),
            new Point(426, 361),
            new Point(536,359 ),
            new Point(607, 275),
            new Point(578, 209),
            new Point(150, 161),
            new Point(68,5 ),
            new Point(565,29 ),
            new Point(314,282 ),
            new Point(310,33 ),
            new Point(242,71 ),
            new Point (31,73),
    };

    private Telemetry mTelemetry;

    private Mat homography ;

    public Calibration(){
        this.warp();}

    public  void warp( ) {



        MatOfPoint2f srcMatOfPoint = new MatOfPoint2f(srcPoints);
        MatOfPoint2f dstMatOfPoint = new MatOfPoint2f(dstPoints);

        // Find homography
         homography = Calib3d.findHomography(srcMatOfPoint, dstMatOfPoint);

        // Warp the source image



    }

    public Point distance(Point pixelPoint) {
        MatOfPoint2f src = new MatOfPoint2f(pixelPoint);
        MatOfPoint2f dst = new MatOfPoint2f();

        Core.perspectiveTransform(src, dst, homography);

        Point[] dstPoints = dst.toArray();
        if (dstPoints.length > 0) {
            FtcDashboard.getInstance().getTelemetry().addData("Real World X", dstPoints[0].x);
            FtcDashboard.getInstance().getTelemetry().addData("Real World Y", dstPoints[0].y);
        }
        return dstPoints[0];
    }


    }


