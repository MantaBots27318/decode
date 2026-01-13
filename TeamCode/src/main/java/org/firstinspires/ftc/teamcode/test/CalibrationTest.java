/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Calibration test : opmode to test calibration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.test;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* OpenCV includes */
import org.opencv.core.Point;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Calibration;

@Config
@TeleOp(name="CalibrationTest", group="Test")
public class CalibrationTest extends LinearOpMode {

    // Inputs (editable from FTC Dashboard)
    public static double PIXEL_X = 0;
    public static double PIXEL_Y = 0;

    private Calibration mCalibration;
    private Logger      mLogger;

    @Override
    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"calibration-test");
        mCalibration = new Calibration();

        waitForStart();

        while (opModeIsActive()) {

            try {
                // Create a pixel Point from dashboard inputs
                Point inputPixelPoint = new Point(PIXEL_X, PIXEL_Y);

                // Transform and send back through dashboard
                mCalibration.distance(inputPixelPoint);

                mLogger.metric("X", ""+PIXEL_X);
                mLogger.metric("Y", ""+PIXEL_X);

                mLogger.update();

                sleep(100); // Refresh rate
            }
            catch( Exception e) { mLogger.error(e.getMessage()); }
        }
    }

    // Function to transform pixel point and display real-world point

}
