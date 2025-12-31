/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Calibration test : opmode to test limelight
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.test;

// Java includes
import java.util.List;

// FTCController includes
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Roadrunner includes
import com.acmerobotics.dashboard.config.Config;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;


@Config
@TeleOp
public class VisionTest extends LinearOpMode {


    public enum Function {

        PATTERN("PATTERN"),
        DETECTION("DETECTION"),
        LOCALIZATION("LOCALIZATION");

        private final String mText;

        Function(String text) { mText = text; };

        public String text() { return mText; }
    }

    public static String  FUNCTION = "PATTERN";

    private Vision  mVision;

    private Pose3D  mPreviousOutput = null;

    private Logger  mLogger;


    @Override
    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"vision-test");


        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap,"vision",mLogger);
        mVision.initialize();
        mLogger.update();

        waitForStart();

        while (opModeIsActive()) {

            try {

                mLogger.info(FUNCTION);

                if(FUNCTION.equals(Function.PATTERN.text())) {
                    // Transform and send back through dashboard
                    Pattern pattern = mVision.readPattern();
                    mLogger.info("Pattern : " + pattern.text());
                }
                else if (FUNCTION.equals(Function.DETECTION.text())) {

                    List<Ball> detectedBalls = mVision.getArtifactPosition();

                    mLogger.info("Artifacts number: " + detectedBalls.size());

                    for (Ball ball : detectedBalls) {
                        mLogger.info("Color: " + ball.color());
                        mLogger.info("Position: " + ball.position());
                    }
                }
                else if(FUNCTION.equals(Function.LOCALIZATION.text())) {

                    FtcDashboard.getInstance().getTelemetry().addLine("Loc");

                    Pose3D output = mVision.getPosition();
                    Pose3D prevOutput = null;
                    if (output != null) {
                        mLogger.info("Pose3D" + output);
                        mPreviousOutput = output;
                    } else {
                        if(mPreviousOutput != null) {
                            mLogger.info("Pose3D" + prevOutput);
                        }
                    }
                }

                sleep(100); // Refresh rate

            }
            catch( Exception e) { mLogger.error(e.getMessage()); }
            mLogger.update();
        }

        mVision.close();
    }

    // Function to transform pixel point and display real-world point

}
