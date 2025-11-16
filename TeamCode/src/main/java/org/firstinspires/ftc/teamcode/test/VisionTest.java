package org.firstinspires.ftc.teamcode.test;

// Java includes
import java.util.List;

// FTCController includes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Roadrunner includes
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Local includes */
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Ball;
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


    @Override
    public void runOpMode() throws InterruptedException {

        // Setup the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap,"vision",dashboard.getTelemetry());
        mVision.initialize();
        dashboard.getTelemetry().update();

        waitForStart();

        while (opModeIsActive()) {

            try {

                FtcDashboard.getInstance().getTelemetry().addLine(FUNCTION);

                FtcDashboard.getInstance().getTelemetry().addLine(Function.PATTERN.text());

                FtcDashboard.getInstance().getTelemetry().addLine(Function.DETECTION.text());

                FtcDashboard.getInstance().getTelemetry().addLine(Function.LOCALIZATION.text());

                if(FUNCTION.equals(Function.PATTERN.text())) {
                    // Transform and send back through dashboard
                    FtcDashboard.getInstance().getTelemetry().addLine("Pattern");
                    Vision.Pattern pattern = mVision.readPattern();

                    telemetry.addData("Pattern ", pattern.text());
                    dashboard.getTelemetry().addData("Pattern ", pattern.text());
                }
                else if (FUNCTION.equals(Function.DETECTION.text())) {

                    FtcDashboard.getInstance().getTelemetry().addLine("Detection");
                    List<Ball> detectedBalls = mVision.getArtifactPosition();

                    telemetry.addData("Nombre de artifacts: ", detectedBalls.size());
                    dashboard.getTelemetry().addData("Nombre de artifacts ", detectedBalls.size());

                    for (Ball ball : detectedBalls) {
                        telemetry.addData("Color: ", ball.color());
                        dashboard.getTelemetry().addData("Color ", ball.color());
                        telemetry.addData("Position: ", ball.position());
                        dashboard.getTelemetry().addData("Position ", ball.position());
                    }
                }
                else if(FUNCTION.equals(Function.LOCALIZATION.text())) {

                    FtcDashboard.getInstance().getTelemetry().addLine("Loc");

                    Pose3D output = mVision.getPosition();
                    Pose3D prevOutput = null;
                    if (output != null) {
                        telemetry.addData("Pose3D", output);
                        dashboard.getTelemetry().addData("Pose3D ", output);
                        mPreviousOutput = output;
                    } else {
                        if(mPreviousOutput != null) {
                            telemetry.addData("Pose3D", prevOutput);
                            dashboard.getTelemetry().addData("Pose3D", prevOutput);
                        }
                    }
                }

                sleep(100); // Refresh rate

            }
            catch( Exception e) { dashboard.getTelemetry().addLine(e.getMessage()); }


            dashboard.getTelemetry().update();
        }

        mVision.close();
    }

    // Function to transform pixel point and display real-world point

}
