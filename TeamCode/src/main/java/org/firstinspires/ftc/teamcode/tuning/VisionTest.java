package org.firstinspires.ftc.teamcode.tuning;

// FTCController includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Roadrunner includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Local includes */
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.List;

@Config
@TeleOp
public class VisionTest extends LinearOpMode {

    // Inputs (editable from FTC Dashboard)
    private Vision mVision;


    @Override
    public void runOpMode() throws InterruptedException {

        // Setup the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Example source points (pixels in image)

        mVision = new Vision(dashboard.getTelemetry());
        mVision.initialize(hardwareMap);
        dashboard.getTelemetry().update();

        waitForStart();

        while (opModeIsActive()) {

            try {
                // Transform and send back through dashboard
                Vision.Pattern pattern = mVision.readPattern();

                telemetry.addData("Pattern ", pattern.text());
                dashboard.getTelemetry().addData("Pattern ", pattern.text());

                sleep(100); // Refresh rate
                List <Ball> detectedBalls = mVision.getArtifactPosition();
                for (Ball ball : detectedBalls ) {
                    telemetry.addData("Color: ",ball.color());
                    dashboard.getTelemetry().addData("Color ", ball.color());
                    telemetry.addData("Position: ",ball.position() );
                    dashboard.getTelemetry().addData("Position ", ball.position());
                }

                telemetry.addData("Pose3D",mVision.getPosition("blue"));
                dashboard.getTelemetry().addData("Pose3D",mVision.getPosition("blue"));

            }
            catch( Exception e) { dashboard.getTelemetry().addLine(e.getMessage()); }


            dashboard.getTelemetry().update();
        }

        mVision.close();
    }

    // Function to transform pixel point and display real-world point

}
