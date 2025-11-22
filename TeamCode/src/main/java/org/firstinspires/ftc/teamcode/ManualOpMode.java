package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Robot include */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    Driving mDriving;
    Collecting mCollecting;
    Vision mVision;
    public enum Alliance {
        Blue,
        Red,
        None
    }
    Alliance alliance = Alliance.Blue;

    boolean dpad_rightWasPressed = false;
    boolean dpad_leftWasPressed = false;
    public void runOpMode() throws InterruptedException {


        while (opModeInInit()) {

            try {
                mDriving = new Driving();
                mCollecting = new Collecting();
                mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);

                mVision.initialize();

                mDriving.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad1, mVision);
                mCollecting.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad2);
                if (gamepad1.dpad_right && !dpad_rightWasPressed) {
                    alliance = Alliance.Red;
                }
                dpad_rightWasPressed = gamepad1.dpad_right;

                // Toggle BLUE
                if (gamepad1.dpad_left && !dpad_leftWasPressed) {
                    alliance = Alliance.Blue;
                }
                dpad_leftWasPressed = gamepad1.dpad_left;

                // Display menu
                telemetry.addLine("=== TELEOP CONFIG MENU ===");
                telemetry.addLine("Choose Alliance:");
                telemetry.addData("Right", "RED");
                telemetry.addData("Left", "BLUE");
                telemetry.addData("Current Selection", alliance);
                telemetry.update();

            } catch (Exception e) {
                telemetry.addLine("INIT error : " + e.getMessage());
            }

        }

        while (opModeIsActive()){
            try {
                mDriving.control();
                mCollecting.control();

                // Update state machines
                mCollecting.loop();
                telemetry.update();
                FtcDashboard.getInstance().getTelemetry().update();
            } catch (Exception e) {
                telemetry.addLine("LOOP error : " + e.getMessage());
            }
        }
        Configuration.s_Current.reinit();
        mVision.close();
    }


            // Make sure that once Teleop is over, we reset all the persisted data



    }
