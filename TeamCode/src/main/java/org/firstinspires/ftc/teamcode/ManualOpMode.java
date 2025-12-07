package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Robot include */
import org.firstinspires.ftc.teamcode.camera.Camera;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.Poses;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class ManualOpMode extends LinearOpMode {

    Driving     mDriving;
    Collecting  mCollecting;
    Vision      mVision;

    Poses       mPoses;
    Camera      mCamera;
    Controller  mGamepad1;
    Controller  mGamepad2;

    public void runOpMode() throws InterruptedException {

        Alliance alliance = Alliance.NONE;

        Double alliance_value = Configuration.s_Current.retrieve("alliance");
        if(alliance_value == null) { alliance_value = Alliance.BLUE.getValue(); }
        telemetry.addData("value",alliance_value);
        telemetry.addData("red",Alliance.RED.getValue());
        if(Math.abs(alliance_value - Alliance.RED.getValue()) < 0.01) { alliance = Alliance.RED;}
        if(Math.abs(alliance_value - Alliance.BLUE.getValue()) < 0.01){ alliance = Alliance.BLUE;}

        mGamepad1 = new Controller(gamepad1,telemetry);
        mGamepad2 = new Controller(gamepad2,telemetry);

        mPoses = new Poses(FtcDashboard.getInstance().getTelemetry());
        mPoses.initialize(alliance, Vision.Pattern.GPP,true);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();

        mDriving = new Driving();
        mDriving.setHW(Configuration.s_Current, hardwareMap, telemetry, mGamepad1, mVision,mPoses);

        mCollecting = new Collecting();
        mCollecting.setHW(Configuration.s_Current, hardwareMap, telemetry, mGamepad2);


        mCamera = new Camera();
        mCamera.setHW(Configuration.s_Current, hardwareMap, telemetry);
        mCamera.setPosition(Camera.Position.TAG);

        telemetry.addData("Current Selection", alliance);
        telemetry.update();


        waitForStart();
        while (opModeIsActive()){

            try {
                mDriving.control();
                mCollecting.control();
                telemetry.addData("Current Alliance",alliance);
                // Update state machines
                mCollecting.loop();
                telemetry.addLine(mCollecting.logState());
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
