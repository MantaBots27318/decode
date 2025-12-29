package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Robot include */
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.path.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class ManualOpMode extends LinearOpMode {

    Logger      mLogger;

    Driving     mDriving;
    Collecting  mCollecting;
    Vision      mVision;

    Path        mPath;
    Camera      mCamera;
    Controller  mGamepad1;
    Controller  mGamepad2;

    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-middle-start");

        Alliance alliance = Alliance.NONE;

        Double alliance_value = Configuration.s_Current.retrieve("alliance");
        if(alliance_value == null) { alliance_value = Alliance.BLUE.getValue(); }
        telemetry.addData("value",alliance_value);
        telemetry.addData("red",Alliance.RED.getValue());
        if(Math.abs(alliance_value - Alliance.RED.getValue()) < 0.01) { alliance = Alliance.RED;}
        if(Math.abs(alliance_value - Alliance.BLUE.getValue()) < 0.01){ alliance = Alliance.BLUE;}

        mGamepad1 = new Controller(gamepad1,mLogger);
        mGamepad2 = new Controller(gamepad2,mLogger);

        mPath = new Path(mLogger);
        mPath.initialize(alliance, true);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();

        mDriving = new Driving();
        mDriving.setHW(Configuration.s_Current, hardwareMap, telemetry, mGamepad1, mVision, mPath);

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
