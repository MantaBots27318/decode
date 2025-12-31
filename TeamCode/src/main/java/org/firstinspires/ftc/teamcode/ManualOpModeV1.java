package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Robot include */
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class ManualOpModeV1 extends LinearOpMode {

    Logger      mLogger;

    Alliance    mAlliance;

    Driving     mDriving;
    Collecting  mCollecting;
    Vision      mVision;

    Path        mPath;
    Camera      mCamera;
    Controller  mGamepad1;
    Controller  mGamepad2;

    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"manual");

        mAlliance = Alliance.NONE;
        Double alliance_value = Configuration.s_Current.retrieve("alliance");
        if(alliance_value == null) { alliance_value = Alliance.BLUE.getValue(); }
        if(Math.abs(alliance_value - Alliance.RED.getValue()) < 0.01)  { mAlliance = Alliance.RED;}
        if(Math.abs(alliance_value - Alliance.BLUE.getValue()) < 0.01) { mAlliance = Alliance.BLUE;}

        mGamepad1 = new Controller(gamepad1,mLogger);
        mGamepad2 = new Controller(gamepad2,mLogger);

        mPath = new Path(mLogger);
        mPath.initialize(mAlliance, true);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();

        mDriving = new Driving();
        mDriving.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, mVision, mPath);

        mCollecting = new Collecting();
        mCollecting.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad2);

        mCamera = new Camera();
        mCamera.setHW(Configuration.s_Current, hardwareMap, mLogger);
        mCamera.setPosition(Camera.Position.TAG);

        mLogger.info("ALL : " +  mAlliance);
        mLogger.update();

        waitForStart();
        while (opModeIsActive()){

            try {

                mDriving.control();

                mCollecting.control();
                mCollecting.loop();

                mLogger.info(mCollecting.logState());
                mLogger.update();
            } catch (Exception e) {
                mLogger.error("LOOP error : " + e.getMessage());
                mLogger.update();
            }
        }

        Configuration.s_Current.reinit();
        mVision.close();
    }


            // Make sure that once Teleop is over, we reset all the persisted data



    }
