package org.firstinspires.ftc.teamcode;

/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at middle
   ------------------------------------------------------- */


/* System includes */
import androidx.annotation.NonNull;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousShootFar;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;


@Autonomous
public class AutonomousShootFar extends LinearOpMode {

    Vision mVision;
    MecanumDrive mDrive;
    Robot mRobot;

    Pattern mPattern;
    Pattern mTargetPattern;
    int mPatternShift = 0;
    Alliance mAlliance = Alliance.NONE;
    PathAutonomousShootFar mPath;
    double mWaitingTime = 0.0;
    boolean mShallParkInLaunchZone = false;

    SmartTimer mTimer;

    Controller mGamepad1;
    Controller mGamepad2;

    Camera mCamera;

    Logger mLogger;

    @Override
    public void runOpMode() throws InterruptedException {


        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-middle-start");
        mTimer = new SmartTimer(mLogger);

        mCamera = new Camera();
        mCamera.setHW(Configuration.s_Current, hardwareMap, mLogger);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();
        mPattern = Pattern.PGP;
        mTargetPattern = Pattern.PGP;

        mPath = new PathAutonomousShootFar(mLogger);
        mDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        mGamepad1 = new Controller(gamepad1, mLogger);
        mGamepad2 = new Controller(gamepad2, mLogger);

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, mGamepad2, mPath);

        mCamera.setPosition(Camera.Position.TAG);

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce()) {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce()) {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce()) {
                mWaitingTime += 1;
                mWaitingTime = Math.min(mWaitingTime, 4);
            }
            if (mGamepad1.buttons.dpad_down.pressedOnce()) {
                mWaitingTime -= 1;
                mWaitingTime = Math.max(mWaitingTime, 0);
            }

            if (mGamepad1.buttons.x.pressedOnce()) {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift, 0);
                if (mPattern == Pattern.GPP) {
                    mTargetPattern = this.computePattern(mPattern, mPatternShift);
                }
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.b.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift, 3);
                if (mPattern == Pattern.GPP) {
                    mTargetPattern = this.computePattern(mPattern, mPatternShift);
                }
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.y.pressedOnce()) {
                mShallParkInLaunchZone = true;
            }
            if (mGamepad1.buttons.a.pressedOnce()) {
                mShallParkInLaunchZone = false;
            }

            Pattern pattern = mVision.readPattern();
            if (pattern != Pattern.NONE) {
                mPattern = pattern;
                if (mPattern == Pattern.PPG) {
                    mTargetPattern = Pattern.PGP;
                }
                if (mPattern == Pattern.PGP) {
                    mTargetPattern = Pattern.PPG;
                }
                if (mPattern == Pattern.GPP) {
                    mTargetPattern = this.computePattern(mPattern, mPatternShift);
                }
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> PATTERN : ", mPattern.text());
            mLogger.metric("==> PATTERN SHIFT : ", "" + mPatternShift);
            mLogger.metric("==> PATTERN TARGET : ", mTargetPattern.text());
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());
            mLogger.metric("==> WAITING TIME : ", mWaitingTime + " s");
            if (!mShallParkInLaunchZone) {
                mLogger.metric("==> PARKING POSITION", "Gate Zone");
            }
            if (mShallParkInLaunchZone) {
                mLogger.metric("==> PARKING POSITION", "Launch Zone");
            }


        }
        Action startIntakeAction = p -> mRobot.start_intake();

        Action stopIntakeAction = p -> mRobot.stop_intake();

        Action engageAction = p -> mRobot.start_engage(2.8);



        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                .splineToLinearHeading(mPath.shoot(),mPath.shoot().heading)
                .build()
        );

        mRobot.shoot3(3.25);
        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .splineToLinearHeading(mPath.pattern(),mPath.pattern().heading)
                        .build()
        );
        double distance_pattern = mPath.pattern().minus(mPath.start()).line.norm();
        double distance_intake = mPath.endIntake().minus(mPath.pattern()).line.norm();
        Actions.runBlocking(
                mDrive.actionBuilder(mPath.start())
                        .waitSeconds(mWaitingTime)
                        .afterDisp(0.4 * distance_pattern,startIntakeAction)
                        .setTangent(mPath.start().heading)
                        .splineToLinearHeading(mPath.pattern(),mPath.start().heading)
                        .afterDisp(0.9 * distance_intake,stopIntakeAction)
                        .setTangent(mPath.pattern().heading.toDouble())
                        .splineToLinearHeading(mPath.endIntake(),mPath.pattern().heading.toDouble(), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-15,15))
                        .setTangent(-mPath.endIntake().heading.toDouble())
                        .splineToLinearHeading(mPath.backIntake(), -mPath.endIntake().heading.toDouble(), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))
                        .afterDisp(0.1,engageAction)
                        .setTangent(mPath.tgtIntakeToCalibrationRadians())
                        .splineToLinearHeading(mPath.calibration(),0, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30,30))
                        .build()
        );
        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .splineToLinearHeading(mPath.shoot(),mPath.shoot().heading)
                        .build()
        );
        mRobot.shoot3(3.25);


    }
    Pattern  computePattern(Pattern official, int shift) {

        Pattern result = Pattern.NONE;

        if(official != Pattern.NONE) {
            int target_identifier = (official.identifier() + shift) % 3;
            if (target_identifier == Pattern.GPP.identifier()) {
                result = Pattern.GPP;
            }
            if (target_identifier == Pattern.PGP.identifier()) {
                result = Pattern.PGP;
            }
            if (target_identifier == Pattern.PPG.identifier()) {
                result = Pattern.PPG;
            }
        }

        return result;
    }
}
