/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at middle
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* System includes */

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousMiddle;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;


@Autonomous
public class AutonomousMiddleStartMOB extends LinearOpMode {

    Vision                  mVision;
    MecanumDrive            mDrive;
    Robot                   mRobot;

    Pattern                 mPattern;
    Pattern                 mTargetPattern;
    int                     mPatternShift = 0;
    Alliance                mAlliance = Alliance.NONE;
    PathAutonomousMiddle    mPath;
    double                  mWaitingTime = 0.0;
    boolean                 mShallGrabAnotherPattern = true;

    SmartTimer              mTimer;

    Controller              mGamepad1;
    Controller              mGamepad2;

    Camera                  mCamera;

    Logger                  mLogger;
    double                  mVelocityFar = 3.3;
    double                  mVelocityClose = 2.7;

    @Override
    public void runOpMode() throws InterruptedException {


        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-middle-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer          = new SmartTimer(mLogger);

        mCamera         = new Camera();
        mCamera.setHW(Configuration.s_Current,hardwareMap,mLogger);

        mVision         = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();
        mPattern        = Pattern.PGP;
        mTargetPattern  = Pattern.PGP;

        mPath           = new PathAutonomousMiddle(mLogger);
        mDrive          = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mGamepad1       = new Controller(gamepad1, mLogger);
        mGamepad2       = new Controller(gamepad2, mLogger);

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, mGamepad2, mPath);

        mCamera.setPosition(Camera.Position.TAG);

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance,mTargetPattern);
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, mTargetPattern);
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce())            { mWaitingTime += 1; mWaitingTime = Math.min(mWaitingTime,4);}
            if (mGamepad1.buttons.dpad_down.pressedOnce())          { mWaitingTime -= 1; mWaitingTime = Math.max(mWaitingTime,0); }

            if (mGamepad1.buttons.x.pressedOnce())  {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift,0);
                if(mPattern == Pattern.GPP) { mTargetPattern = this.computePattern(mPattern,mPatternShift);}
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if (mGamepad1.buttons.b.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift,3);
                if(mPattern == Pattern.GPP) { mTargetPattern = this.computePattern(mPattern,mPatternShift);}
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if(mGamepad1.buttons.a.pressedOnce()){
                mShallGrabAnotherPattern = true;
            }
            if(mGamepad1.buttons.y.pressedOnce()){
                mShallGrabAnotherPattern = false;
            }

            Pattern pattern = mVision.readPattern();
            pattern = Pattern.GPP;
            if (pattern != Pattern.NONE) {
                mPattern = pattern;
                mTargetPattern = this.computePattern(mPattern,mPatternShift);
                mPath.initialize(mAlliance, mTargetPattern);
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> PATTERN : " , mPattern.text());
            mLogger.metric("==> PATTERN SHIFT : " , "" + mPatternShift);
            mLogger.metric("==> PATTERN TARGET : " , mTargetPattern.text());
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());
            mLogger.metric("==> WAITING TIME : ", mWaitingTime + " s");
            if (mShallGrabAnotherPattern) {
                mLogger.metric("==> THIRD GRAB","YES");
            }
            else {
                mLogger.metric("==> THIRD GRAB","NO");
            }
            mPath.log();

            mLogger.update();

        }

        mLogger.info("======= ACTIONS =======");
        mLogger.info("==> GO TO PATTERN");

        mLogger.update();

        mDrive = new MecanumDrive(hardwareMap,mPath.start());

        Action startIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                mDrive.localizer.update();
                return mRobot.start_intake();
            }
        };

        Action stopIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                mDrive.localizer.update();
                return mRobot.stop_intake();
            }
        };

        Action engageFarAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                mDrive.localizer.update();
                return mRobot.start_engage(mVelocityFar);
            }
        };

        Action engageCloseAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                mDrive.localizer.update();
                return mRobot.start_engage(mVelocityClose);
            }
        };

        Pose2d start = mPath.start();Pose2d pattern = mPath.pattern();
        Pose2d next_pattern = mPath.nextPattern();
        Pose2d end_intake = mPath.endIntake();
        Pose2d back_intake = mPath.backIntake();
        Pose2d end_next_intake = mPath.endNextIntake();
        Pose2d back_next_intake = mPath.backNextIntake();
        Pose2d shoot = mPath.shootingFar();
        Pose2d shootinit = mPath.shootingVeryFar();
        Pose2d leave = mPath.leaveVeryFar();
        Pose2d zelie = mPath.zelie();

        Actions.runBlocking(
            mDrive.actionBuilder(start)
                    .waitSeconds(mWaitingTime)
                    .afterTime(0.01, engageFarAction)
                    .setTangent(start.heading.toDouble())
                    .splineToLinearHeading(new Pose2d(shootinit.position.x,shootinit.position.y,start.heading.toDouble()),start.heading.toDouble())
                    .turnTo(shootinit.heading.toDouble())
                    .build());
        mLogger.metric("leave " , ""+leave);
        mLogger.update();
        mRobot.shoot3(mVelocityFar);
        updatePoseFromAprilTagIfVisible();
//      updatePoseFromAprilTagIfVisible();
        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .setTangent(pattern.heading.toDouble())
                        .splineToLinearHeading(leave,pattern.heading.toDouble())
                        .build());


        Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
        Configuration.s_Current.persist("alliance",mAlliance.getValue());

        mVision.close();
        mLogger.stop();

    }

    void updatePoseFromAprilTagIfVisible() {
        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            mLogger.metric("VISION POSE",""+output);

            Pose2d newReference = new Pose2d(
                    -output.getPosition().x * Path.M_TO_INCHES,
                    -output.getPosition().y * Path.M_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            mDrive.localizer.update();
            mDrive.updatePose(newReference);
            mLogger.metric("UPDATED","YES");
            mLogger.metric("UPDATED POSE",""+newReference);
        }
        else {
            mLogger.metric("UPDATED","NO");
        }

        mLogger.update();
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


