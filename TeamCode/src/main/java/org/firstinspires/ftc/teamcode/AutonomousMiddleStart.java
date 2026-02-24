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
import org.firstinspires.ftc.teamcode.pose.Posable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutonomousMiddleStart extends LinearOpMode {

    List<AutonomousStep> mListActions;
    Vision mVision;
    MecanumDrive mDrive;
    Robot mRobot;

    Pattern mPattern;
    Pattern mTargetPattern;
    int mPatternShift = 0;
    Alliance mAlliance = Alliance.NONE;
    PathAutonomousMiddle mPath;
    Pose2d mLimelightPositionInRR;

    double mWaitingTime = 0.0;
    boolean mShallGrabAnotherPattern = true;

    SmartTimer mTimer;

    Controller mGamepad1;
    Controller mGamepad2;

    Logger mLogger;

    @Override
    public void runOpMode() throws InterruptedException {
        mListActions = new ArrayList<AutonomousStep>();

        int mCurrentStepNumber = 0;

        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-middle-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer = new SmartTimer(mLogger);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();
        mPattern = Pattern.PGP;
        mTargetPattern = Pattern.PGP;

        mPath = new PathAutonomousMiddle(mLogger);
        mLimelightPositionInRR = Configuration.s_Current.getPosition("turret");

        mDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        mGamepad1 = new Controller(gamepad1, mLogger);
        mGamepad2 = new Controller(gamepad2, mLogger);

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, mGamepad2, mPath);

        while (opModeInInit()) {
            if (mGamepad1.buttons.dpad_right.pressedOnce()) {
                mCurrentStepNumber = mCurrentStepNumber + 1;
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce() && mCurrentStepNumber > 0) {
                mCurrentStepNumber = mCurrentStepNumber - 1;

            }
            if (mGamepad1.buttons.dpad_up.pressedOnce()) {
                if (mListActions.size() <= mCurrentStepNumber) {
                    mListActions.add(AutonomousStep.NONE);
                } else {
                    mListActions.set(mCurrentStepNumber, mListActions.get(mCurrentStepNumber).next());
                }
            }

            if (mGamepad1.buttons.right_trigger.pressedOnce()) {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if (mGamepad1.buttons.left_trigger.pressedOnce()) {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, mTargetPattern);
            }

            if (mGamepad1.buttons.x.pressedOnce()) {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift, 0);
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if (mGamepad1.buttons.b.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift, 3);
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if (mGamepad1.buttons.a.pressedOnce()) {
                mShallGrabAnotherPattern = true;
            }
            if (mGamepad1.buttons.y.pressedOnce()) {
                mShallGrabAnotherPattern = false;
            }
            if (mGamepad1.buttons.dpad_down.pressedOnce()) {
                mListActions.clear();
            }


            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());
            mLogger.metric("==> PATTERN SHIFT : ", "" + mPatternShift);

            if (mShallGrabAnotherPattern) {
                mLogger.metric("==> THIRD GRAB", "YES");
            } else {
                mLogger.metric("==> THIRD GRAB", "NO");
            }
            mPath.log();
            mLogger.info( "List"+mListActions);
            mLogger.update();

        }

        mLogger.info("======= ACTIONS =======");
        mLogger.info("==> GO TO PATTERN");

        mLogger.update();

        mDrive = new MecanumDrive(hardwareMap, mPath.start());

        Action startStopIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                mRobot.start_stop_intake();
                return false;
            }
        };

        Action engageFarAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                mDrive.localizer.update();
                return mRobot.start_engage();
            }
        };

        Action engageCloseAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                mDrive.localizer.update();
                return mRobot.start_engage();
            }
        };

        Pose2d start = mPath.start();
        Pose2d shootinit = mPath.shootingVeryFar();

        Actions.runBlocking(
                mDrive.actionBuilder(start)
                        .waitSeconds(mWaitingTime)
                        .afterTime(0.01, engageFarAction)
                        .setTangent(start.heading.toDouble())
                        .splineToLinearHeading(new Pose2d(shootinit.position.x, shootinit.position.y, start.heading.toDouble()), start.heading.toDouble())
                        .turnTo(shootinit.heading.toDouble())
                        .build());

        mRobot.shoot();
        updatePoseFromAprilTagIfVisible();

        mLogger.metric("SHOOT POSE POSITION",""+mDrive.localizer.getPose().position);
        mLogger.metric("SHOOT POSE HEADING",""+mDrive.localizer.getPose().heading.toDouble());
        mLogger.update();

        for (AutonomousStep step : mListActions) {
            mLogger.info("" + step);
            if (step != AutonomousStep.NONE) {
                if (step == AutonomousStep.GPP) {
                    mPath.initialize(mAlliance, Pattern.GPP);
                } else if (step == AutonomousStep.PGP) {
                    mPath.initialize(mAlliance, Pattern.PGP);
                } else if (step == AutonomousStep.PPG) {
                    mPath.initialize(mAlliance, Pattern.PPG);
                }


                Pose2d pattern = mPath.pattern();
                Pose2d end_intake = mPath.endIntake();
                Pose2d back_intake = mPath.backIntake();
                Pose2d shoot = mPath.shootingVeryFar();
                Pose2d leave = mPath.parking();
                Pose2d ready = mPath.ready();

                Actions.runBlocking(
                        mDrive.actionBuilder(shootinit)
                                .afterTime(0.01, startStopIntakeAction)
                                .setTangent(shootinit.heading.toDouble())
                                .splineToLinearHeading(pattern, pattern.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15, 15))
                                .setTangent(pattern.heading.toDouble())
                                .splineToLinearHeading(end_intake, pattern.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                .afterTime(2, startStopIntakeAction)
                                .setTangent(-end_intake.heading.toDouble())
                                .splineToLinearHeading(back_intake, -end_intake.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                                .afterTime(0, engageFarAction)
                                .setTangent(-back_intake.heading.toDouble())
                                .splineToLinearHeading(shoot, Math.PI, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-25, 50))
                                .build());

                mRobot.shoot();
                updatePoseFromAprilTagIfVisible();

            }
            mLogger.update();
        }

//                Actions.runBlocking(
//                    mDrive.actionBuilder(shoot)
//                            .setTangent(shoot.heading.toDouble() + Math.PI)
//                            .splineToLinearHeading(leave, leave.heading.toDouble() + Math.PI, new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
//                            .build());


        Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
        Configuration.s_Current.persist("x", mDrive.getPose().position.x);
        Configuration.s_Current.persist("y", mDrive.getPose().position.y);
        Configuration.s_Current.persist("alliance", mAlliance.getValue());

        mVision.close();
        mLogger.stop();

    }

        void updatePoseFromAprilTagIfVisible () {
            // Gather April Tags
            Pose3D output = mVision.getPosition();
            mTimer.arm(100);

            while (output == null && mTimer.isArmed()) {
                output = mVision.getPosition();
            }
            if (output != null) {

                mLogger.metric("VISION POSE", "" + output);

                Pose2d newReference = new Pose2d(
                        -output.getPosition().x * Path.M_TO_INCHES,
                        -output.getPosition().y * Path.M_TO_INCHES,
                        (output.getOrientation().getYaw() + 180) * Math.PI / 180);
                // Ajouter l appel a la fonction de Zelie
                if (mLimelightPositionInRR != null) {
                    newReference = Posable.derivePose(newReference, mLimelightPositionInRR);
                }

                mDrive.localizer.update();
                mDrive.updatePose(newReference);
                mLogger.metric("UPDATED", "YES");
                mLogger.metric("UPDATED POSE", "" + newReference);
            } else {
                mLogger.metric("UPDATED", "NO");
            }

            mLogger.update();
        }

}



