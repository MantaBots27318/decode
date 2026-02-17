/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at goal
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Android includes */

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PositionMath;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;

import javax.xml.XMLConstants;


@Autonomous
public class AutonomousGoalStart extends LinearOpMode {

    List<AutonomousStep> mListActions;

    Vision              mVision;
    MecanumDrive        mDrive;
    Robot               mRobot;

    Pattern             mPattern;
    Pattern             mTargetPattern;
    Pattern             mThirdPattern = Pattern.PPG;
    int                 mPatternShift = 0;
    Alliance            mAlliance = Alliance.NONE;
    PathAutonomousGoal  mPath;

    Pose2d              mLimelightPositionInRR;

    SmartTimer          mTimer;

    Controller          mGamepad1;
    Controller          mGamepad2;
    Camera              mCamera;

    Logger              mLogger;
    boolean             mShallGrabAnotherPattern;

    double              mShootVelocity = 2.7;

    @Override
    public void runOpMode() throws InterruptedException {
        mListActions = new ArrayList<AutonomousStep>();

        int mCurrentStepNumber     = 0;

        mLogger                 = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer                  = new SmartTimer(mLogger);

        mCamera                 = new Camera();
        mCamera.setHW(Configuration.s_Current,hardwareMap,mLogger);

        mVision                 = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();
        mPattern                = Pattern.PGP;
        mTargetPattern          = Pattern.PGP;

        mPath                   = new PathAutonomousGoal(mLogger);
        mLimelightPositionInRR  = Configuration.s_Current.getPosition("turret");

        mDrive                  = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mGamepad1               = new Controller(gamepad1, mLogger);
        mGamepad2               = new Controller(gamepad2, mLogger);

        mRobot                  = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, mGamepad2, mPath);

        mCamera.setPosition(Camera.Position.TAG);
        mShallGrabAnotherPattern = true;

        while (opModeInInit()) {
            if (gamepad1.dpad_right){
                mCurrentStepNumber = mCurrentStepNumber + 1;
            }
            if(gamepad1.dpad_left){
                mCurrentStepNumber = mCurrentStepNumber - 1;
            }
            if(gamepad1.dpad_up){
                mListActions.set(mCurrentStepNumber,mListActions.get(mCurrentStepNumber).next());
            }

            if (mGamepad1.buttons.dpad_right.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance,mTargetPattern);
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, mTargetPattern);
            }

            if (mGamepad1.buttons.x.pressedOnce())  {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift,0);
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if (mGamepad1.buttons.b.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift,3);
                mPath.initialize(mAlliance, mTargetPattern);
            }
            if(mGamepad1.buttons.a.pressedOnce()){
                mShallGrabAnotherPattern = true;
            }
            if(mGamepad1.buttons.y.pressedOnce()){
                mShallGrabAnotherPattern = false;
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());
            mLogger.metric("==> PATTERN SHIFT : ", ""+mPatternShift);

            if (mShallGrabAnotherPattern) {
                mLogger.metric("==> THIRD GRAB","YES");
            }
            else {
                mLogger.metric("==> THIRD GRAB","NO");
            }
            mPath.log();
            mLogger.info( "List"+mListActions);

            mLogger.update();

        }

        mLogger.info("======= ACTIONS =======");
        mLogger.info("==> GO TO SHOOTING POSITION");
        mLogger.update();

        mDrive = new MecanumDrive(hardwareMap,mPath.start());

        Action startIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mRobot.start_intake();
            }
        };

        Action stopIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mRobot.stop_intake();
            }
        };

        Action engageAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                return mRobot.start_engage(mShootVelocity);
            }
        };

        Pose2d start = mPath.start();

        Actions.runBlocking(
                mDrive.actionBuilder(start)
                        .afterTime(0.1,engageAction)
                        .lineToXConstantHeading(mPath.shootingFar().position.x + 3, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))
                        .build());

        mLogger.info("==> Shoot");
        mLogger.update();

        mRobot.shoot3(mShootVelocity);
        updatePoseFromAprilTagIfVisible();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .turn(mPath.hObeliskFTCRadians() )
                        .build());

        mTimer.arm(100);
        Pattern pat = mVision.readPattern();
        mLogger.metric("READ PATTERN" , ""+pat);
        while(pat == Pattern.NONE && mTimer.isArmed()) {
            pat = mVision.readPattern();
            mLogger.metric("READ PATTERN" , ""+pat);
        }
        if(pat == Pattern.NONE) { pat = Pattern.PGP; }

        mPattern = pat;
        mTargetPattern = this.computePattern(mPattern,mPatternShift);
        mPath.initialize(mAlliance, mTargetPattern);
        if(mTargetPattern == Pattern.PPG) { mThirdPattern = Pattern.PGP; }

        mLogger.metric("==> PATTERN : " , mPattern.text());
        mLogger.metric("==> PATTERN SHIFT : " , "" + mPatternShift);
        mLogger.metric("==> PATTERN TARGET : " , mTargetPattern.text());
        mLogger.update();

        Pose2d pattern = mPath.pattern();
        Pose2d next_pattern = mPath.nextPattern();
        Pose2d end_intake = mPath.endIntake();
        Pose2d back_intake = mPath.backIntake();
        Pose2d end_next_intake = mPath.endNextIntake();
        Pose2d back_next_intake = mPath.backNextIntake();
        Pose2d shoot = mPath.shootingFar();
        Pose2d leave = mPath.parking();
        Pose2d ready = mPath.ready();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .afterTime(0.01,startIntakeAction)
                        .setTangent(mPath.hObeliskFTCRadians() - Math.PI)
                        .splineToLinearHeading(pattern,pattern.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,15))
                        .setTangent(pattern.heading.toDouble())
                        .splineToLinearHeading(end_intake,pattern.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15,15))
                        .afterTime(2,stopIntakeAction)
                        .setTangent(-pattern.heading.toDouble())
                        .splineToLinearHeading(back_intake,-pattern.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100,100))
                        .afterTime(0.01,engageAction)
                        .setTangent(mPath.tgtIntakeToShootRadians())
                        .splineToLinearHeading(shoot,0, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-25,50))
                        .build());

        mRobot.shoot3(mShootVelocity) ;

        if(mShallGrabAnotherPattern) {
            Actions.runBlocking(
                    mDrive.actionBuilder(mDrive.getPose())
                            .afterTime(0.01,startIntakeAction)
                            .setTangent(mPath.hObeliskFTCRadians() - Math.PI)
                            .splineToLinearHeading(next_pattern,next_pattern.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,15))
                            .setTangent(next_pattern.heading.toDouble())
                            .splineToLinearHeading(end_next_intake,next_pattern.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15,15))
                            .afterTime(2,stopIntakeAction)
                            .setTangent(-next_pattern.heading.toDouble())
                            .splineToLinearHeading(ready,ready.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100,100))
                            .build());

        }
        else {
            Actions.runBlocking(
                    mDrive.actionBuilder(shoot)
                            .setTangent(shoot.heading.toDouble() + Math.PI)
                            .splineToLinearHeading(leave, leave.heading.toDouble() + Math.PI, new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100,100))
                            .build());
        }


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

            Pose2d newPose = new Pose2d(
                    -output.getPosition().x * Path.M_TO_INCHES,
                    -output.getPosition().y * Path.M_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            // Ajouter l appel a la fonction de Zelie
            if(mLimelightPositionInRR != null) {
                newPose = PositionMath.getRobotPoseFromLimelight(newPose,mLimelightPositionInRR);
            }

            mDrive.localizer.update();
            mDrive.updatePose(newPose);
            mLogger.metric("==> POSE","UPDATED");
        }
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