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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutonomousGoalStart extends LinearOpMode {

    SmartTimer              mTimer;
    Controller              mGamepad1;
    Logger                  mLogger;

    MecanumDrive            mDrive;
    Robot                   mRobot;
    
    Alliance                mAlliance = Alliance.NONE;
    PathAutonomousGoal      mPath;

    Pose2d                  mLimelightPositionInRR;

    List<AutonomousStep>    mSteps;

    @Override
    public void runOpMode() throws InterruptedException {
        
        mSteps = new ArrayList<AutonomousStep>();

        int mCurrentStepNumber  = 0;

        mLogger                 = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer                  = new SmartTimer(mLogger);

        mGamepad1               = new Controller(gamepad1, mLogger);

        mPath                   = new PathAutonomousGoal(mLogger);
        mLimelightPositionInRR  = Configuration.s_Current.getPosition("turret");

        mDrive                  = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mRobot                  = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, null, mPath);

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce()){
                mCurrentStepNumber = mCurrentStepNumber + 1;
            }
            if(mGamepad1.buttons.dpad_left.pressedOnce() && mCurrentStepNumber > 0){
                mCurrentStepNumber = mCurrentStepNumber - 1;
            }

            if(mGamepad1.buttons.dpad_up.pressedOnce()){
                if(mSteps.size() <= mCurrentStepNumber) { mSteps.add(AutonomousStep.NONE); }
                else{ mSteps.set(mCurrentStepNumber,mSteps.get(mCurrentStepNumber).next());}
            }

            if (mGamepad1.buttons.right_trigger.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance,Pattern.GPP);
            }
            if (mGamepad1.buttons.left_trigger.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, Pattern.GPP);
            }

            if(mGamepad1.buttons.dpad_down.pressedOnce()){
                mSteps.clear();
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());

            mPath.log();
            mLogger.info( "==> STEPS : " + mSteps);

            mLogger.update();

        }

        mLogger.info("======= ACTIONS =======");
        mLogger.info("==> GO TO SHOOTING POSITION");
        mLogger.update();

        mDrive = new MecanumDrive(hardwareMap,mPath.start());

        Action startStopIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                mRobot.start_stop_intake();
                return false;
            }
        };

        Action engageAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                return mRobot.start_engage();
            }
        };

        // Go To Shooting position
        Pose2d start = mPath.start();
        Actions.runBlocking(
                mDrive.actionBuilder(start)
                        .afterTime(0.1,engageAction)
                        .lineToXConstantHeading(mPath.shootingFar().position.x + 3, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))
                        .build());

        mLogger.info("==> SHOOT");
        mLogger.update();

        mRobot.shoot();
        updatePoseFromAprilTagIfVisible();

        // Cycle Through Steps

        for(AutonomousStep step : mSteps) {

            mLogger.info("==> STEP : " + step);

            if(step != AutonomousStep.NONE) {

                if (step == AutonomousStep.GPP)      { mPath.initialize(mAlliance, Pattern.GPP); }
                else if (step == AutonomousStep.PGP) { mPath.initialize(mAlliance, Pattern.PGP); }
                else if (step == AutonomousStep.PPG) { mPath.initialize(mAlliance, Pattern.PPG); }

                Pose2d pattern = mPath.pattern();
                Pose2d end_intake = mPath.endIntake();
                Pose2d back_intake = mPath.backIntake();
                Pose2d shoot = mPath.shootingFar();

                Actions.runBlocking(
                        mDrive.actionBuilder(mDrive.getPose())
                                .afterTime(0.01, startStopIntakeAction)
                                .setTangent(-Math.PI)
                                .splineToLinearHeading(pattern, pattern.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15, 15))
                                .setTangent(pattern.heading.toDouble())
                                .splineToLinearHeading(end_intake, pattern.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                .afterTime(2, startStopIntakeAction)
                                .setTangent(-pattern.heading.toDouble())
                                .splineToLinearHeading(back_intake, -pattern.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                                .afterTime(0.01, engageAction)
                                .setTangent(mPath.tgtIntakeToShootRadians())
                                .splineToLinearHeading(shoot, 0, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-25, 50))
                                .build());

                mRobot.shoot();
                updatePoseFromAprilTagIfVisible();

                mLogger.metric("PATTERN POSE POSITION","" + mDrive.localizer.getPose().position);
                mLogger.metric("PATTERN POSE HEADING","" + mDrive.localizer.getPose().heading.toDouble());
                mLogger.update();

            }

            mLogger.update();
        }

        mPath.initialize(mAlliance, Pattern.GPP);

        Pose2d leave = mPath.parking();
        Pose2d shoot = mPath.shootingFar();

        Actions.runBlocking(
                mDrive.actionBuilder(shoot)
                        .setTangent(shoot.heading.toDouble() + Math.PI)
                        .splineToLinearHeading(leave, leave.heading.toDouble() + Math.PI, new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100,100))
                        .build());

        Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
        Configuration.s_Current.persist("x", mDrive.getPose().position.x);
        Configuration.s_Current.persist("y", mDrive.getPose().position.y);
        Configuration.s_Current.persist("alliance",mAlliance.getValue());

        mLogger.stop();

    }

    void updatePoseFromAprilTagIfVisible() {
        // Gather April Tags
//        Pose3D output = mVision.getPosition();
//        mTimer.arm(100);
//
//        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
//        if (output != null) {
//
//            Pose2d newPose = new Pose2d(
//                    -output.getPosition().x * Path.M_TO_INCHES,
//                    -output.getPosition().y * Path.M_TO_INCHES,
//                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);
//
//            if (mLimelightPositionInRR != null) {
//                newPose = PositionMath.getRobotPoseFromLimelight(newPose, mLimelightPositionInRR);
//            }
//
//            mLogger.metric("NEW POSE POSITION",""+newPose.position);
//            mLogger.metric("NEW POSE HEADING",""+newPose.heading.toDouble());
//
//            mLogger.update();
//
//            mDrive.localizer.update();
//            mDrive.updatePose(newPose);
//            mLogger.metric("UPDATED POSE POSITION",""+mDrive.localizer.getPose().position);
//            mLogger.metric("UPDATED POSE HEADING",""+mDrive.localizer.getPose().heading.toDouble());
//            mLogger.update();
//
//            mLogger.metric("==> POSE","UPDATED");
//        }
    }


}