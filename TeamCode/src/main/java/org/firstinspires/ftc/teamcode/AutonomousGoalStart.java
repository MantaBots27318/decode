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

    Logger                  mLogger;
    SmartTimer              mTimer;

    Controller              mGamepad1;
    MecanumDrive            mDrive;
    Robot                   mRobot;
    
    Alliance                mAlliance = Alliance.NONE;
    PathAutonomousGoal      mPath;

    Pose2d                  mLimelightPositionInRR;

    List<AutonomousStep>    mSteps;

    @Override
    public void runOpMode() throws InterruptedException {
        
        mLogger                 = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer                  = new SmartTimer(mLogger);

        mGamepad1               = new Controller(gamepad1, mLogger);

        mPath                   = new PathAutonomousGoal(mLogger);
        mLimelightPositionInRR  = Configuration.s_Current.getPosition("turret");

        mDrive                  = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mRobot                  = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, null, mPath);

        mSteps = new ArrayList<AutonomousStep>();
        mSteps.add(AutonomousStep.NONE);
        int current_step  = mSteps.size() - 1;
        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce()) {
                current_step ++;
                if(current_step >= mSteps.size()) { mSteps.add(AutonomousStep.NONE); }
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce()) {
                current_step --;
                if(current_step < 0) { current_step = 0; }
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce()){
                mSteps.set(current_step,mSteps.get(current_step).next());
            }
            if(mGamepad1.buttons.dpad_down.pressedOnce()){
                mSteps.set(current_step,mSteps.get(current_step).previous());
            }

            if(mGamepad1.buttons.y.pressedOnce()){
                mSteps.clear();
                current_step = 0;
                mSteps.add(AutonomousStep.NONE);
            }

            if (mGamepad1.buttons.right_trigger.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance);
            }
            if (mGamepad1.buttons.left_trigger.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance);
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance : TRIGGER LEFT/RIGHT");
            mLogger.info("Add a step      : DPAD LEFT/RIGHT");
            mLogger.info("Modify step     : DPAD UP/DOWN");
            mLogger.info("Clear all steps : Y");
            mLogger.info("");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.info("==> ALLIANCE : " + mAlliance.name());
            StringBuilder steps = new StringBuilder("[");
            for(int i_step = 0; i_step < mSteps.size(); i_step ++)
            {
                if(i_step == current_step) {
                    steps.append(mSteps.get(i_step).text().toUpperCase());
                }
                else {
                    steps.append(mSteps.get(i_step).text().toLowerCase());
                }
                if(i_step < (mSteps.size() - 1)) { steps.append(" , "); }

            }
            steps.append("]");
            mLogger.info("==> STEPS : " + steps);
            mLogger.info("" + current_step);

            mPath.log();
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

                Pose2d start_intake,end_intake, back_intake;
                Pose2d shoot = mPath.shootingFar();

                if (step == AutonomousStep.GPP) {
                    start_intake = mPath.startIntake(Pattern.GPP);
                    end_intake = mPath.endIntake(Pattern.GPP);
                    back_intake = mPath.backIntake(Pattern.GPP);
                }
                else if (step == AutonomousStep.PGP) {
                    start_intake = mPath.startIntake(Pattern.PGP);
                    end_intake = mPath.endIntake(Pattern.PGP);
                    back_intake = mPath.backIntake(Pattern.PGP);
                }
                else {
                    start_intake = mPath.startIntake(Pattern.PPG);
                    end_intake = mPath.endIntake(Pattern.PPG);
                    back_intake = mPath.backIntake(Pattern.PPG);
                }


                Actions.runBlocking(
                        mDrive.actionBuilder(mDrive.getPose())
                                .afterTime(0.01, startStopIntakeAction)
                                .setTangent(-Math.PI)
                                .splineToLinearHeading(start_intake, start_intake.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15, 15))
                                .setTangent(start_intake.heading.toDouble())
                                .splineToLinearHeading(end_intake, end_intake.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                .afterTime(2, startStopIntakeAction)
                                .setTangent(-end_intake.heading.toDouble())
                                .splineToLinearHeading(back_intake, -back_intake.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
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

        Pose2d leave = mPath.leave();
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