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
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousMiddle;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutonomousMiddleStart extends LinearOpMode {

    Logger                  mLogger;
    SmartTimer              mTimer;

    List<AutonomousStep>    mSteps;
    Alliance                mAlliance = Alliance.NONE;
    PathAutonomousMiddle    mPath;
    Controller              mGamepad1;

    Vision                  mVision;
    MecanumDrive            mDrive;
    Robot                   mRobot;

    Pose2d                  mLimelightPositionInRR;


    @Override
    public void runOpMode() throws InterruptedException {

        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-middle-start");
        mLogger.level(Logger.Severity.INFO);
        mTimer = new SmartTimer(mLogger);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);
        mVision.initialize();

        mPath = new PathAutonomousMiddle(mLogger);
        mLimelightPositionInRR = Configuration.s_Current.getPosition("turret");

        mDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        mGamepad1 = new Controller(gamepad1, mLogger);

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, null, mPath);

        mSteps = new ArrayList<>();
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
            StringBuilder steps = getStringBuilder(current_step, mSteps);
            mLogger.info("==> STEPS : " + steps);

            mPath.log();
            mLogger.update();

        }

        Pose2d start = mPath.start();
        Pose2d shoot = mPath.shootingVeryFar();
        mDrive = new MecanumDrive(hardwareMap, start);
        mRobot.initialize(start, Robot.Mode.AUTONOMOUS);

        Action startStopIntakeAction = p -> {
            mRobot.start_stop_intake();
            return false;
        };

        Action engageAction = p -> {
            mRobot.start_stop_flywheel();
            return false;
        };

        Action loopAction = p -> {
            mRobot.loop();
            return true;
        };

        mLogger.metric("STEP", "GO TO SHOOTING");
        mLogger.update();

        Actions.runBlocking(
            new RaceAction(
                mDrive.actionBuilder(start)
                        .afterTime(0.01, engageAction)
                        .setTangent(start.heading.toDouble())
                        .splineToLinearHeading(new Pose2d(shoot.position.x, shoot.position.y, start.heading.toDouble()), start.heading.toDouble())
                        .turnTo(shoot.heading.toDouble())
                        .build(),
                loopAction));


        mLogger.metric("STEP", "SHOOT");
        mLogger.update();

        mRobot.shoot();
        mRobot.loop();

        for (AutonomousStep step : mSteps) {

            if (step != AutonomousStep.NONE) {

                Pattern pattern;
                if (step == AutonomousStep.GPP) { pattern = Pattern.GPP; }
                else if (step == AutonomousStep.PGP) { pattern = Pattern.PGP; }
                else { pattern = Pattern.PPG; }

                Pose2d start_intake = mPath.startIntake(pattern);
                Pose2d end_intake = mPath.endIntake(pattern);
                Pose2d back_intake = mPath.backIntake(pattern);

                mLogger.metric("STEP", "GO TO AND BACK " + pattern.text() );
                mLogger.update();


                Actions.runBlocking(
                    new RaceAction(
                        mDrive.actionBuilder(shoot)
                                .afterTime(0.01, startStopIntakeAction)
                                .setTangent(shoot.heading.toDouble())
                                .splineToLinearHeading(start_intake, start_intake.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15, 15))
                                .setTangent(start_intake.heading.toDouble())
                                .splineToLinearHeading(end_intake, end_intake.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                .afterTime(2, startStopIntakeAction)
                                .setTangent(-end_intake.heading.toDouble())
                                .splineToLinearHeading(back_intake, -end_intake.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                                .afterTime(0, engageAction)
                                .setTangent(-back_intake.heading.toDouble())
                                .splineToLinearHeading(shoot, Math.PI, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-25, 50))
                                .build(),
                        loopAction));

                mLogger.metric("STEP", "SHOOT" );
                mLogger.update();

                mRobot.shoot();
                mRobot.loop();

            }
        }

        mLogger.metric("STEP", "LEAVE" );
        mLogger.update();

        Pose2d leave = mPath.leaveVeryFar();
        double tgt = mPath.tgtShootToLeaveRadians();

        Actions.runBlocking(
            new RaceAction(
                mDrive.actionBuilder(shoot)
                            .setTangent(tgt)
                            .splineToLinearHeading(leave, tgt, new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                            .build(),
               loopAction));

        Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
        Configuration.s_Current.persist("x", mDrive.getPose().position.x);
        Configuration.s_Current.persist("y", mDrive.getPose().position.y);
        Configuration.s_Current.persist("alliance", mAlliance.getValue());

        mVision.close();
        mLogger.stop();

    }

    @NonNull
    private static StringBuilder getStringBuilder(int current_step, List<AutonomousStep> mSteps) {
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
        return steps;
    }

}



