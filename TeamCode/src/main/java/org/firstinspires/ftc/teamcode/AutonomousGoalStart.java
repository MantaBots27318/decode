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
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Pattern;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutonomousGoalStart extends LinearOpMode {

    Logger mLogger;

    Controller mGamepad1;
    MecanumDrive mDrive;
    Robot mRobot;

    Alliance mAlliance = Alliance.NONE;
    PathAutonomousGoal mPath;

    List<AutonomousStep> mSteps;

    @Override
    public void runOpMode() throws InterruptedException {

        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);

        mGamepad1 = new Controller(gamepad1, mLogger);

        mPath = new PathAutonomousGoal(mLogger);

        mDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, null, mPath);

        mSteps = new ArrayList<>();
        mSteps.add(AutonomousStep.NONE);
        int current_step = mSteps.size() - 1;

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce()) {
                current_step++;
                if (current_step >= mSteps.size()) {
                    mSteps.add(AutonomousStep.NONE);
                }
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce()) {
                current_step--;
                if (current_step < 0) {
                    current_step = 0;
                }
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce()) {
                mSteps.set(current_step, mSteps.get(current_step).next());
            }
            if (mGamepad1.buttons.dpad_down.pressedOnce()) {
                mSteps.set(current_step, mSteps.get(current_step).previous());
            }

            if (mGamepad1.buttons.y.pressedOnce()) {
                mSteps.clear();
                current_step = 0;
                mSteps.add(AutonomousStep.NONE);
            }

            if (mGamepad1.buttons.right_trigger.pressedOnce()) {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance);
            }
            if (mGamepad1.buttons.left_trigger.pressedOnce()) {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance);
            }

            mLogger.info(Logger.Target.DRIVER_STATION,"=========== MENU ============");
            mLogger.info(Logger.Target.DRIVER_STATION,"Choose Alliance : TRIGGER LEFT/RIGHT");
            mLogger.info(Logger.Target.DRIVER_STATION,"Add a step      : DPAD LEFT/RIGHT");
            mLogger.info(Logger.Target.DRIVER_STATION,"Modify step     : DPAD UP/DOWN");
            mLogger.info(Logger.Target.DRIVER_STATION,"Clear all steps : Y");
            mLogger.info(Logger.Target.DRIVER_STATION,"");

            mLogger.info(Logger.Target.DRIVER_STATION,"======= CONFIGURATION =======");
            mLogger.info(Logger.Target.DRIVER_STATION,"==> ALLIANCE : " + mAlliance.name());
            StringBuilder steps = getStringBuilder(current_step, mSteps);
            mLogger.info(Logger.Target.DRIVER_STATION,"==> STEPS : " + steps);

            mPath.log();
            mLogger.update();

        }

        Pose2d start = mPath.start();
        Pose2d shoot = mPath.shootingFar();

        mRobot.initialize(start, Robot.Mode.AUTONOMOUS);
        mDrive = new MecanumDrive(hardwareMap, start);

        mRobot.start_stop_flywheel();
        Thread.sleep(200);
        mRobot.start_stop_intake();

        Action loopAction = p -> {
            mRobot.loop();
            return true;
        };

        mLogger.metric("STEP", "GO TO SHOOTING");
        mLogger.update();

        Actions.runBlocking(
                new RaceAction(
                        mDrive.actionBuilder(start)
                                .setTangent(start.heading.toDouble() + Math.PI)
                                .splineToLinearHeading(shoot, start.heading.toDouble() + Math.PI, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-30, 50))
                                .build(),
                        loopAction
                ));



        mRobot.loop();
        Thread.sleep(100); // Give the flywheel time to reach back its velocity, now that wheel motors are stopped
        mLogger.metric("STEP", "SHOOT");
        mLogger.update();
        mRobot.shoot();
        mRobot.loop();

        mLogger.update();

        int i_step = 0;
        for (AutonomousStep step : mSteps) {

            if (step != AutonomousStep.NONE) {

                Pattern pattern;
                if (step == AutonomousStep.GPP) { pattern = Pattern.GPP; }
                else if (step == AutonomousStep.PGP) { pattern = Pattern.PGP; }
                else { pattern = Pattern.PPG; }

                Pose2d start_intake = mPath.startIntake(pattern);
                Pose2d end_intake = mPath.endIntake(pattern);
                Pose2d leave = mPath.leave();

                mLogger.metric("STEP", "GO TO AND BACK " + pattern.text() );
                mLogger.update();

                if(i_step < (mSteps.size() - 1)) {

                    mLogger.metric("STEP", "TO SHOOT" );

                    Actions.runBlocking(
                            new RaceAction(
                                    mDrive.actionBuilder(mDrive.getPose())
                                            .setTangent(-Math.PI)
                                            .splineToConstantHeading(start_intake.position, start_intake.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                            .setTangent(start_intake.heading.toDouble())
                                            .splineToConstantHeading(end_intake.position, end_intake.heading.toDouble(), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                                            .setTangent(mPath.tgtIntakeToShootRadians())
                                            .splineToConstantHeading(shoot.position, mPath.tgtIntakeToShootRadians(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-50, 50))
                                            .build(),
                                    loopAction
                            ));
                }
                else {

                    mLogger.metric("STEP", "TO LEAVE" );
                    Actions.runBlocking(
                            new RaceAction(
                                    mDrive.actionBuilder(mDrive.getPose())
                                            .setTangent(-Math.PI)
                                            .splineToConstantHeading(start_intake.position, start_intake.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                            .setTangent(start_intake.heading.toDouble())
                                            .splineToConstantHeading(end_intake.position, end_intake.heading.toDouble(), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                                            .setTangent(mPath.tgtIntakeToShootRadians())
                                            .splineToLinearHeading(leave, mPath.tgtIntakeToShootRadians(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-50, 50))
                                            .build(),
                                    loopAction
                            ));


                    Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
                    Configuration.s_Current.persist("x", mDrive.getPose().position.x);
                    Configuration.s_Current.persist("y", mDrive.getPose().position.y);
                    Configuration.s_Current.persist("alliance", mAlliance.getValue());
                }


                mRobot.loop();
                Thread.sleep(100); // Give the flywheel time to reach back its velocity, now that wheel motors are stopped
                mLogger.metric("STEP", "SHOOT");
                mLogger.update();
                mRobot.shoot();
                mRobot.loop();

                i_step ++;

                mLogger.update();

            }

        }



        mLogger.metric("STEP", "LEAVE" );
        mLogger.update();


        mLogger.update();

        Configuration.s_Current.persist("heading", mDrive.getPose().heading.toDouble());
        Configuration.s_Current.persist("x", mDrive.getPose().position.x);
        Configuration.s_Current.persist("y", mDrive.getPose().position.y);
        Configuration.s_Current.persist("alliance", mAlliance.getValue());

        mLogger.stop();

    }

    @NonNull
    private static StringBuilder getStringBuilder(int current_step, List<AutonomousStep> mSteps) {
        StringBuilder steps = new StringBuilder("[");
        for (int i_step = 0; i_step < mSteps.size(); i_step++) {
            if (i_step == current_step) {
                steps.append(mSteps.get(i_step).text().toUpperCase());
            } else {
                steps.append(mSteps.get(i_step).text().toLowerCase());
            }
            if (i_step < (mSteps.size() - 1)) { steps.append(" , "); }
        }

        steps.append("]");
        return steps;
    }
}