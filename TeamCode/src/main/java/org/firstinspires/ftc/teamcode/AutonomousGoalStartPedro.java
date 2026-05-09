/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at goal - Pedro Pathing version
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoalPedro;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Pattern;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutonomousGoalStartPedro extends CommandOpMode {

    static final double sIntakePower   = 0.85;
    static final double sGuidingPower  = 0.6;

    Logger                  mLogger;
    Controller              mGamepad1;

    Follower                follower;
    PathAutonomousGoalPedro mPedroPaths;
    PathAutonomousGoal      mRRPaths;

    Turret                  mTurret;
    Transfer                mTransfer;
    Intake                  mIntake;

    Alliance                mAlliance         = Alliance.NONE;
    List<AutonomousStep>    mSteps;
    int                     mCurrentStepIndex = 0;
    boolean                 mFirstRun         = true;

    @Override
    public void initialize() {
        super.reset();

        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);

        mGamepad1   = new Controller(gamepad1, mLogger);
        mPedroPaths = new PathAutonomousGoalPedro(mLogger);
        mRRPaths    = new PathAutonomousGoal(mLogger);
        follower    = Constants.createFollower(hardwareMap);

        mTurret = new Turret();
        mTurret.setHW(Configuration.s_Current, hardwareMap, mLogger, mRRPaths);

        mTransfer = new Transfer();
        mTransfer.setHW(Configuration.s_Current, hardwareMap, mLogger);

        mIntake = new Intake();
        mIntake.setHW(Configuration.s_Current, hardwareMap, mLogger);

        mSteps = new ArrayList<>();
        mSteps.add(AutonomousStep.NONE);
        mCurrentStepIndex = mSteps.size() - 1;

        // Init menu runs every loop tick during init phase
        schedule(new RunCommand(this::runInitMenu));
    }

    private void runInitMenu() {

        if (mGamepad1.buttons.dpad_right.pressedOnce()) {
            mCurrentStepIndex++;
            if (mCurrentStepIndex >= mSteps.size()) { mSteps.add(AutonomousStep.NONE); }
        }
        if (mGamepad1.buttons.dpad_left.pressedOnce()) {
            mCurrentStepIndex--;
            if (mCurrentStepIndex < 0) { mCurrentStepIndex = 0; }
        }
        if (mGamepad1.buttons.dpad_up.pressedOnce()) {
            mSteps.set(mCurrentStepIndex, mSteps.get(mCurrentStepIndex).next());
        }
        if (mGamepad1.buttons.dpad_down.pressedOnce()) {
            mSteps.set(mCurrentStepIndex, mSteps.get(mCurrentStepIndex).previous());
        }
        if (mGamepad1.buttons.y.pressedOnce()) {
            mSteps.clear();
            mCurrentStepIndex = 0;
            mSteps.add(AutonomousStep.NONE);
        }
        if (mGamepad1.buttons.right_trigger.pressedOnce()) {
            mAlliance = Alliance.RED;
            mPedroPaths.initialize(mAlliance);
            mRRPaths.initialize(mAlliance);
        }
        if (mGamepad1.buttons.left_trigger.pressedOnce()) {
            mAlliance = Alliance.BLUE;
            mPedroPaths.initialize(mAlliance);
            mRRPaths.initialize(mAlliance);
        }

        mLogger.info(Logger.Target.DRIVER_STATION, "=========== MENU ============");
        mLogger.info(Logger.Target.DRIVER_STATION, "Choose Alliance : TRIGGER LEFT/RIGHT");
        mLogger.info(Logger.Target.DRIVER_STATION, "Add a step      : DPAD LEFT/RIGHT");
        mLogger.info(Logger.Target.DRIVER_STATION, "Modify step     : DPAD UP/DOWN");
        mLogger.info(Logger.Target.DRIVER_STATION, "Clear all steps : Y");
        mLogger.info(Logger.Target.DRIVER_STATION, "");
        mLogger.info(Logger.Target.DRIVER_STATION, "======= CONFIGURATION =======");
        mLogger.info(Logger.Target.DRIVER_STATION, "==> ALLIANCE : " + mAlliance.name());
        mLogger.info(Logger.Target.DRIVER_STATION, "==> STEPS : " + buildStepsString());
        if (mAlliance != Alliance.NONE) { mPedroPaths.log(); }
        mLogger.update();
    }

    @Override
    public void run() {
        if (mFirstRun) {
            mFirstRun = false;
            super.reset(); // Cancel init menu command

            if (mAlliance == Alliance.NONE) { mAlliance = Alliance.BLUE; }
            mPedroPaths.initialize(mAlliance);
            mRRPaths.initialize(mAlliance);

            follower.setStartingPose(mPedroPaths.start());
            mTurret.setFTCPosition(mRRPaths.start());

            buildAndScheduleAutonomous();
        }
        super.run();
    }

    // -------------------------------------------------------------------------
    // Autonomous sequence builder
    // -------------------------------------------------------------------------

    private void buildAndScheduleAutonomous() {

        SequentialCommandGroup sequence = new SequentialCommandGroup();

        // Start flywheel and intake, drive to shooting position, then shoot
        sequence.addCommands(
                new InstantCommand(mTurret::start),
                new InstantCommand(() -> mIntake.start(sIntakePower, sGuidingPower)),
                new FollowPathCommand(follower, buildPathToShoot(), true, 1.0).setGlobalMaxPower(1.0),
                shootCommand()
        );

        // One segment per non-NONE step
        int i_step = 0;
        for (AutonomousStep step : mSteps) {
            if (step != AutonomousStep.NONE) {
                Pattern  pattern = stepToPattern(step);
                boolean  isLast  = (i_step >= mSteps.size() - 1);
                PathChain path   = isLast ? buildIntakeAndLeave(pattern) : buildIntakeAndReturn(pattern);
                sequence.addCommands(
                        new FollowPathCommand(follower, path, true, 1.0).setGlobalMaxPower(1.0),
                        shootCommand()
                );
                i_step++;
            }
        }

        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> mTurret.loop(0, 0, 0.04, true)),
                sequence
        );
    }

    // -------------------------------------------------------------------------
    // Path builders
    // -------------------------------------------------------------------------

    private PathChain buildPathToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(mPedroPaths.start(), mPedroPaths.shootingFar()))
                .setLinearHeadingInterpolation(mPedroPaths.start().getHeading(), mPedroPaths.shootingFar().getHeading())
                .build();
    }

    private PathChain buildIntakeAndReturn(Pattern pattern) {
        return follower.pathBuilder()
                .addPath(new BezierLine(mPedroPaths.shootingFar(), mPedroPaths.startIntake(pattern)))
                .setLinearHeadingInterpolation(mPedroPaths.shootingFar().getHeading(), mPedroPaths.startIntake(pattern).getHeading())
                .addPath(new BezierLine(mPedroPaths.startIntake(pattern), mPedroPaths.endIntake(pattern)))
                .setLinearHeadingInterpolation(mPedroPaths.startIntake(pattern).getHeading(), mPedroPaths.endIntake(pattern).getHeading())
                .addPath(new BezierLine(mPedroPaths.endIntake(pattern), mPedroPaths.shootingFar()))
                .setLinearHeadingInterpolation(mPedroPaths.endIntake(pattern).getHeading(), mPedroPaths.shootingFar().getHeading())
                .build();
    }

    private PathChain buildIntakeAndLeave(Pattern pattern) {
        return follower.pathBuilder()
                .addPath(new BezierLine(mPedroPaths.shootingFar(), mPedroPaths.startIntake(pattern)))
                .setLinearHeadingInterpolation(mPedroPaths.shootingFar().getHeading(), mPedroPaths.startIntake(pattern).getHeading())
                .addPath(new BezierLine(mPedroPaths.startIntake(pattern), mPedroPaths.endIntake(pattern)))
                .setLinearHeadingInterpolation(mPedroPaths.startIntake(pattern).getHeading(), mPedroPaths.endIntake(pattern).getHeading())
                .addPath(new BezierLine(mPedroPaths.endIntake(pattern), mPedroPaths.leave()))
                .setLinearHeadingInterpolation(mPedroPaths.endIntake(pattern).getHeading(), mPedroPaths.leave().getHeading())
                .build();
    }

    // -------------------------------------------------------------------------
    // Command and step helpers
    // -------------------------------------------------------------------------

    private SequentialCommandGroup shootCommand() {
        return new SequentialCommandGroup(
                new WaitCommand(2500),
                new FunctionalCommand(() -> {}, mTransfer::open_and_close_loop, interrupted -> {}, () -> !mTransfer.ongoing())
        );
    }

    private Pattern stepToPattern(AutonomousStep step) {
        if (step == AutonomousStep.GPP) { return Pattern.GPP; }
        if (step == AutonomousStep.PGP) { return Pattern.PGP; }
        return Pattern.PPG;
    }

    private String buildStepsString() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < mSteps.size(); i++) {
            sb.append(i == mCurrentStepIndex
                    ? mSteps.get(i).text().toUpperCase()
                    : mSteps.get(i).text().toLowerCase());
            if (i < mSteps.size() - 1) { sb.append(" , "); }
        }
        sb.append("]");
        return sb.toString();
    }
}