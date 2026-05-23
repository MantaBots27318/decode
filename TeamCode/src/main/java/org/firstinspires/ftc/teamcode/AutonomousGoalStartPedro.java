/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at goal - Pedro Pathing version
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class AutonomousGoalStartPedro extends CommandOpMode {

    static final double sIntakePower  = 0.85;
    static final double sGuidingPower = 0.6;

    Logger             mLogger;
    Follower           follower;
    PathAutonomousGoal mRRPaths;

    Turret   mTurret;
    Transfer mTransfer;
    Intake   mIntake;

    boolean mFirstRun = true;

    @Override
    public void initialize() {
        super.reset();

        mLogger  = new Logger(telemetry, "autonomous-goal-start");
        mLogger.level(Logger.Severity.INFO);

        mRRPaths = new PathAutonomousGoal(mLogger);
        mRRPaths.initialize(Alliance.BLUE);
        follower = Constants.createFollower(hardwareMap);

        mTurret = new Turret();
        mTurret.setHW(Configuration.s_Current, hardwareMap, mLogger, mRRPaths);

        mTransfer = new Transfer();
        mTransfer.setHW(Configuration.s_Current, hardwareMap, mLogger);

        mIntake = new Intake();
        mIntake.setHW(Configuration.s_Current, hardwareMap, mLogger);
    }

    @Override
    public void run() {
        if (mFirstRun) {
            mFirstRun = false;
            super.reset();

            follower.setStartingPose(new Pose(22.000, 122.000, Math.toRadians(140)));
            mTurret.setFTCPosition(mRRPaths.start());

            schedule(
                    new RunCommand(() -> follower.update()),
                    new RunCommand(() -> mTurret.loop(0, 0, 0.04, true)),
                    new RunCommand(() -> mTransfer.periodic()),
                    new SequentialCommandGroup(
                            // new InstantCommand(mTurret::start),
                            // new WaitCommand(200),
                            new InstantCommand(() -> mIntake.start(sIntakePower, sGuidingPower)),
                            new FollowPathCommand(follower, buildPathToShoot(), true, 1.0).setGlobalMaxPower(1.0),
                            // shootCommand(),
                            new FollowPathCommand(follower, buildTravelToIntake(), true, 1.0).setGlobalMaxPower(1.0)
                    )
            );
        }
        super.run();
    }

    private PathChain buildPathToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.000, 122.000),
                        new Pose(48.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();
    }

    private PathChain buildTravelToIntake() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 96.000),
                                new Pose(41.675, 88.257),
                                new Pose(19.617, 88.863)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    // private SequentialCommandGroup shootCommand() {
    //     return new SequentialCommandGroup(
    //             new FunctionalCommand(
    //                     () -> { mTransfer.open_and_close_loop(); mLogger.info("here"); mLogger.update(); },
    //                     () -> {},
    //                     interrupted -> {},
    //                     () -> !mTransfer.ongoing()
    //             )
    //     );
    // }
}
