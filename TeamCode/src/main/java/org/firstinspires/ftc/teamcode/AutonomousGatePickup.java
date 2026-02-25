/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Autonomous starting at goal
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;


/* Java includes */
import java.util.List;

/* Android includes */
import androidx.annotation.NonNull;

/* Acme robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* Local includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGatePickup;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;


@Autonomous
public class AutonomousGatePickup extends LinearOpMode {

    static final int            sAttempts = 3;

    Logger                      mLogger;
    SmartTimer                  mTimer;

    Controller                  mGamepad1;
    MecanumDrive                mDrive;
    Robot                       mRobot;

    Alliance                    mAlliance = Alliance.NONE;
    PathAutonomousGatePickup    mPath;


    @Override
    public void runOpMode() throws InterruptedException {

        mLogger = new Logger(telemetry, FtcDashboard.getInstance(), "autonomous-gate-pickup");
        mLogger.level(Logger.Severity.INFO);
        mTimer = new SmartTimer(mLogger);

        mGamepad1 = new Controller(gamepad1, mLogger);

        mPath = new PathAutonomousGatePickup(mLogger);

        mDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current, hardwareMap, mLogger, mGamepad1, null, mPath);

        while (opModeInInit()) {

            if (mGamepad1.buttons.right_trigger.pressedOnce()) {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance);
            }
            if (mGamepad1.buttons.left_trigger.pressedOnce()) {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance);
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance : TRIGGER LEFT/RIGHT");
            mLogger.info("");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.info("==> ALLIANCE : " + mAlliance.name());

            mPath.log();
            mLogger.update();

        }

        Pose2d start = mPath.start();
        Pose2d shoot = mPath.shootingFar();
        mRobot.initialize(start, Robot.Mode.AUTONOMOUS);
        mDrive = new MecanumDrive(hardwareMap, start);

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
                                .afterTime(0.1, engageAction)
                                .lineToXConstantHeading(shoot.position.x + 3, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50, 50))
                                .build(),
                        loopAction
                ));

        mLogger.metric("STEP", "SHOOT");
        mLogger.update();

        mRobot.shoot();
        mRobot.loop();

        mLogger.metric("STEP", "GO TO PPG AND BACK" );
        mLogger.update();

        Pose2d start_intake = mPath.startIntake(Pattern.PPG);
        Pose2d end_intake = mPath.endIntake(Pattern.PPG);
        Pose2d back_intake = mPath.backIntake(Pattern.PPG);

        Actions.runBlocking(
                new RaceAction(
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
                                .build(),
                        loopAction
                ));

        mLogger.metric("STEP", "SHOOT" );
        mLogger.update();

        mRobot.shoot();
        mRobot.loop();

        for (int i_attempt = 0; i_attempt < sAttempts; i_attempt ++) {

            mLogger.metric("STEP", "GO GATE TO AND BACK" );
            mLogger.update();

            Pose2d start_gate = mPath.startGate();
            Pose2d end_gate = mPath.endGate();
            Pose2d back_gate = mPath.backGate();

            Actions.runBlocking(
                    new RaceAction(
                            mDrive.actionBuilder(mDrive.getPose())
                                    .afterTime(0.01, startStopIntakeAction)
                                    .setTangent(-Math.PI)
                                    .splineToLinearHeading(start_gate, start_gate.heading.toDouble(), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15, 15))
                                    .setTangent(start_gate.heading.toDouble())
                                    .splineToLinearHeading(end_gate, end_gate.heading.toDouble(), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 15))
                                    .afterTime(2, startStopIntakeAction)
                                    .setTangent(-end_gate.heading.toDouble())
                                    .splineToLinearHeading(back_gate, -back_gate.heading.toDouble(), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                                    .afterTime(0.01, engageAction)
                                    .setTangent(mPath.tgtIntakeToShootRadians())
                                    .splineToLinearHeading(shoot, 0, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-25, 50))
                                    .build(),
                            loopAction
                    ));

            mLogger.metric("STEP", "SHOOT" );
            mLogger.update();

            mRobot.shoot();
            mRobot.loop();
        }


        Pose2d leave = mPath.leave();

        mLogger.metric("STEP", "LEAVE" );
        mLogger.update();

        Actions.runBlocking(
            new RaceAction(
                mDrive.actionBuilder(shoot)
                    .setTangent(shoot.heading.toDouble() + Math.PI)
                    .splineToLinearHeading(leave, leave.heading.toDouble() + Math.PI, new TranslationalVelConstraint(200), new ProfileAccelConstraint(-100, 100))
                    .build(),
                loopAction
            ));

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