package org.firstinspired.ftc.teamcode.utils;

/* Java includes */
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.acmerobotics.roadrunner.Pose2d;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Project includes */
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.vision.Vision;

/* Component Under Test */
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;


@ExtendWith(MockitoExtension.class)
public class PathAutonomousGoalTest {


    private PathAutonomousGoal   mPath;
    private Logger               mLogger;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new Logger(null, null, "path-autonomous-goal-test",2);
            mLogger.level(Logger.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }


    @AfterEach
    public void tearDown() {
        mLogger.stop();
    }

    @Test
    public void positionsRedGPPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.GPP,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedGPPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.GPP,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBlueGPPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.GPP,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBlueGPPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.GPP,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPGPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PGP,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPGPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PGP,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePGPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PGP,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePGPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PGP,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPPGLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PPG,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPPGGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PPG,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePPGLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PPG,true);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);

        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePPGGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PPG,false);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        assertEquals(mPath.xCalibrationFromGoal(),-38.18376618407357,0.01);

        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);

        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);
        assertEquals(mPath.hObeliskFTCRadians(),-Math.PI / 3,0.01);

        mLogger.update();
    }

    private void checkFarShootingPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,16,0.01);
            assertEquals(position.position.y,-16,0.01);
            assertEquals(position.heading.toDouble(),-Math.PI / 4,0.01);

        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,16,0.01);
            assertEquals(position.position.y,16,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 4,0.01);
        }
    }

    private void checkCloseShootingPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,36,0.01);
            assertEquals(position.position.y,-36,0.01);
            assertEquals(position.heading.toDouble(),-Math.PI / 4,0.01);

        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,36,0.01);
            assertEquals(position.position.y,36,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 4,0.01);
        }
    }

    private void checkLaunchPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,60,0.01);
            assertEquals(position.position.y,-30,0.01);
            assertEquals(position.heading.toDouble(),-Math.PI,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,60,0.01);
            assertEquals(position.position.y,30,0.01);
            assertEquals(position.heading.toDouble(),-Math.PI,0.01);
        }
    }

    private void checkGatePosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,32,0.01);
            assertEquals(position.position.y,-53,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,32,0.01);
            assertEquals(position.position.y,53,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);
        }
    }

    private void checkCalibrationPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,11,0.01);
            assertEquals(position.position.y,-20,0.01);
            assertEquals(position.heading.toDouble(),-Math.PI / 4,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,11,0.01);
            assertEquals(position.position.y,20,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 4,0.01);
        }
    }

    private void checkStartPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,53.082632729367745,0.01);
            assertEquals(position.position.y,-48.36134705062547,0.01);
            assertEquals(position.heading.toDouble(),-0.6283185307179586,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,53.082632729367745,0.01);
            assertEquals(position.position.y,48.36134705062547,0.01);
            assertEquals(position.heading.toDouble(),0.6283185307179586,0.01);
        }

    }


    private void checkPatternPosition(Pose2d position, Vision.Pattern pattern, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-36,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,36,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }
    }

    private void checkEndIntakePosition(Pose2d position, Vision.Pattern pattern, Alliance alliance)
    {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-76,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,76,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }

    }
    private void checkBackIntakePosition(Pose2d position, Vision.Pattern pattern, Alliance alliance) {

        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-52,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,52,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }

    }


}
