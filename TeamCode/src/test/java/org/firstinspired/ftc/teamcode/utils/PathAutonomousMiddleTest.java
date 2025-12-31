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
import org.firstinspires.ftc.teamcode.pose.PathAutonomousMiddle;


@ExtendWith(MockitoExtension.class)
public class PathAutonomousMiddleTest {

    private PathAutonomousMiddle mPath;
    private Logger               mLogger;


    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new Logger(null, null, "path-autonomous-middle-test",2);
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

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.GPP,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }

    @Test
    public void positionsRedGPPGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.GPP,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }


    @Test
    public void positionsBlueGPPLaunch() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.GPP,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);

    }

    @Test
    public void positionsBlueGPPGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.GPP,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.GPP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);

    }


    @Test
    public void positionsRedPGPLaunch() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PGP,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }

    @Test
    public void positionsRedPGPGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PGP,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }


    @Test
    public void positionsBluePGPLaunch() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PGP,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);


    }

    @Test
    public void positionsBluePGPGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PGP,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PGP,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);


    }


    @Test
    public void positionsRedPPGLaunch() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PPG,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),-Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }

    @Test
    public void positionsRedPPGGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED, Vision.Pattern.PPG,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.RED);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.RED);
        checkCalibrationPosition(mPath.calibration(),Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.parking(),Alliance.RED);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),Math.PI / 2,0.01);

    }


    @Test
    public void positionsBluePPGLaunch() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PPG,true);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI / 2);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);

    }

    @Test
    public void positionsBluePPGGate() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE, Vision.Pattern.PPG,false);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(),Alliance.BLUE);
        checkPatternPosition(mPath.pattern(),Vision.Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(),Vision.Pattern.PPG,Alliance.BLUE);
        checkCalibrationPosition(mPath.calibration(),Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.parking(),Alliance.BLUE);

        assertEquals(mPath.hAutoToTeleopRadians(),Math.PI);
        assertEquals(mPath.tgtIntakeToCalibrationRadians(),-Math.PI / 2,0.01);

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
            assertEquals(position.position.x,11, 0.01);
            assertEquals(position.position.y,- 20,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 4,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,11,0.01);
            assertEquals(position.position.y,20,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 4,0.01);
        }
    }

    private void checkStartPosition(Pose2d position, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.x,-63,0.01);
            assertEquals(position.position.y,-10,0.01);
            assertEquals(position.heading.toDouble(),0,0.01);
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.x,-63,0.01);
            assertEquals(position.position.y,10,0.01);
            assertEquals(position.heading.toDouble(),0,0.01);
        }

    }

    private void checkPatternPosition(Pose2d position, Vision.Pattern pattern, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-10,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-33,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-11,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,12,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,10,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-38,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-13,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,9,0.01);
            }
        }
    }

    private void checkEndIntakePosition(Pose2d position, Vision.Pattern pattern, Alliance alliance)
    {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-50,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-33,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-11,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,12,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,50,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-38,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-13,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,9,0.01);
            }
        }

    }
    private void checkBackIntakePosition(Pose2d position, Vision.Pattern pattern, Alliance alliance) {

        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-22,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-33,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-11,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,12,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,22,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Vision.Pattern.GPP) {
                assertEquals(position.position.x,-38,0.01);
            }
            if(pattern == Vision.Pattern.PGP) {
                assertEquals(position.position.x,-13,0.01);
            }
            if(pattern == Vision.Pattern.PPG) {
                assertEquals(position.position.x,9,0.01);
            }
        }

    }

}
