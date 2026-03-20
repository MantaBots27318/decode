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
import org.firstinspires.ftc.teamcode.vision.Pattern;

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
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedGPPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP),Pattern.GPP,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBlueGPPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        checkPatternPosition(mPath.startIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBlueGPPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);


        checkPatternPosition(mPath.startIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP),Pattern.GPP,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPGPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPGPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP),Pattern.PGP,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePGPLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        checkPatternPosition(mPath.startIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePGPGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        checkPatternPosition(mPath.startIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP),Pattern.PGP,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPPGLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkLaunchPosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsRedPPGGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.RED);

        checkPatternPosition(mPath.startIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG),Pattern.PPG,Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.RED);
        checkGatePosition(mPath.leave(),Alliance.RED);

        assertEquals(mPath.tgtIntakeToShootRadians(),Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePPGLaunch() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        checkPatternPosition(mPath.startIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkLaunchPosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

        mLogger.update();
    }

    @Test
    public void positionsBluePPGGate() {

        mPath = new PathAutonomousGoal(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();

        checkStartPosition(mPath.start(),Alliance.BLUE);

        checkPatternPosition(mPath.startIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG),Pattern.PPG,Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(),Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(),Alliance.BLUE);
        checkGatePosition(mPath.leave(),Alliance.BLUE);

        assertEquals(mPath.tgtIntakeToShootRadians(),-Math.PI / 2,0.01);

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


    private void checkPatternPosition(Pose2d position, Pattern pattern, Alliance alliance) {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-36,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,36,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }
    }

    private void checkEndIntakePosition(Pose2d position, Pattern pattern, Alliance alliance)
    {
        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-76,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,76,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }

    }
    private void checkBackIntakePosition(Pose2d position, Pattern pattern, Alliance alliance) {

        if(alliance == Alliance.RED) {
            assertEquals(position.position.y,-52,0.01);
            assertEquals(position.heading.toDouble(),- Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-25,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-6,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,20,0.01);
            }
        }
        if(alliance == Alliance.BLUE) {
            assertEquals(position.position.y,52,0.01);
            assertEquals(position.heading.toDouble(),Math.PI / 2,0.01);

            if(pattern == Pattern.GPP) {
                assertEquals(position.position.x,-30,0.01);
            }
            if(pattern == Pattern.PGP) {
                assertEquals(position.position.x,-5,0.01);
            }
            if(pattern == Pattern.PPG) {
                assertEquals(position.position.x,17,0.01);
            }
        }

    }


}
