package org.firstinspired.ftc.teamcode.utils;

/* Java includes */
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
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
    public void positionsRedGPP() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.RED);
        checkPatternPosition(mPath.startIntake(Pattern.GPP), Pattern.GPP, Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP), Pattern.GPP, Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP), Pattern.GPP, Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.RED);
        checkLeavePosition(mPath.leave(), Alliance.RED);
    }

    @Test
    public void positionsBlueGPP() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.BLUE);
        checkPatternPosition(mPath.startIntake(Pattern.GPP), Pattern.GPP, Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.GPP), Pattern.GPP, Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.GPP), Pattern.GPP, Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.BLUE);
        checkLeavePosition(mPath.leave(), Alliance.BLUE);
    }

    @Test
    public void positionsRedPGP() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.RED);
        checkPatternPosition(mPath.startIntake(Pattern.PGP), Pattern.PGP, Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP), Pattern.PGP, Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP), Pattern.PGP, Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.RED);
        checkLeavePosition(mPath.leave(), Alliance.RED);
    }

    @Test
    public void positionsBluePGP() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.BLUE);
        checkPatternPosition(mPath.startIntake(Pattern.PGP), Pattern.PGP, Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PGP), Pattern.PGP, Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PGP), Pattern.PGP, Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.BLUE);
        checkLeavePosition(mPath.leave(), Alliance.BLUE);
    }

    @Test
    public void positionsRedPPG() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.RED);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.RED);
        checkPatternPosition(mPath.startIntake(Pattern.PPG), Pattern.PPG, Alliance.RED);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG), Pattern.PPG, Alliance.RED);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG), Pattern.PPG, Alliance.RED);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.RED);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.RED);
        checkLeavePosition(mPath.leave(), Alliance.RED);
    }

    @Test
    public void positionsBluePPG() {

        mPath = new PathAutonomousMiddle(mLogger);
        mPath.initialize(Alliance.BLUE);
        mPath.log();
        mLogger.update();

        checkStartPosition(mPath.start(), Alliance.BLUE);
        checkPatternPosition(mPath.startIntake(Pattern.PPG), Pattern.PPG, Alliance.BLUE);
        checkEndIntakePosition(mPath.endIntake(Pattern.PPG), Pattern.PPG, Alliance.BLUE);
        checkBackIntakePosition(mPath.backIntake(Pattern.PPG), Pattern.PPG, Alliance.BLUE);
        checkFarShootingPosition(mPath.shootingFar(), Alliance.BLUE);
        checkCloseShootingPosition(mPath.shootingClose(), Alliance.BLUE);
        checkLeavePosition(mPath.leave(), Alliance.BLUE);
    }

    private void checkStartPosition(Pose2d position, Alliance alliance) {
        assertEquals(PathAutonomousMiddle.X_START_INCHES, position.position.x, 0.01);
        if (alliance == Alliance.RED) {
            assertEquals(PathAutonomousMiddle.Y_START_INCHES_RED, position.position.y, 0.01);
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(PathAutonomousMiddle.Y_START_INCHES_BLUE, position.position.y, 0.01);
        }
        assertEquals(0, position.heading.toDouble(), 0.01);
    }

    private void checkFarShootingPosition(Pose2d position, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(24, position.position.x, 0.01);
            assertEquals(-24, position.position.y, 0.01);
            assertEquals(-Math.PI / 2, position.heading.toDouble(), 0.01);
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(24, position.position.x, 0.01);
            assertEquals(24, position.position.y, 0.01);
            assertEquals(Math.PI / 2, position.heading.toDouble(), 0.01);
        }
    }

    private void checkCloseShootingPosition(Pose2d position, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(36, position.position.x, 0.01);
            assertEquals(-36, position.position.y, 0.01);
            assertEquals(-Math.PI / 2, position.heading.toDouble(), 0.01);
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(36, position.position.x, 0.01);
            assertEquals(36, position.position.y, 0.01);
            assertEquals(Math.PI / 2, position.heading.toDouble(), 0.01);
        }
    }

    private void checkLeavePosition(Pose2d position, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(PathAutonomousMiddle.X_LEAVE_INCHES, position.position.x, 0.01);
            assertEquals(PathAutonomousMiddle.Y_LEAVE_INCHES_RED, position.position.y, 0.01);
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(PathAutonomousMiddle.X_LEAVE_INCHES, position.position.x, 0.01);
            assertEquals(PathAutonomousMiddle.Y_LEAVE_INCHES_BLUE, position.position.y, 0.01);
        }
    }

    private void checkPatternPosition(Pose2d position, Pattern pattern, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_RED, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_RED, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_RED, position.position.x, 0.01); }
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_BLUE, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_BLUE, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
        }
    }

    private void checkEndIntakePosition(Pose2d position, Pattern pattern, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_RED + PathAutonomousMiddle.Y_DELTA_INTAKE_INCHES_RED, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_RED, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_RED, position.position.x, 0.01); }
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_BLUE + PathAutonomousMiddle.Y_DELTA_INTAKE_INCHES_BLUE, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_BLUE, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
        }
    }

    private void checkBackIntakePosition(Pose2d position, Pattern pattern, Alliance alliance) {
        if (alliance == Alliance.RED) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_RED + 0.4 * PathAutonomousMiddle.Y_DELTA_INTAKE_INCHES_RED, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_RED, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_RED, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_RED, position.position.x, 0.01); }
        }
        if (alliance == Alliance.BLUE) {
            assertEquals(PathAutonomousMiddle.Y_PATTERN_INCHES_BLUE + 0.4 * PathAutonomousMiddle.Y_DELTA_INTAKE_INCHES_BLUE, position.position.y, 0.01);
            assertEquals(PathAutonomousMiddle.ANGLE_PATTERN_RADIANS_BLUE, position.heading.toDouble(), 0.01);
            if (pattern == Pattern.GPP) { assertEquals(PathAutonomousMiddle.X_GPP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PGP) { assertEquals(PathAutonomousMiddle.X_PGP_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
            if (pattern == Pattern.PPG) { assertEquals(PathAutonomousMiddle.X_PPG_PATTERN_INCHES_BLUE, position.position.x, 0.01); }
        }
    }
}