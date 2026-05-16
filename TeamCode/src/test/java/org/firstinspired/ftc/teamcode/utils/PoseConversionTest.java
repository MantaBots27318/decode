package org.firstinspired.ftc.teamcode.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;

import org.junit.jupiter.api.Test;

import org.firstinspires.ftc.teamcode.utils.PoseConversion;

public class PoseConversionTest {

    private static final double DELTA = 1e-6;

    // -------------------------------------------------------
    // RR -> Pedro
    // -------------------------------------------------------

    @Test
    public void fieldCenterTowardForward() {
        // RR (0,0) heading 0 = center of field, facing forward
        // Pedro: x = 72-0 = 72, y = 0+72 = 72, heading = 0 + π/2
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(0, 0), 0));
        assertEquals(72.0, result.getX(), DELTA);
        assertEquals(72.0, result.getY(), DELTA);
        assertEquals(Math.PI / 2, result.getHeading(), DELTA);
    }

    @Test
    public void rrForwardBecomesHigherY() {
        // Moving forward in RR (x+) should increase Pedro y
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(24, 0), 0));
        assertEquals(72.0, result.getX(), DELTA);
        assertEquals(96.0, result.getY(), DELTA);
    }

    @Test
    public void rrLeftBecomesLowerX() {
        // Moving left in RR (y+) should decrease Pedro x
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(0, 24), 0));
        assertEquals(48.0, result.getX(), DELTA);
        assertEquals(72.0, result.getY(), DELTA);
    }

    @Test
    public void blueStartPosition() {
        // Typical blue alliance start: RR (50, 50, π/9 = 20°)
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(50, 50), Math.PI / 9));
        assertEquals(72 - 50, result.getX(), DELTA);
        assertEquals(50 + 72, result.getY(), DELTA);
        assertEquals(Math.PI / 9 + Math.PI / 2, result.getHeading(), DELTA);
    }

    @Test
    public void redStartPosition() {
        // Typical red alliance start: RR (50, -50, -π/9)
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(50, -50), -Math.PI / 9));
        assertEquals(72 - (-50), result.getX(), DELTA);
        assertEquals(50 + 72, result.getY(), DELTA);
        assertEquals(-Math.PI / 9 + Math.PI / 2, result.getHeading(), DELTA);
    }

    @Test
    public void headingFacingLeft() {
        // RR heading π/2 (facing left) becomes Pedro heading π
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(0, 0), Math.PI / 2));
        assertEquals(Math.PI, result.getHeading(), DELTA);
    }

    @Test
    public void headingFacingRight() {
        // RR heading -π/2 (facing right) becomes Pedro heading 0
        Pose result = PoseConversion.topedroPose(new Pose2d(new Vector2d(0, 0), -Math.PI / 2));
        assertEquals(0.0, result.getHeading(), DELTA);
    }

    // -------------------------------------------------------
    // Pedro -> RR
    // -------------------------------------------------------

    @Test
    public void pedroCenterToRR() {
        // Pedro (72, 72, π/2) = field center facing forward in RR
        Pose2d result = PoseConversion.toRRPose(new Pose(72, 72, Math.PI / 2));
        assertEquals(0.0, result.position.x, DELTA);
        assertEquals(0.0, result.position.y, DELTA);
        assertEquals(0.0, result.heading.toDouble(), DELTA);
    }

    @Test
    public void pedroHighYBecomesRRForward() {
        Pose2d result = PoseConversion.toRRPose(new Pose(72, 96, Math.PI / 2));
        assertEquals(24.0, result.position.x, DELTA);
        assertEquals(0.0, result.position.y, DELTA);
    }

    @Test
    public void pedroLowXBecomesRRLeft() {
        Pose2d result = PoseConversion.toRRPose(new Pose(48, 72, Math.PI / 2));
        assertEquals(0.0, result.position.x, DELTA);
        assertEquals(24.0, result.position.y, DELTA);
    }

    // -------------------------------------------------------
    // Round-trip
    // -------------------------------------------------------

    @Test
    public void roundTripRRToPedroToRR() {
        Pose2d original = new Pose2d(new Vector2d(36, -18), Math.PI / 4);
        Pose2d result = PoseConversion.toRRPose(PoseConversion.topedroPose(original));
        assertEquals(original.position.x, result.position.x, DELTA);
        assertEquals(original.position.y, result.position.y, DELTA);
        assertEquals(original.heading.toDouble(), result.heading.toDouble(), DELTA);
    }

    @Test
    public void roundTripPedroToRRToPedro() {
        Pose original = new Pose(40, 100, Math.PI);
        Pose result = PoseConversion.topedroPose(PoseConversion.toRRPose(original));
        assertEquals(original.getX(), result.getX(), DELTA);
        assertEquals(original.getY(), result.getY(), DELTA);
        assertEquals(original.getHeading(), result.getHeading(), DELTA);
    }
}