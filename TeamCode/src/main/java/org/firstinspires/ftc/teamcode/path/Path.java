package org.firstinspires.ftc.teamcode.path;

/* ACME ROBOTICS include */
import com.acmerobotics.roadrunner.Pose2d;

/* Project includes */
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.configurations.Alliance;

public class Path {

    public static final double FIELD_SIZE_INCHES                                 = 12 * 12;
    public static final double M_TO_INCHES                                       = 39.37;

    protected static final double Y_DELTA_INTAKE_INCHES_BLUE                     = 40;
    protected static final double Y_DELTA_INTAKE_INCHES_RED                      = -40;

    private static final double X_SHOOTING_CLOSE_INCHES                          = 36;
    private static final double Y_SHOOTING_CLOSE_INCHES_BLUE                     = 36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_BLUE                = Math.PI / 4;
    private static final double Y_SHOOTING_CLOSE_INCHES_RED                      = -36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_RED                 = - Math.PI / 4;

    private static final double X_SHOOTING_FAR_INCHES                            = 16;
    private static final double Y_SHOOTING_FAR_INCHES_BLUE                       = 16;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_BLUE                  = Math.PI / 4;
    private static final double Y_SHOOTING_FAR_INCHES_RED                        = -16;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_RED                   = - Math.PI / 4;

    private static final double X_PARKING_GATE_ZONE_INCHES                       = 32;
    private static final double Y_PARKING_GATE_ZONE_INCHES_BLUE                  = 53;
    private static final double Y_PARKING_GATE_ZONE_INCHES_RED                   = -53;
    private static final double ANGLE_PARKING_GATE_ZONE_RADIANS_RED              = Math.PI/2;
    private static final double ANGLE_PARKING_GATE_ZONE_RADIANS_BLUE             = -Math.PI/2;

    private static final double X_PARKING_LAUNCH_ZONE_INCHES                     = 60;
    private static final double Y_PARKING_LAUNCH_ZONE_INCHES_BLUE                = 30;
    private static final double Y_PARKING_LAUNCH_ZONE_INCHES_RED                 = -30;
    private static final double ANGLE_PARKING_LAUNCH_ZONE_RADIANS                = -Math.PI;

    private static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED     = - Math.PI/2;
    private static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE    = Math.PI/2;
    private static final double ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_RED       = Math.PI;
    private static final double ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_BLUE      = Math.PI;

    Logger          mLogger;

    double          mYDeltaIntakeInches         = 0;

    Pose2d          mShootingClose              = new Pose2d(0,0,0);
    Pose2d          mShootingFar                = new Pose2d(0,0,0);
    Pose2d          mParking                    = new Pose2d(0,0,0);

    double          mAngleAutoToTeleopRadians   = 0;
    double          mFieldCentric2FTC = 0;

    public Path(Logger logger) {
        mLogger = logger;
    }

    public void initialize(Alliance alliance, boolean ShallParkInLaunchZone) {

        if (alliance == Alliance.RED) {

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_RED;

            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_RED,ANGLE_SHOOTING_FAR_RADIANS_RED);
            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_RED,ANGLE_SHOOTING_CLOSE_RADIANS_RED);

            if(ShallParkInLaunchZone) {

                mParking = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_RED,ANGLE_PARKING_LAUNCH_ZONE_RADIANS);
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED;

            }
            else {

                mParking = new Pose2d(X_PARKING_GATE_ZONE_INCHES, Y_PARKING_GATE_ZONE_INCHES_RED,ANGLE_PARKING_GATE_ZONE_RADIANS_RED);
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_RED;

            }

            mFieldCentric2FTC = - Math.PI /2;

        }

        if (alliance == Alliance.BLUE) {

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_BLUE;

            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_BLUE,ANGLE_SHOOTING_CLOSE_RADIANS_BLUE);
            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_BLUE,ANGLE_SHOOTING_FAR_RADIANS_BLUE);

            if(ShallParkInLaunchZone) {

                mParking = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_BLUE,ANGLE_PARKING_LAUNCH_ZONE_RADIANS);
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE;

            }
            else {

                mParking = new Pose2d(X_PARKING_GATE_ZONE_INCHES, Y_PARKING_GATE_ZONE_INCHES_BLUE,ANGLE_PARKING_GATE_ZONE_RADIANS_BLUE);
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_BLUE;

            }
            mFieldCentric2FTC = Math.PI /2;

        }
    }

    public Pose2d   shootingClose()         { return mShootingClose; }
    public Pose2d   shootingFar()           { return mShootingFar; }
    public Pose2d   parking()               { return mParking;}
    public double   fieldCentric2FTC()      { return mFieldCentric2FTC; }

    public double   hAutoToTeleopRadians()  { return mAngleAutoToTeleopRadians; }

    public void log() {
        mLogger.info("SHOOTING FAR: " + mShootingFar.position.x + " Y: " + mShootingFar.position.y + " H: " + mShootingFar.heading.toDouble());
        mLogger.info("SHOOTING CLOSE: " + mShootingClose.position.x + " Y: " + mShootingClose.position.y + " H: " + mShootingClose.heading.toDouble());
        mLogger.info("LEAVE: " + mParking.position.x + " Y: " + mParking.position.y + " H: " + mParking.heading.toDouble());
        mLogger.info("AUTO TO TELEOP: " + mAngleAutoToTeleopRadians);
        mLogger.info("INTAKE DELTA Y : " + mYDeltaIntakeInches);
    }

}