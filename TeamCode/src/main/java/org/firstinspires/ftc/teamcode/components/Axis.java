package org.firstinspires.ftc.teamcode.components;

/* System includes */
import java.lang.reflect.Field;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Axis {

    public  static  final   double      sDefaultMaximum = 1.0;
    public  static  final   double      sDefaultDeadzone = 0.0;

    final   Telemetry   mLogger;

    final   Gamepad     mGamepad;
    final   String      mName;

    final   double      mMultiplier;
            double      mDeadZone;
            double      mMaximum;

    /**
     * Axis constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param logger logger
     */
    public Axis(Gamepad gamepad, String name, Telemetry logger) {
        mLogger     = logger;
        mGamepad    = gamepad;
        mName       = name;
        mMultiplier = 1.0;
        mMaximum    = sDefaultMaximum;
        mDeadZone   = sDefaultDeadzone;
    }

    /**
     * Axis constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param multiplier value multiplier
     * @param logger logger
     */
    public Axis(Gamepad gamepad, String name, double multiplier, Telemetry logger) {
        mLogger     = logger;
        mGamepad    = gamepad;
        mName       = name;
        mMultiplier = multiplier;
        mMaximum    = sDefaultMaximum;
        mDeadZone   = sDefaultDeadzone;
    }

    /**
     * Deadzone accessor
     *
     * @param deadzone : value under which trigger value should be considered null
     */
    public  void    deadzone(double deadzone) {
        if (deadzone >= 0 && deadzone <= 1.0) { mDeadZone = deadzone; }
    }

    /**
     * Maximum accessor
     *
     * @param maximum : maximum value of the trigger
     */
    public  void    maximum(double maximum) {
        if (maximum >= 0 && maximum <= 1.0) { mMaximum = maximum; }
    }

    /**
     * Value accessor
     *
     * @return trigger value
     */
    public double   value() {

        double result = 0;

        if(mGamepad != null) {

            try {

                Field field = Gamepad.class.getDeclaredField(mName);
                Object status = field.get(mGamepad);
                if (status != null) {
                    if (field.getType() == double.class) {
                        result = ((double) status * mMultiplier);
                    } else if (field.getType() == float.class) {
                        result = ((float) status * mMultiplier);
                    }
                }

                result = Axis.applyDeadzone(result, mDeadZone, mMaximum);
            }
            catch(NoSuchFieldException | NullPointerException | IllegalAccessException e ) {
                mLogger.addLine("Axis Error : " + e.getMessage());
            }

        }

        return result;
    }

    /**
     * Deadzone scaling function
     *
     * @param value : raw trigger value
     * @param deadzone : deadzone value
     * @param maximum : maximum allowed value
     */
    private static double applyDeadzone(double value, double deadzone, double maximum) {
        if (Math.abs(value) < deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        return ((value - Math.signum(value) * deadzone) / (1.0 - deadzone) * maximum);
    }

}