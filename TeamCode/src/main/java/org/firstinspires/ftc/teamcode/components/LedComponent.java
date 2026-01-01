/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   LedComponent is the abstract interface managing leds,
   mocked leds and coupled leds with the same methods
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

public abstract class LedComponent {

    public enum Color {
        RED("RED"),
        GREEN("GREEN"),
        NONE("NONE");

        private final String mText;

        Color(String text) {
            mText = text;
        };

        public String text()       { return mText;       }
    }
    
    public abstract boolean isReady();
    public abstract void on(Color color);
    public abstract void off();
}