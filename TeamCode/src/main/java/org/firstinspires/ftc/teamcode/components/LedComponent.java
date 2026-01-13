/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   LedComponent is the abstract interface managing leds,
   mocked leds and coupled leds with the same methods
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public abstract class LedComponent {
    protected Color         mCurrentColor = Color.RED;

    protected SmartTimer    mTimer = null;

    protected boolean       mBlinkingOn = false;

    protected boolean       mIsBlinking = false;

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

    public LedComponent(Logger logger) {
        mTimer = new SmartTimer(logger);
    }
    public LedComponent() {}
    
    public abstract boolean isReady();
    public abstract void    on(Color color);
    public void             on() { on(mCurrentColor); }
    public abstract void    off();
    public void             blink(){
       mIsBlinking = true;
    }
    public void             unblink(){
        mIsBlinking = false;
    }
}