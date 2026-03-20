package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

public class Ball {
    final Point matPoint;
    final String color;

    public Ball(String color, Point matPoint) {
        this.color = color;
        this.matPoint = matPoint;
    }

    public Point position() { return matPoint;}
    public String color() { return color;}

}

