package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Vector;

public class Vector2D extends Vector {
    public Vector2D(double x, double y) {
        super(x,y);
    }

    public double distSquared(Vector2D other) {
        Vector offset = this.minus(other);
        return offset.dot(offset);
    }

    public double magSquared() {
        return this.dot(this);
    }

    public double getX() {
        return get(0);
    }

    public double getY() {
        return get(1);
    }

    public Vector rotate(double theta) {
        return rotate(theta);
    }
}

