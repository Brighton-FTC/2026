package org.firstinspires.ftc.teamcode.Auto.Shotptimiser.Geometry;

public class Vector {
    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double scalarProduct(Vector other) {
        return x * other.x + y * other.y;
    }

    public double dot(Vector other) {
        return this.x*other.x+this.y*other.y;
    }

    public double cross(Vector other) {
        return this.x*other.x-this.y*other.y;
    }
    public Vector edgeWith(Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public Vector perpendicular() {
        return new Vector(-y, x);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ") ";
    }
}
