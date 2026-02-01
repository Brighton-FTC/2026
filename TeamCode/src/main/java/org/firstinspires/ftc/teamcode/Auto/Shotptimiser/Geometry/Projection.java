package org.firstinspires.ftc.teamcode.Auto.Shotptimiser.Geometry;

public class Projection {
    public double min, max;

    public Projection(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public boolean overlaps(Projection other) {
        return !(max < other.min || other.max < min);
    }
}
