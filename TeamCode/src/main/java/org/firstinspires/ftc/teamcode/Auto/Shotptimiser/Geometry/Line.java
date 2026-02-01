package org.firstinspires.ftc.teamcode.Auto.Shotptimiser.Geometry;

public class Line {
    double m, c;

    public Line(double m, double c) {
        this.m = m;
        this.c = c;
    }

    public Line(LineSegment segment) {
        this.m = (segment.b.y - segment.a.y)/(segment.a.x - segment.b.y);
        this.c = segment.a.y - m*segment.a.x;
    }


    public Point intersect(Line other) {
        double x = (other.c-this.c)/(this.m-other.m);
        return new Point(x, m*x+c);
    }
}
