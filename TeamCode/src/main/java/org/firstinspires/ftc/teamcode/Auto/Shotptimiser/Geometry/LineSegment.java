package org.firstinspires.ftc.teamcode.Auto.Shotptimiser.Geometry;

import java.util.Optional;

public class LineSegment {
    public Point a, b;

    public LineSegment(Point a, Point b) {
        this.a = a;
        this.b = b;
    }

    public Boolean rayIntersect(Ray ray) {
        Vector v1 = new Vector(ray.origin.x - this.a.x, ray.origin.y - this.a.y);
        Vector v2 = new Vector(this.b.x - this.a.x, this.b.y - this.a.y);
        Vector v3 = new Vector(-ray.direction.y, ray.direction.x);


        double dot = v2.dot(v3);
        if (Math.abs(dot) < 0.000001)
            return false;

        double t1 = v2.cross(v1) / dot;
        double t2 = v1.dot(v3) / dot;

        if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
            return true;

        return false;
    }

    public Boolean segmentIntersect(LineSegment segment) {

    }

}
