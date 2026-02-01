package org.firstinspires.ftc.teamcode.Auto.Shotptimiser.Geometry;

import android.media.MediaCodec;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

class Polygon {

    private List<Vector> vertices;
    private List<LineSegment> lines;
    private final Point centroid;

    public Polygon(List<Point> points) {
        vertices = points.stream()
                .map(point -> new Vector(point.x, point.y))
                .collect(Collectors.toCollection(ArrayList::new));

        double xSum = 0;
        double ySum = 0;

        for (int i = 0; i<points.size(); i++) {
            lines.add(new LineSegment(points.get(i), points.get((i+1)%points.size())));
        }

        for (int i = 0; i<vertices.size(); i++) {
            xSum += vertices.get(i).x;
            ySum += vertices.get(i).y;

        }

        centroid = new Point(xSum/points.size(), ySum/points.size());
    }

    public boolean overlaps(Polygon other) {
        Ray centroidRay = new Ray(centroid, new Vector(1, 0));
        int intersects = 0;
        for (int i = 0; i<lines.size(); i++) {
            if (lines.get(i).rayIntersect(centroidRay)) { intersects++; }
        }

        if (intersects % 2 == 1) { return true; }

        

        return false;
    }


    public String toString() {
        StringBuilder result = new StringBuilder("[ ");
        for ( Vector vertex : vertices ) {
            result.append(vertex);
        }

        result.append("]");
        return result.toString();
    }


}
