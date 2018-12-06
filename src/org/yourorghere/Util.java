package org.yourorghere;

public class Util {

    public static Vector[] inscribedInEllipse(double angle, double a, double b, int countStep) {
        Vector[] offset = new Vector[countStep];
        double angleInc = Math.PI * 2 / (double) countStep;
        for (int i = 0; i < countStep; i++) {
            offset[i] = new Vector(a * Math.cos(angle), b * Math.sin(angle));
            angle += angleInc;
        }
        return offset;
    }

    public static Vector[] inscribedInCircle(double angle, double R, int countStep) {
        return inscribedInEllipse(angle, R, R, countStep);
    }

    //радиус описывающей окружности
    public static double R(Vector[] vertexS) {
        double R = 0;
        for (Vector vertex : vertexS) {
            double tempR = vertex.len();
            R = Math.max(R, tempR);
        }
        return R;
    }

    public static Vector centerMass(Vector[] vertexS) {
        double xSum = 0;
        double ySum = 0;
        for (Vector vertex : vertexS) {
            xSum += vertex.x;
            ySum += vertex.y;
        }
        return new Vector(xSum / vertexS.length, ySum / vertexS.length);
    }

    //площадь
    public static double area(Vector[] vertexS) {
        double area = 0;
        for (int i = 0; i < vertexS.length; i++) {
            area += (vertexS[(i + 1) % vertexS.length].x - vertexS[i].x) * (vertexS[(i + 1) % vertexS.length].y + vertexS[i].y);
        }
        return Math.abs(area) / 2;
    }
}
