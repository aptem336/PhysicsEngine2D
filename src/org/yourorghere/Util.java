package org.yourorghere;

import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Util {

    public static void sleep(int time) {
        try {
            TimeUnit.MILLISECONDS.sleep(time);
        } catch (InterruptedException ex) {
            Logger.getLogger(Util.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    //<editor-fold defaultstate="collapsed" desc="Задание полей фигур">
    public static Vector[] inscribedInCircle(double R, int countStep) {
        Vector[] offset = new Vector[countStep];
        for (int i = 0; i < countStep; i++) {
            double angle = (double) i / countStep * Math.PI * 2d;
            offset[i] = Vector.angleDefine(R, angle);
        }
        return offset;
    }

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

    public static double area(Vector[] vertexS) {
        double area = 0;
        for (int i = 0; i < vertexS.length; i++) {
            area += (vertexS[(i + 1) % vertexS.length].x - vertexS[i].x) * (vertexS[(i + 1) % vertexS.length].y + vertexS[i].y);
        }
        return Math.abs(area) / 2;
    }
    //</editor-fold>
}
