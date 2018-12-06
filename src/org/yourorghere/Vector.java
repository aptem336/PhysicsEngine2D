package org.yourorghere;

public class Vector {

    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double len() {
        return Math.sqrt(squareLen());
    }

    public double squareLen() {
        return x * x + y * y;
    }

    public void normalize() {
        double len = len();
        x /= len;
        y /= len;
    }

    public void set(double newX, double newY) {
        x = newX;
        y = newY;
    }

    public void set(Vector set) {
        x = set.x;
        y = set.y;
    }

    public void add(Vector v) {
        x += v.x;
        y += v.y;
    }

    public void add(Vector v, double d) {
        x += v.x * d;
        y += v.y * d;
    }

    public void setRotated(double cos, double sin, Vector point, Vector center) {
        x = center.x + (point.x - center.x) * cos + (point.y - center.y) * sin;
        y = center.x - (point.x - center.x) * sin + (point.y - center.y) * cos;
    }

    public void setRightN() {
        set(-y, x);
    }

    public void setLeftN() {
        set(y, -x);
    }

    public void setDiff(Vector v1, Vector v2) {
        x = v2.x - v1.x;
        y = v2.y - v1.y;
    }

    public void setSum(Vector v1, Vector v2) {
        x = v1.x + v2.x;
        y = v1.y + v2.y;
    }

    public void setProduct(Vector v1, double val) {
        x = v1.x * val;
        y = v1.y * val;
    }

    public Vector copy() {
        return new Vector(x, y);
    }

    public Vector getInvert() {
        return new Vector(-x, -y);
    }

    public Vector getNormalized() {
        double len = len();
        return new Vector(x / len, y / len);
    }

    public Vector getRightN() {
        return new Vector(-y, x);
    }

    public Vector getLeftN() {
        return new Vector(y, -x);
    }

    public static Vector getDiff(Vector v1, Vector v2) {
        return new Vector(v2.x - v1.x, v2.y - v1.y);
    }

    public static Vector getSum(Vector v1, Vector v2) {
        return new Vector(v1.x + v2.x, v1.y + v2.y);
    }

    public static Vector getProduct(Vector v1, double val) {
        return new Vector(v1.x * val, v1.y * val);
    }

    public static double dotProduct(Vector v1, Vector v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    public static double crossProduct(Vector v1, Vector v2) {
        return v1.x * v2.y - v2.x * v1.y;
    }

    public boolean eq(Vector v) {
        return v.x == x && v.y == y;
    }

    public static final Vector GRAVITY = new Vector(0d, -9.8d);
    public static final Vector NULL = new Vector(0d, 0d);
    public static final Vector X = new Vector(1d, 0d);
    public static final Vector Y = new Vector(0d, 1d);
}
