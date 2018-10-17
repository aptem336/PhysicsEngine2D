package org.yourorghere;

import java.awt.Color;

public class Body {

    public Body(double x, double y, double angle, Vector[] vertex_offset, double density, double elasticity, double friction) {
        this.vertex_offset = vertex_offset;
        vertex = new Vector[vertex_offset.length];
        axis = new Vector[vertex_offset.length];
        AABB = new Vector[2];
        R = Util.R(vertex_offset);

        double mass = Util.area(vertex_offset) * density;
        linear = new Vector(0d, 0d);
        d_linear = new Vector(0d, 0d);
        location = new Vector(x, y);
        iMass = 1d / mass;

        double inertia = mass * R * R / 2;
        angular = 0d;
        d_angular = 0d;
        this.angle = angle;
        iInertia = 1d / inertia;

        centerMass_offset = Util.centerMass(vertex_offset);
        centerMass = new Vector(0d, 0d);

        this.elasticity = elasticity;
        this.friction = friction;

        this.color = Color.white;
        allocateMemory();
        integrateLocation();
    }

    public final void integrateLocation() {
        Vector comp_linear = Vector.getSum(Vector.getProduct(linear, DT), d_linear);
        location.plus(comp_linear);

        double comp_angular = angular * DT + d_angular;
        angle += comp_angular;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        for (int i = 0; i < vertex_offset.length; i++) {
            vertex[i].setRotated(cos, sin, vertex_offset[i], centerMass_offset);
            vertex[i].plus(location);
        }
        centerMass.setSum(centerMass_offset, location);
        calcAABB();
        for (int i = 0; i < axis.length; i++) {
            axis[i].setDiff(vertex[i], vertex[(i + 1) % vertex.length]);
            axis[i].setLeftN();
            axis[i].normalize();
        }
        d_linear.set(Vector.nullVector);
        d_angular = 0d;
    }

    private void allocateMemory() {
        //выделяем память под все переменнные
        for (int i = 0; i < vertex.length; i++) {
            vertex[i] = new Vector(0d, 0d);
        }
        for (int i = 0; i < axis.length; i++) {
            axis[i] = new Vector(0d, 0d);
        }
        for (int i = 0; i < AABB.length; i++) {
            AABB[i] = new Vector(0d, 0d);
        }
    }

    private void calcAABB() {

    }

    private static final double DT = 1d / 60d;
    //геометрия
    private final Vector[] vertex_offset;
    final Vector[] vertex;
    final Vector[] axis;
    //для широкой фазы
    final Vector[] AABB;
    final double R;
    //перемещение
    final Vector linear;
    final Vector d_linear;
    final Vector location;
    final double iMass;
    //вращение
    double angular;
    double d_angular;
    double angle;
    final double iInertia;

    final Vector centerMass_offset;
    final Vector centerMass;
    //параметры
    final double elasticity;
    final double friction;

    Color color;
}
