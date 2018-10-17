package org.yourorghere;

import java.awt.Color;

public class ContactJoint {

    public void preStep() {
        for (int i = 0; i < contactPointCount; i++) {
            applyImpulse(acummLambda[i], i);
        }
    }

    public void solveImpulse() {
        for (int i = 0; i < contactPointCount; i++) {
            double dLambda = calcLambda(a.linear, b.linear, a.angular, b.angular, dstVelocity[i], i) / contactPointCount;
            if (dLambda + acummLambda[i] < 0d) {
                dLambda = -acummLambda[i];
            }
            applyImpulse(dLambda, i);
            acummLambda[i] += dLambda;
        }
    }

    public void solvePenetration() {
        for (int i = 0; i < contactPointCount; i++) {
            double dLambda = calcLambda(a.d_linear, b.d_linear, a.d_angular, b.d_angular, displacement_dstVelocity, i) / contactPointCount;
            if (dLambda + displasement_acummLambda[i] < 0d) {
                dLambda = -displasement_acummLambda[i];
            }
            applyDisplacementImpulse(dLambda, i);
            displasement_acummLambda[i] += dLambda;
        }

    }

    private double calcLambda(Vector linearA, Vector linearB, double aangleA, double angleB, double distanseVelocity, int num) {
        double lambda = 0;
        lambda -= Vector.dotProduct(normal, linearA);
        lambda += Vector.dotProduct(normal, linearB);
        lambda -= raCrossN[num] * aangleA;
        lambda -= rbCrossN[num] * angleB;
        lambda -= distanseVelocity;
        return lambda * invSumInvMass[num];
    }

    private void applyImpulse(double lambda, int num) {
        a.linear.plus(linearProjector1, lambda);
        b.linear.plus(lineatProjector2, lambda);
        a.angular += lambda * angularProjector1[num];
        b.angular += lambda * angularProjector2[num];
    }

    private void applyDisplacementImpulse(double d_lambda, int num) {
        a.d_linear.plus(linearProjector1, d_lambda);
        b.d_linear.plus(lineatProjector2, d_lambda);
        a.d_angular += d_lambda * angularProjector1[num];
        b.d_angular += d_lambda * angularProjector2[num];
    }

    public boolean refresh() {
        if (recalcCollis()) {
            linearProjector1.set(Vector.getProduct(normal, a.iMass));
            lineatProjector2.set(Vector.getProduct(normal, -b.iMass));

//            double linearComponent = Vector.dotProduct(b.linear, normal) - Vector.dotProduct(a.linear, normal);
            displacement_dstVelocity = -ERP * Math.max(0d, deep - 0.05d);

            displasement_acummLambda[0] = 0d;
            displasement_acummLambda[1] = 0d;
            for (int i = 0; i < contactPointCount; i++) {
                raCrossN[i] = Vector.crossProduct(Vector.getDiff(contactPoint[i], a.centerMass), normal);
                rbCrossN[i] = -Vector.crossProduct(Vector.getDiff(contactPoint[i], b.centerMass), normal);
//                dstVelocity[i] = -Math.min(a.elasticity, b.elasticity) * (linearComponent + b.angular * rbCrossN[i] - a.angular * raCrossN[i]);
//                dstVelocity[i] = Math.min(0d, dstVelocity[i] + 1d);
                angularProjector1[i] = raCrossN[i] * a.iInertia;
                angularProjector2[i] = rbCrossN[i] * b.iInertia;
                invSumInvMass[i] = 1d / (a.iMass + b.iMass + raCrossN[i] * raCrossN[i] * a.iInertia + rbCrossN[i] * rbCrossN[i] * b.iInertia);
            }
            return true;
        }
        return false;
    }

    //TO-DO: split
    private boolean recalcCollis() {
        double[] dotAA = new double[a.axis.length];
        for (int i = 0; i < a.axis.length; i++) {
            dotAA[i] = Vector.dotProduct(a.vertex[i], a.axis[i]);
        }
        double[] dotBB = new double[b.axis.length];
        for (int i = 0; i < b.axis.length; i++) {
            dotBB[i] = Vector.dotProduct(b.vertex[i], b.axis[i]);
        }
        double[] minADotB = new double[b.axis.length];
        for (int i = 0; i < b.axis.length; i++) {
            double min = Double.MAX_VALUE;
            for (Vector vertex : a.vertex) {
                double dotProduct = Vector.dotProduct(vertex, b.axis[i]);
                if (dotProduct < min) {
                    min = dotProduct;
                }
            }
            minADotB[i] = min;
        }
        double[] minBDotA = new double[a.axis.length];
        for (int i = 0; i < a.axis.length; i++) {
            double min = Double.MAX_VALUE;
            for (Vector vertex : b.vertex) {
                double dotProduct = Vector.dotProduct(vertex, a.axis[i]);
                if (dotProduct < min) {
                    min = dotProduct;
                }
            }
            minBDotA[i] = min;
        }
        double minA = Double.MAX_VALUE;
        int minAxisIndexA = 0;
        for (int i = 0; i < a.axis.length; i++) {
            double diff = dotAA[i] - minBDotA[i];
            if (diff > 0) {
                if (diff < minA) {
                    minA = diff;
                    minAxisIndexA = i;
                }
            } else {
                return false;
            }
        }
        double minB = Double.MAX_VALUE;
        int minAxisIndexB = 0;
        for (int i = 0; i < b.axis.length; i++) {
            double diff = dotBB[i] - minADotB[i];
            if (diff > 0) {
                if (diff < minB) {
                    minB = diff;
                    minAxisIndexB = i;
                }
            } else {
                return false;
            }
        }
        Vector[] referenceVertex = new Vector[2];
        Vector[] incidentBodyVertex;
        Vector[] incidentBodyAxis;
        double bottomSide;
        boolean flip;
        if (minA <= minB) {
            normal.set(a.axis[minAxisIndexA]);
            deep = minA;
            referenceVertex[0] = a.vertex[minAxisIndexA];
            referenceVertex[1] = a.vertex[(minAxisIndexA + 1) % a.vertex.length];
            incidentBodyVertex = b.vertex;
            incidentBodyAxis = b.axis;
            bottomSide = dotAA[minAxisIndexA];
            flip = true;
            if (Solver.draw) {
                b.color = Color.red;
            }
        } else {
            normal.set(b.axis[minAxisIndexB]);
            deep = minB;
            referenceVertex[0] = b.vertex[minAxisIndexB];
            referenceVertex[1] = b.vertex[(minAxisIndexB + 1) % b.vertex.length];
            incidentBodyVertex = a.vertex;
            incidentBodyAxis = a.axis;
            bottomSide = dotBB[minAxisIndexB];
            flip = false;
            if (Solver.draw) {
                a.color = Color.red;
            }
        }
        Vector[] incidentVertex = findIncidentFace(incidentBodyAxis, incidentBodyVertex, normal);
        Vector referenceLine = normal.getLeftN();
        double negSide = -Vector.dotProduct(referenceVertex[1], referenceLine);
        if (clipIncidentFace(referenceLine.getInvert(), negSide, incidentVertex) < 2) {
            return false;
        }
        double posSide = Vector.dotProduct(referenceVertex[0], referenceLine);
        if (clipIncidentFace(referenceLine, posSide, incidentVertex) < 2) {
            return false;
        }
        contactPointCount = 0;
        double separation = Vector.dotProduct(incidentVertex[0], normal) - bottomSide;
        if (separation < 0) {
            contactPoint[contactPointCount].set(incidentVertex[0]);
            contactPointCount++;
            deep = -separation;
        }
        separation = Vector.dotProduct(incidentVertex[1], normal) - bottomSide;
        if (separation < 0) {
            contactPoint[contactPointCount].set(incidentVertex[1]);
            contactPointCount++;
            deep -= separation;
        }
        deep /= contactPointCount;
        if (flip) {
            normal.invert();
        }
        if (Solver.draw) {
            Build.buildPoint(incidentVertex[0], Color.blue);
            Build.buildPoint(incidentVertex[1], Color.blue);
            for (int i = 0; i < contactPointCount; i++) {
                Build.buildPoint(contactPoint[i], Color.green);
            }
        }
        return true;
    }

    private Vector[] findIncidentFace(Vector[] incBodyAxis, Vector[] incBodyVertex, Vector normal) {
        Vector[] incidentVertex = new Vector[2];
        int minDotIndex = 0;
        double minDot = Double.MAX_VALUE;
        for (int i = 0; i < incBodyVertex.length; i++) {
            double dot = Vector.dotProduct(incBodyAxis[i], normal);
            if (dot < minDot) {
                minDotIndex = i;
                minDot = dot;
            }
        }
        incidentVertex[0] = incBodyVertex[minDotIndex];
        incidentVertex[1] = incBodyVertex[(minDotIndex + 1) % incBodyVertex.length];
        return incidentVertex;
    }

    private int clipIncidentFace(Vector referenceLine, double d, Vector[] incidentVertex) {
        int suportPointCount = 0;
        Vector[] edges = {
            incidentVertex[0].copy(),
            incidentVertex[1].copy()
        };
        double dot1 = Vector.dotProduct(referenceLine, incidentVertex[0]) - d;
        double dot2 = Vector.dotProduct(referenceLine, incidentVertex[1]) - d;
        if (dot1 <= 0d) {
            edges[suportPointCount++].set(incidentVertex[0]);
        }
        if (dot2 <= 0d) {
            edges[suportPointCount++].set(incidentVertex[1]);
        }
        if (dot1 * dot2 < 0.0f) {
            double alpha = dot1 / (dot1 - dot2);
            edges[suportPointCount].set(incidentVertex[0]);
            edges[suportPointCount].plus(Vector.getProduct(Vector.getDiff(incidentVertex[0], incidentVertex[1]), alpha));
            suportPointCount++;
        }
        incidentVertex[0] = edges[0].copy();
        incidentVertex[1] = edges[1].copy();
        return suportPointCount;
    }

    public boolean isCorresponds(Body a, Body b) {
        return this.a == a && this.b == b;
    }

    public void drawAfterSolve() {
        float coef = (float) Math.min(1, Vector.getDiff(a.linear, b.linear).len() / 3);
        Build.buildLine(new Vector[]{a.centerMass, b.centerMass}, new Color(coef, 1 - coef, 0f));
    }

    ContactJoint(Body a, Body b) {
        this.a = a;
        this.b = b;

        contactPoint = new Vector[2];
        contactPoint[0] = new Vector(0d, 0d);
        contactPoint[1] = new Vector(0d, 0d);

        normal = new Vector(0d, 0d);
        linearProjector1 = new Vector(0d, 0d);
        lineatProjector2 = new Vector(0d, 0d);

        raCrossN = new double[]{0d, 0d};
        rbCrossN = new double[]{0d, 0d};
        angularProjector1 = new double[]{0d, 0d};
        angularProjector2 = new double[]{0d, 0d};
        acummLambda = new double[]{0d, 0d};
        displasement_acummLambda = new double[]{0d, 0d};
        invSumInvMass = new double[]{0d, 0d};
        dstVelocity = new double[]{0d, 0d};
    }

    private final static double ERP = 0.25d;
    private final Body a;
    private final Body b;

    private final Vector[] contactPoint;
    private int contactPointCount;
    private double deep;

    private final Vector normal;
    private final Vector linearProjector1, lineatProjector2;

    private final double[] raCrossN, rbCrossN;
    private final double[] angularProjector1, angularProjector2;

    private final double[] invSumInvMass;

    private final double dstVelocity[];
    private double displacement_dstVelocity;

    private final double[] acummLambda;
    private final double[] displasement_acummLambda;

}
