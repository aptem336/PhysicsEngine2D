package org.yourorghere;

import java.awt.Color;

public class Collision {

    //������� �������
    public static boolean calcCollis(Body a, Body b, ContactJoint[] contacts) {
        //����� �������� ����������� �����������
        double R = a.R + b.R;
        //���� ���������� ����� �������� ������, ��� ����� �������� � ��������
        if (Vector.getDiff(b.location, a.location).squareLen() > R * R) {
            //������� �� ������������
            return false;
        }
        //�������� �������� �� ����������� ���
        double[] dotsAA = a.getSelfDots();
        double[] dotsBB = b.getSelfDots();
        //�������� �������� �� ����� ���
        double[][] dotsAB = a.getDots(b.axis);
        double[][] dotsBA = b.getDots(a.axis);
        //�������� �������� �� ������ ��� - ����� �����
        double[] mindotAB = getMinDots(dotsAB);
        double[] mindotBA = getMinDots(dotsBA);
        double minDiffA = Double.MAX_VALUE;
        //������ ����� ������ �������
        int minAxisIndexA = 0;
        for (int i = 0; i < a.axis.length; i++) {
            //��������� ����������� ��������
            double diff = dotsAA[i] - mindotBA[i];
            //���� �����������
            if (diff > 0) {
                //���� ����� �������� ������ ��������
                if (diff < minDiffA) {
                    //��������� ������� � ������ ����� ������ �������
                    minDiffA = diff;
                    minAxisIndexA = i;
                }
                //����� ������� �� �����������    
            } else {
                return false;
            }
        }
        //�� ����������, ������ ��� ������� �������
        double minDiffB = Double.MAX_VALUE;
        int minAxisIndexB = 0;
        for (int i = 0; i < b.axis.length; i++) {
            double diff = dotsBB[i] - mindotAB[i];
            if (diff > 0) {
                if (diff < minDiffB) {
                    minDiffB = diff;
                    minAxisIndexB = i;
                }
            } else {
                return false;
            }
        }
        //������� �����������
        Vector normal;
        //�����-�� ����������� �������
        boolean flip;
        //����� ��� ������ �������
        Vector[] referenceVertex = new Vector[2];
        //����� ������������� ����������� �������
        Vector[] incidentBodyVertex;
        //����������� ���  ������������� ����������� �������
        Vector[] incidentBodyAxis;
        //����� ������� �� ������� �������
        double bottomSide;
        //��������� ����� ���� ���� ��� � ����������� �����������
        if (minDiffA <= minDiffB) {
            //����� ������� �� �������� �������
            normal = a.axis[minAxisIndexA];
            //����� � ������������
            referenceVertex[0] = a.vertex[minAxisIndexA];
            referenceVertex[1] = a.vertex[(minAxisIndexA + 1) % a.vertex.length];
            //����� � ��� "����������" � ���������� ����
            incidentBodyVertex = b.vertex;
            incidentBodyAxis = b.axis;
            //������ �������
            bottomSide = dotsAA[minAxisIndexA];
            //����������� ����
            flip = true;
        } else {
            //����������
            normal = b.axis[minAxisIndexB];
            referenceVertex[0] = b.vertex[minAxisIndexB];
            referenceVertex[1] = b.vertex[(minAxisIndexB + 1) % b.vertex.length];
            incidentBodyVertex = a.vertex;
            incidentBodyAxis = a.axis;
            bottomSide = dotsBB[minAxisIndexB];
            flip = false;
        }
        //����� �����, ������ ����� "���������" � �����������
        Vector[] incidentVertex = findIncidentFace(incidentBodyAxis, incidentBodyVertex, normal);
        //��� ���������������� ������� �����������
        Vector referenceLine = normal.getLeftN();
        //�������� �� ��� ��� ������� �����
        double negSide = -Vector.dotProduct(referenceVertex[1], referenceLine);
        //�������� "��������� �����" ���, ����� ��� �� �������� �� ������
        if (clipIncidentFace(referenceLine.getInvert(), negSide, incidentVertex) < 2) {
            return false;
        }
        double posSide = Vector.dotProduct(referenceVertex[0], referenceLine);
        if (clipIncidentFace(referenceLine, posSide, incidentVertex) < 2) {
            return false;
        }
        //������� ������������� = ������� ����� ��������� ����� "����������" ����� �� ������� � ��������� ����� ������ ��� �������
        double deep = Vector.dotProduct(incidentVertex[0], normal) - bottomSide;
        //���� ������� ���� - ����� ������ ����
        if (deep < 0) {
            //�������������� �������
            Vector newNormal = flip ? normal.getInvert() : normal;
            //��������� ��� �� ����� ����� ��������� ����������
            //���� �� ����
            if (contacts[0] == null || contacts[0].normal != newNormal) {
                //������
                contacts[0] = new ContactJoint(a, b, newNormal);
            }
            //������ ��������� � ����� ������ � ��������
            contacts[0].refresh(incidentVertex[0], deep);
            //��������� ����� ��������
            if (Solver.draw) {
                Build.buildPoint(incidentVertex[0], Color.red);
            }
            //���� ������� ��� - ����� ��� �������    
        } else {
            contacts[0] = null;
        }
        deep = Vector.dotProduct(incidentVertex[1], normal) - bottomSide;
        if (deep < 0) {
            Vector newNormal = flip ? normal.getInvert() : normal;
            if (contacts[1] == null || contacts[1].normal != newNormal) {
                contacts[1] = new ContactJoint(a, b, newNormal);
            }
            contacts[1].refresh(incidentVertex[1], deep);
            if (Solver.draw) {
                Build.buildPoint(incidentVertex[1], Color.red);
            }
        } else {
            contacts[1] = null;
        }
        //��������� ����� ����������� ������ � ������� �������������� �������� ����� ����� ���������
        if (Solver.draw) {
            float coef = (float) Math.min(1, Vector.getDiff(a.linear, b.linear).len() / 3);
            Build.buildLine(new Vector[]{a.centerMass, b.centerMass}, new Color(coef, 1 - coef, 0f));
        }
        //������� ����������� - ���������� ���
        return true;
    }

    //���� ������� ����� �������� ��� ������ ���
    private static double[] getMinDots(double[][] dots) {
        double[] mindot = new double[dots.length];
        for (int i = 0; i < dots.length; i++) {
            double min = Double.MAX_VALUE;
            for (int j = 0; j < dots[i].length; j++) {
                if (dots[i][j] < min) {
                    min = dots[i][j];
                }
            }
            mindot[i] = min;
        }
        return mindot;
    }

    //���� ����� "���������" � �����������
    //��� ����� ������������ ����� ������� �������
    private static Vector[] findIncidentFace(Vector[] incBodyAxis, Vector[] incBodyVertex, Vector normal) {
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

    //�������� "���������" ����� ���, ����� ��� �� �������� �� ������
    private static int clipIncidentFace(Vector referenceLine, double d, Vector[] incidentVertex) {
        int suportPointCount = 0;
        Vector[] edges = {incidentVertex[0].copy(), incidentVertex[1].copy()};
        //�������� �� ����������
        double dot1 = Vector.dotProduct(referenceLine, incidentVertex[0]) - d;
        double dot2 = Vector.dotProduct(referenceLine, incidentVertex[1]) - d;
        //���� ����� ������
        if (dot1 <= 0d) {
            //��� � ���� ������ ���
            edges[suportPointCount++].set(incidentVertex[0]);
        }
        //���� ����� ������
        if (dot2 <= 0d) {
            //��� � ���� ������ ���
            edges[suportPointCount++].set(incidentVertex[1]);
        }
        //������� ������������ (����� ������� �������), ���� ����� �� ������ �������
        if (dot1 * dot2 < 0.0f) {
            double alpha = dot1 / (dot1 - dot2);
            edges[suportPointCount].set(incidentVertex[0]);
            edges[suportPointCount].add(Vector.getProduct(Vector.getDiff(incidentVertex[0], incidentVertex[1]), alpha));
            suportPointCount++;
        }
        //����� ������ "���������" ����� ��������
        incidentVertex[0] = edges[0];
        incidentVertex[1] = edges[1];
        return suportPointCount;
    }
}
