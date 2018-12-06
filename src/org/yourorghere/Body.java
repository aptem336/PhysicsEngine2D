package org.yourorghere;

import java.awt.Color;

public class Body {

    //����������� �������� �� ����������� ���
    private final double[] selfDots;

    //������� ��������� ������ ����� �� �������� ��� ���
    public double[][] getDots(Vector[] axis) {
        double[][] dots = new double[axis.length][vertex.length];
        //���������� ��� ��� � ��� �����
        for (int i = 0; i < axis.length; i++) {
            for (int j = 0; j < vertex.length; j++) {
                //��������� ������������ � ���� ��������
                dots[i][j] = Vector.dotProduct(vertex[j], axis[i]);
            }
        }
        return dots;
    }

    //����������� �������
    public double[] getSelfDots() {
        //��� ���� ����
        for (int i = 0; i < axis.length; i++) {
            //����������� �������� ���� �����, ������������ ��� ����� ��� - � ��� �� ��������
            selfDots[i] = Vector.dotProduct(vertex[i], axis[i]);
        }
        return selfDots;
    }

    public Body(double x, double y, double angle, Vector[] vertex_offset, double density, Color color) {
        //������� ��������� ��������� �����
        //������������� ���������
        this.vertex_offset = vertex_offset;
        vertex = new Vector[vertex_offset.length];
        selfDots = new double[vertex_offset.length];
        //��������� ����� ����
        centerMass_offset = Util.centerMass(vertex_offset);
        centerMass = new Vector(0d, 0d);
        axis = new Vector[vertex_offset.length];
        //��������� ������ ����������� ����������
        R = Util.R(vertex_offset);
        //����� = ������� 8 ���������
        double mass = Util.area(vertex_offset) * density;
        linear = new Vector(0d, 0d);
        d_linear = new Vector(0d, 0d);
        location = new Vector(x, y);
        //��������������� �����, ������ ��� ��������, ����� ������ ��� �� ������
        iMass = 1d / mass;

        double inertia = mass * R * R / 2;
        angular = new MutableDouble(0d);
        d_angular = new MutableDouble(0d);
        this.angle = angle;
        //��������������� �������, ����� ��� ��������, ����� ������ ��� �� ������
        iInertia = 1d / inertia;

        this.color = color;
        //�������� ������
        allocateMemory();
        //���������� ��������� ������ ���
        integrateLocation();
    }

    //����������� ���������
    public final void integrateLocation() {
        //����������� ����� ����� �������� �� DT � ��������������
        Vector comp_linear = Vector.getSum(Vector.getProduct(linear, DT), d_linear);
        //�������� ����������� � ���������
        location.add(comp_linear);

        //������� ����� ����� �������� �� DT � ��������������
        double comp_angular = angular.getValue() * DT + d_angular.getValue();
        angle += comp_angular;

        //������� � ����� �������� ����, ����� �� ��������� ����� ���
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        for (int i = 0; i < vertex_offset.length; i++) {
            //����� ����� ���������� ��������� ����������� �� ������� ����
            vertex[i].setRotated(cos, sin, vertex_offset[i], centerMass_offset);
            //���� ������� ���������
            vertex[i].add(location);
        }
        //������ ���� ����� ������ ���� ����������, ��������� �� ������� ���������
        centerMass.setSum(centerMass_offset, location);
        //��������� ����������� ���
        for (int i = 0; i < axis.length; i++) {
            //��� = ������� = ������������� ������� ����� �������
            axis[i].setDiff(vertex[i], vertex[(i + 1) % vertex.length]);
            axis[i].setLeftN();
            //�������� �� �������
            axis[i].normalize();
        }
        //�������� ��������������
        d_linear.set(Vector.NULL);
        d_angular.setValue(0d);
    }

    private void allocateMemory() {
        //�������� ������ ��� ��� �����������
        for (int i = 0; i < vertex.length; i++) {
            vertex[i] = new Vector(0d, 0d);
        }
        for (int i = 0; i < axis.length; i++) {
            axis[i] = new Vector(0d, 0d);
        }
    }

    public static double DT = 1d / 120d;
    //���������
    //������ ��������� ��������� � �������� (�������) ��������� �����
    private final Vector[] vertex_offset;
    public final Vector[] vertex;

    public final Vector centerMass_offset;
    public final Vector centerMass;

    public final Vector[] axis;
    //��� ������� ����
    public final double R;
    //�����������
    public final Vector linear;
    public final Vector d_linear;
    public final Vector location;
    public final double iMass;
    //��������
    public final MutableDouble angular;
    public final MutableDouble d_angular;
    public double angle;
    public final double iInertia;

    public Color color;
}
