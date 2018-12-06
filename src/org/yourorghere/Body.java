package org.yourorghere;

import java.awt.Color;

public class Body {

    //собственные проекции на собственные оси
    private final double[] selfDots;

    //полчаем проецкции каждой точки на поданные оси оси
    public double[][] getDots(Vector[] axis) {
        double[][] dots = new double[axis.length][vertex.length];
        //перебираем все оси и все точки
        for (int i = 0; i < axis.length; i++) {
            for (int j = 0; j < vertex.length; j++) {
                //скалярное произведение и есть проекция
                dots[i][j] = Vector.dotProduct(vertex[j], axis[i]);
            }
        }
        return dots;
    }

    //собственные проеции
    public double[] getSelfDots() {
        //для всех осей
        for (int i = 0; i < axis.length; i++) {
            //минимальную проекцию даст точка, образоваашая эту самую ось - с тем же индексом
            selfDots[i] = Vector.dotProduct(vertex[i], axis[i]);
        }
        return selfDots;
    }

    public Body(double x, double y, double angle, Vector[] vertex_offset, double density, Color color) {
        //полуаем начальные положения точек
        //иницализируем перменные
        this.vertex_offset = vertex_offset;
        vertex = new Vector[vertex_offset.length];
        selfDots = new double[vertex_offset.length];
        //вычисляем центр масс
        centerMass_offset = Util.centerMass(vertex_offset);
        centerMass = new Vector(0d, 0d);
        axis = new Vector[vertex_offset.length];
        //вычисляем радиус описывающей окружности
        R = Util.R(vertex_offset);
        //масса = площадб 8 плотность
        double mass = Util.area(vertex_offset) * density;
        linear = new Vector(0d, 0d);
        d_linear = new Vector(0d, 0d);
        location = new Vector(x, y);
        //инвертированная масса, нужная для удобства, чтобы каждый раз не делить
        iMass = 1d / mass;

        double inertia = mass * R * R / 2;
        angular = new MutableDouble(0d);
        d_angular = new MutableDouble(0d);
        this.angle = angle;
        //инвертированная инерция, нужен для удобства, чтобы каждый раз не делить
        iInertia = 1d / inertia;

        this.color = color;
        //выделяем память
        allocateMemory();
        //интрегирем положение первый раз
        integrateLocation();
    }

    //интегиируем положение
    public final void integrateLocation() {
        //перемещение равно сумме скорости на DT и псевдоскорости
        Vector comp_linear = Vector.getSum(Vector.getProduct(linear, DT), d_linear);
        //добавлям перемещение к положению
        location.add(comp_linear);

        //поворот равен сумме поворота на DT и псевдоповорота
        double comp_angular = angular.getValue() * DT + d_angular.getValue();
        angle += comp_angular;

        //косинус и синус текущего угла, чтобы не вычислять много раз
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        for (int i = 0; i < vertex_offset.length; i++) {
            //точка равна начальному положению повернутому на текущий угод
            vertex[i].setRotated(cos, sin, vertex_offset[i], centerMass_offset);
            //плюс текущее положение
            vertex[i].add(location);
        }
        //центра масс равен центру масс начальному, сещенному на текущее положение
        centerMass.setSum(centerMass_offset, location);
        //вычисляем разделяющие оси
        for (int i = 0; i < axis.length; i++) {
            //ось = нормаль = перпенжикуляр разницы между точками
            axis[i].setDiff(vertex[i], vertex[(i + 1) % vertex.length]);
            axis[i].setLeftN();
            //обрезаем до единицы
            axis[i].normalize();
        }
        //обнуляем псевдоскорости
        d_linear.set(Vector.NULL);
        d_angular.setValue(0d);
    }

    private void allocateMemory() {
        //выделяем память под все переменнные
        for (int i = 0; i < vertex.length; i++) {
            vertex[i] = new Vector(0d, 0d);
        }
        for (int i = 0; i < axis.length; i++) {
            axis[i] = new Vector(0d, 0d);
        }
    }

    public static double DT = 1d / 120d;
    //геометрия
    //храним начальные положения и реальные (текущие) положения точек
    private final Vector[] vertex_offset;
    public final Vector[] vertex;

    public final Vector centerMass_offset;
    public final Vector centerMass;

    public final Vector[] axis;
    //для широкой фазы
    public final double R;
    //перемещение
    public final Vector linear;
    public final Vector d_linear;
    public final Vector location;
    public final double iMass;
    //вращение
    public final MutableDouble angular;
    public final MutableDouble d_angular;
    public double angle;
    public final double iInertia;

    public Color color;
}
