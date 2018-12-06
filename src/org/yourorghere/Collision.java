package org.yourorghere;

import java.awt.Color;

public class Collision {

    //просчёт колизии
    public static boolean calcCollis(Body a, Body b, ContactJoint[] contacts) {
        //сумма радиусов описывающих окржуностей
        double R = a.R + b.R;
        //если расстояние между центрами больше, чем сумма радиусов в квадрате
        if (Vector.getDiff(b.location, a.location).squareLen() > R * R) {
            //объекты не пересекаются
            return false;
        }
        //получаем проекции на собственные оси
        double[] dotsAA = a.getSelfDots();
        double[] dotsBB = b.getSelfDots();
        //получаем проекции на чужие оси
        double[][] dotsAB = a.getDots(b.axis);
        double[][] dotsBA = b.getDots(a.axis);
        //получаем минимумы на каждой оси - ближе всего
        double[] mindotAB = getMinDots(dotsAB);
        double[] mindotBA = getMinDots(dotsBA);
        double minDiffA = Double.MAX_VALUE;
        //индекс точки давшей минимум
        int minAxisIndexA = 0;
        for (int i = 0; i < a.axis.length; i++) {
            //проверяем пересечение проекций
            double diff = dotsAA[i] - mindotBA[i];
            //если пересеклись
            if (diff > 0) {
                //если новое значение меньще минимума
                if (diff < minDiffA) {
                    //обновляем минимум и индекс точки давшей минимум
                    minDiffA = diff;
                    minAxisIndexA = i;
                }
                //иначе объекты не пересеклись    
            } else {
                return false;
            }
        }
        //всё аналогично, только для другого объекта
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
        //нормаль пересечения
        Vector normal;
        //нужно-ли перевернуть нормаль
        boolean flip;
        //точки оси давшей нормаль
        Vector[] referenceVertex = new Vector[2];
        //точки инициирующего пересечение объекта
        Vector[] incidentBodyVertex;
        //разделяющие оси  инициирующего пересечение объекта
        Vector[] incidentBodyAxis;
        //нижня граница по текущей нормали
        double bottomSide;
        //проверяем какое тело дало ось с минимальным пересечение
        if (minDiffA <= minDiffB) {
            //задаём нормаль из нормалей объекта
            normal = a.axis[minAxisIndexA];
            //точки её образовавшие
            referenceVertex[0] = a.vertex[minAxisIndexA];
            referenceVertex[1] = a.vertex[(minAxisIndexA + 1) % a.vertex.length];
            //точки и оси "виноватого" в пересчение тела
            incidentBodyVertex = b.vertex;
            incidentBodyAxis = b.axis;
            //нижняя граница
            bottomSide = dotsAA[minAxisIndexA];
            //перевернуть надо
            flip = true;
        } else {
            //аналогично
            normal = b.axis[minAxisIndexB];
            referenceVertex[0] = b.vertex[minAxisIndexB];
            referenceVertex[1] = b.vertex[(minAxisIndexB + 1) % b.vertex.length];
            incidentBodyVertex = a.vertex;
            incidentBodyAxis = a.axis;
            bottomSide = dotsBB[minAxisIndexB];
            flip = false;
        }
        //поиск точек, давших ребро "виноватое" в пересечении
        Vector[] incidentVertex = findIncidentFace(incidentBodyAxis, incidentBodyVertex, normal);
        //ось перпендикулярная нормали пересечения
        Vector referenceLine = normal.getLeftN();
        //проекция на эту ось крайних точек
        double negSide = -Vector.dotProduct(referenceVertex[1], referenceLine);
        //обрезаем "виноватое ребро" так, чтобы оне не выходило за объект
        if (clipIncidentFace(referenceLine.getInvert(), negSide, incidentVertex) < 2) {
            return false;
        }
        double posSide = Vector.dotProduct(referenceVertex[0], referenceLine);
        if (clipIncidentFace(referenceLine, posSide, incidentVertex) < 2) {
            return false;
        }
        //глубина проникновения = разнице между проекцией точки "виноватого" ребра на нормаль и проекцией точки давшей эту нормаль
        double deep = Vector.dotProduct(incidentVertex[0], normal) - bottomSide;
        //если глубина есть - точка внутри тела
        if (deep < 0) {
            //инициализируем контакт
            Vector newNormal = flip ? normal.getInvert() : normal;
            //проверяем был ли между этими объектами налогичный
            //если не было
            if (contacts[0] == null || contacts[0].normal != newNormal) {
                //создаём
                contacts[0] = new ContactJoint(a, b, newNormal);
            }
            //всегда обновляем с новое точкой и глубиной
            contacts[0].refresh(incidentVertex[0], deep);
            //отрисовка точки контакта
            if (Solver.draw) {
                Build.buildPoint(incidentVertex[0], Color.red);
            }
            //если глубины нет - точка вне объекта    
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
        //отрисовка линии соединяющей центры с цеветом соответсвующим скорости между этими объектами
        if (Solver.draw) {
            float coef = (float) Math.min(1, Vector.getDiff(a.linear, b.linear).len() / 3);
            Build.buildLine(new Vector[]{a.centerMass, b.centerMass}, new Color(coef, 1 - coef, 0f));
        }
        //объекты пересеклись - возвращаем тру
        return true;
    }

    //ищем минимум среди проекицй для каждой оси
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

    //ищем ребро "виноватое" в пересечении
    //оно самое параллельное ребру давшему нормаль
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

    //обрезаем "виноватое" ребро так, чтобы оно не вылезало за объект
    private static int clipIncidentFace(Vector referenceLine, double d, Vector[] incidentVertex) {
        int suportPointCount = 0;
        Vector[] edges = {incidentVertex[0].copy(), incidentVertex[1].copy()};
        //проверка на персечение
        double dot1 = Vector.dotProduct(referenceLine, incidentVertex[0]) - d;
        double dot2 = Vector.dotProduct(referenceLine, incidentVertex[1]) - d;
        //если точка внутри
        if (dot1 <= 0d) {
            //она и есть нужная нам
            edges[suportPointCount++].set(incidentVertex[0]);
        }
        //если точка внутри
        if (dot2 <= 0d) {
            //она и есть нужная нам
            edges[suportPointCount++].set(incidentVertex[1]);
        }
        //простая интерполяция (поиск нужного решения), если точки по разную сторону
        if (dot1 * dot2 < 0.0f) {
            double alpha = dot1 / (dot1 - dot2);
            edges[suportPointCount].set(incidentVertex[0]);
            edges[suportPointCount].add(Vector.getProduct(Vector.getDiff(incidentVertex[0], incidentVertex[1]), alpha));
            suportPointCount++;
        }
        //точки давшие "виноватое" ребро получены
        incidentVertex[0] = edges[0];
        incidentVertex[1] = edges[1];
        return suportPointCount;
    }
}
