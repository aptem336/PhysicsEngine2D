package org.yourorghere;

public class ContactJoint {

    //коэффициент вытаскивания (насколько процентов от глубины вытащится объект за один кадр)
    private final static double ERP = 0.2d;

    //ограничители отскока, проникновения, трения
    private final BounceLimiter bounceLimiter;
    private final PenetrationLimiter penetrationLimiter;
    private final FrictionLimiter frictionLimiter;
    //тела, котороые столкнулись
    private final Body a;
    private final Body b;
    //точка столкновения
    public Vector contactPoint;
    //нормаль столкновени
    public final Vector normal;

    ContactJoint(Body a, Body b, Vector normal) {
        this.a = a;
        this.b = b;
        bounceLimiter = new BounceLimiter(normal);
        penetrationLimiter = new PenetrationLimiter(normal);
        frictionLimiter = new FrictionLimiter(normal.getLeftN());
        this.normal = normal;
    }

    //применения накопленного - начальное приближение, так система сходится быстрее
    public void preStep() {
        bounceLimiter.preStep();
        frictionLimiter.preStep();
    }

    //рьновляем с новой точкой и глубиной
    public void refresh(Vector contactPoint, double deep) {
        this.contactPoint = contactPoint;
        bounceLimiter.refresh();
        penetrationLimiter.deep = -deep;
        penetrationLimiter.refresh();
        frictionLimiter.refresh();
    }

    //решаем испульсы
    public long solveImpulse() {
        long time = System.currentTimeMillis();
        //решаем отскок
        bounceLimiter.solveImpulse(a.linear, b.linear, a.angular, b.angular);
        //решаем трение
        frictionLimiter.solveImpulse(a.linear, b.linear, a.angular, b.angular, bounceLimiter.accumLambda);
        return System.currentTimeMillis() - time;
    }

    //решаем проникновения
    public long solvePenetration() {
        long time = System.currentTimeMillis();
        penetrationLimiter.solveImpulse(a.d_linear, b.d_linear, a.d_angular, b.d_angular);
        return System.currentTimeMillis() - time;
    }

    //класс ограничителя по какой либо оси
    private class Limiter {

        //нормаль ограничения
        protected final Vector normal;
        //векторы, которые будут умноженные на испульс прибавляться к скорости
        //сразу умноженные на инвертированную массу
        protected final Vector linearProjectorA, linearProjectorB;
        //плечи
        protected double raCrossN, rbCrossN;
        //угол на произведение котрого с импульсом будем поворачивать тело, сразу умноженные на инвертированную инерцию
        protected double angularProjectorA, angularProjectorB;
        //сумма всех инвертированных масс (для решения уравнения сохранения импульса
        protected double invSumInvMass;
        //скорость перемещения пол нормали
        protected double dstVelocity;
        //накопленный и накапливаемый импульс
        protected final MutableDouble accumLambda;

        Limiter(Vector normal) {
            this.normal = normal;
            linearProjectorA = Vector.getProduct(normal, a.iMass);
            linearProjectorB = Vector.getProduct(normal, -b.iMass);
            accumLambda = new MutableDouble(0d);
        }

        //обновляем с новой точкой контакта
        protected void refresh() {
            //изменилсь плечи
            raCrossN = Vector.crossProduct(Vector.getDiff(contactPoint, a.centerMass), normal);
            rbCrossN = -Vector.crossProduct(Vector.getDiff(contactPoint, b.centerMass), normal);
            //изменилася угол поворота
            angularProjectorA = raCrossN * a.iInertia;
            angularProjectorB = rbCrossN * b.iInertia;
            //сумма всех масс
            invSumInvMass = 1d / (a.iMass + b.iMass + raCrossN * raCrossN * a.iInertia + rbCrossN * rbCrossN * b.iInertia);
        }

        //применение накопленных импульсов
        protected void preStep() {
            applyImpulse(a.linear, b.linear, a.angular, b.angular, accumLambda.getValue());
        }

        //вычисление нового импульса (читай уравнение сохранения испульса)
        protected double calcLambda(Vector linearA, Vector linearB, double angularA, double angularB, double distanseVelocity) {
            double lambda = 0;
            //импульс равен сумме всех ипульсов по текущей оси
            //сумма линейных импульсов
            lambda -= Vector.dotProduct(normal, linearA);
            lambda += Vector.dotProduct(normal, linearB);
            //сумма угловых импульсов
            lambda -= raCrossN * angularA;
            lambda -= rbCrossN * angularB;
            //скорость пермещения
            lambda -= distanseVelocity;
            //деленная на сумму масс
            return lambda * invSumInvMass;
        }

        protected void applyImpulse(Vector linearA, Vector linearB, MutableDouble angularA, MutableDouble angularB, double lambda) {
            linearA.add(linearProjectorA, lambda);
            linearB.add(linearProjectorB, lambda);
            angularA.add(lambda * angularProjectorA);
            angularB.add(lambda * angularProjectorB);
        }

        //решение импульсов
        protected void solveImpulse(Vector linearA, Vector linearB, MutableDouble angularA, MutableDouble angularB) {
            //вычисляем новый импульс
            double dLambda = calcLambda(linearA, linearB, angularA.getValue(), angularB.getValue(), dstVelocity);
            //если он обращаем движение так, что тела начинаеют сближаться
            if (dLambda + accumLambda.getValue() < 0d) {
                //обрезаем так, чтобы при его прибавлении накопленный импульс стал нулевым
                dLambda = -accumLambda.getValue();
            }
            //применяем импульсы
            applyImpulse(linearA, linearB, angularA, angularB, dLambda);
            //увеличиваем накопленный
            accumLambda.add(dLambda);
        }
    }

    private class BounceLimiter extends Limiter {

        public BounceLimiter(Vector normal) {
            super(normal);
        }

        @Override
        protected void refresh() {
            super.refresh();
            double linearComponent = Vector.dotProduct(b.linear, normal) - Vector.dotProduct(a.linear, normal);
            //скорость отдаления = коэффициенты упругости * на сумму скоростей до столкновения
            dstVelocity = -Solver.E * (linearComponent + b.angular.getValue() * rbCrossN - a.angular.getValue() * raCrossN);
        }
    }

    private class PenetrationLimiter extends Limiter {

        private double deep;

        public PenetrationLimiter(Vector normal) {
            super(normal);
        }

        @Override
        protected void refresh() {
            super.refresh();
            accumLambda.setValue(0d);
            //скорость отдаления = коэффициент вытакскивания * на глуюину (причем мы позволяем оставаться слегка внутри)
            dstVelocity = -ERP * Math.max(0d, deep - 0.1d);
        }
    }

    private class FrictionLimiter extends Limiter {

        public FrictionLimiter(Vector normal) {
            super(normal);
        }

        protected void solveImpulse(Vector linearA, Vector linearB, MutableDouble angularA, MutableDouble angularB, MutableDouble reactionForce) {
            double dLambda = calcLambda(linearA, linearB, angularA.getValue(), angularB.getValue(), 0);
            double accumLabmda = accumLambda.getValue();
            double frictionForce = accumLabmda + dLambda;
            //сила трения не может быть боьше, чем рекция опоры, умноженная на коэффициент трения
            //проверяем это условие
            //если привысило
            if (Math.abs(frictionForce) > (reactionForce.getValue() * Solver.F)) {
                //обрезаем до допустимого, сохраняя знак
                frictionForce = Math.signum(frictionForce) * reactionForce.getValue() * Solver.F;
                dLambda = frictionForce - accumLabmda;
            }
            //применяем
            applyImpulse(linearA, linearB, angularA, angularB, dLambda);
            //увеличиваем накопленный
            accumLambda.add(dLambda);
        }
    }
}
