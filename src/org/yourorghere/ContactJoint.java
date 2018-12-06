package org.yourorghere;

public class ContactJoint {

    //����������� ������������ (��������� ��������� �� ������� ��������� ������ �� ���� ����)
    private final static double ERP = 0.2d;

    //������������ �������, �������������, ������
    private final BounceLimiter bounceLimiter;
    private final PenetrationLimiter penetrationLimiter;
    private final FrictionLimiter frictionLimiter;
    //����, �������� �����������
    private final Body a;
    private final Body b;
    //����� ������������
    public Vector contactPoint;
    //������� �����������
    public final Vector normal;

    ContactJoint(Body a, Body b, Vector normal) {
        this.a = a;
        this.b = b;
        bounceLimiter = new BounceLimiter(normal);
        penetrationLimiter = new PenetrationLimiter(normal);
        frictionLimiter = new FrictionLimiter(normal.getLeftN());
        this.normal = normal;
    }

    //���������� ������������ - ��������� �����������, ��� ������� �������� �������
    public void preStep() {
        bounceLimiter.preStep();
        frictionLimiter.preStep();
    }

    //��������� � ����� ������ � ��������
    public void refresh(Vector contactPoint, double deep) {
        this.contactPoint = contactPoint;
        bounceLimiter.refresh();
        penetrationLimiter.deep = -deep;
        penetrationLimiter.refresh();
        frictionLimiter.refresh();
    }

    //������ ��������
    public long solveImpulse() {
        long time = System.currentTimeMillis();
        //������ ������
        bounceLimiter.solveImpulse(a.linear, b.linear, a.angular, b.angular);
        //������ ������
        frictionLimiter.solveImpulse(a.linear, b.linear, a.angular, b.angular, bounceLimiter.accumLambda);
        return System.currentTimeMillis() - time;
    }

    //������ �������������
    public long solvePenetration() {
        long time = System.currentTimeMillis();
        penetrationLimiter.solveImpulse(a.d_linear, b.d_linear, a.d_angular, b.d_angular);
        return System.currentTimeMillis() - time;
    }

    //����� ������������ �� ����� ���� ���
    private class Limiter {

        //������� �����������
        protected final Vector normal;
        //�������, ������� ����� ���������� �� ������� ������������ � ��������
        //����� ���������� �� ��������������� �����
        protected final Vector linearProjectorA, linearProjectorB;
        //�����
        protected double raCrossN, rbCrossN;
        //���� �� ������������ ������� � ��������� ����� ������������ ����, ����� ���������� �� ��������������� �������
        protected double angularProjectorA, angularProjectorB;
        //����� ���� ��������������� ���� (��� ������� ��������� ���������� ��������
        protected double invSumInvMass;
        //�������� ����������� ��� �������
        protected double dstVelocity;
        //����������� � ������������� �������
        protected final MutableDouble accumLambda;

        Limiter(Vector normal) {
            this.normal = normal;
            linearProjectorA = Vector.getProduct(normal, a.iMass);
            linearProjectorB = Vector.getProduct(normal, -b.iMass);
            accumLambda = new MutableDouble(0d);
        }

        //��������� � ����� ������ ��������
        protected void refresh() {
            //��������� �����
            raCrossN = Vector.crossProduct(Vector.getDiff(contactPoint, a.centerMass), normal);
            rbCrossN = -Vector.crossProduct(Vector.getDiff(contactPoint, b.centerMass), normal);
            //���������� ���� ��������
            angularProjectorA = raCrossN * a.iInertia;
            angularProjectorB = rbCrossN * b.iInertia;
            //����� ���� ����
            invSumInvMass = 1d / (a.iMass + b.iMass + raCrossN * raCrossN * a.iInertia + rbCrossN * rbCrossN * b.iInertia);
        }

        //���������� ����������� ���������
        protected void preStep() {
            applyImpulse(a.linear, b.linear, a.angular, b.angular, accumLambda.getValue());
        }

        //���������� ������ �������� (����� ��������� ���������� ��������)
        protected double calcLambda(Vector linearA, Vector linearB, double angularA, double angularB, double distanseVelocity) {
            double lambda = 0;
            //������� ����� ����� ���� �������� �� ������� ���
            //����� �������� ���������
            lambda -= Vector.dotProduct(normal, linearA);
            lambda += Vector.dotProduct(normal, linearB);
            //����� ������� ���������
            lambda -= raCrossN * angularA;
            lambda -= rbCrossN * angularB;
            //�������� ����������
            lambda -= distanseVelocity;
            //�������� �� ����� ����
            return lambda * invSumInvMass;
        }

        protected void applyImpulse(Vector linearA, Vector linearB, MutableDouble angularA, MutableDouble angularB, double lambda) {
            linearA.add(linearProjectorA, lambda);
            linearB.add(linearProjectorB, lambda);
            angularA.add(lambda * angularProjectorA);
            angularB.add(lambda * angularProjectorB);
        }

        //������� ���������
        protected void solveImpulse(Vector linearA, Vector linearB, MutableDouble angularA, MutableDouble angularB) {
            //��������� ����� �������
            double dLambda = calcLambda(linearA, linearB, angularA.getValue(), angularB.getValue(), dstVelocity);
            //���� �� �������� �������� ���, ��� ���� ��������� ����������
            if (dLambda + accumLambda.getValue() < 0d) {
                //�������� ���, ����� ��� ��� ����������� ����������� ������� ���� �������
                dLambda = -accumLambda.getValue();
            }
            //��������� ��������
            applyImpulse(linearA, linearB, angularA, angularB, dLambda);
            //����������� �����������
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
            //�������� ��������� = ������������ ��������� * �� ����� ��������� �� ������������
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
            //�������� ��������� = ����������� ������������� * �� ������� (������ �� ��������� ���������� ������ ������)
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
            //���� ������ �� ����� ���� �����, ��� ������ �����, ���������� �� ����������� ������
            //��������� ��� �������
            //���� ���������
            if (Math.abs(frictionForce) > (reactionForce.getValue() * Solver.F)) {
                //�������� �� �����������, �������� ����
                frictionForce = Math.signum(frictionForce) * reactionForce.getValue() * Solver.F;
                dLambda = frictionForce - accumLabmda;
            }
            //���������
            applyImpulse(linearA, linearB, angularA, angularB, dLambda);
            //����������� �����������
            accumLambda.add(dLambda);
        }
    }
}
