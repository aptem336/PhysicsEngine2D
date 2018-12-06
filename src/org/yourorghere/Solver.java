package org.yourorghere;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;

public class Solver {

    //������ ���
    private static final ArrayList<Body> BODIES = new ArrayList<Body>();
    //���������� �������� ������� ��������� � �������������
    private static final int IIC = 200, IPC = 200;

    //����������� ���������
    public static double E = 0.2d;
    //����������� ������
    public static double F = 0.2d;
    //��� ���������� �� c����, ��������� � �����������
    public static boolean reset = true, draw = true, move = true;
    //������ ��������� ������� ������
    private static ContactJoint[][][] CONTACTS;

    //��� ������
    public static void step() {
        //���� ������ �����
        if (reset) {
            //������� ������ ���
            BODIES.clear();
            //�������������� �� �������
            initObjects();
            reset = false;
        }
        //������ ������������ ���������
        ContactJoint[] contactArray = new ContactJoint[0];
        //���������� ���� ������ � ������
        for (int i = 0; i < BODIES.size() - 1; i++) {
            for (int j = i + 1; j < BODIES.size(); j++) {
                //���� ������������ ������������
                if (Collision.calcCollis(BODIES.get(i), BODIES.get(j), CONTACTS[i][j - 1])) {
                    //���� ����� �� ���
                    if (CONTACTS[i][j - 1][0] != null) {
                        //����������� ������ �� 1
                        contactArray = Arrays.copyOf(contactArray, contactArray.length + 1);
                        //���������� �����
                        contactArray[contactArray.length - 1] = CONTACTS[i][j - 1][0];
                    }
                    //���� ����� �� ���
                    if (CONTACTS[i][j - 1][1] != null) {
                        //����������� ������ �� 1
                        contactArray = Arrays.copyOf(contactArray, contactArray.length + 1);
                        //���������� �����
                        contactArray[contactArray.length - 1] = CONTACTS[i][j - 1][1];
                    }
                }
            }
        }
        //��� ���� ���, �������� ���� � ����������� ������
        BODIES.stream().filter((body) -> (body.iMass != 0)).forEachOrdered((Body body) -> {
            //��������� ����������
            body.linear.add(Vector.GRAVITY);
        });
        //���������� ������������ �������� ��� ���� ���������
        for (ContactJoint contact : contactArray) {
            contact.preStep();
        }
        //������� ������� �� ��������
        for (int n = 0; n < IIC; n++) {
            //��� ���� ���������
            for (ContactJoint contact : contactArray) {
                contact.solveImpulse();
            }
        }
        //������� ������� �� �������������
        for (int n = 0; n < IPC; n++) {
            //��� ���� ���������
            for (ContactJoint contact : contactArray) {
                contact.solvePenetration();
            }
        }
        //����������� ���������
        if (move) {
            BODIES.forEach((object) -> {
                object.integrateLocation();
            });
        }
        //���� ���� ������
        if (mousePressed) {
            //���� ���� �� ������
            if (myBody == null) {
                //������ ����� ����
                myBody = new Body(SP.x, SP.y, 0d, Util.inscribedInCircle(0d, 3d, (int) (Math.random() * 5 + 3)), 3d, Color.red);
            }
            //���� ������� ����� ��������� ��������� � ������� ���������� �������
            FORCE.set(Vector.getDiff(SP, Listener.CURSOR_LOCATION));
            //�������� ����� = 80
            FORCE.set(Vector.getProduct(FORCE.getNormalized(), Math.min(FORCE.len(), 80d)));
            //������ ���������������� ������
            Build.buildObject(myBody.vertex, myBody.color);
            //������ ����� ����
            Build.buildLine(new Vector[]{SP, Vector.getSum(SP, FORCE)}, Color.yellow);
            //���� ���� �� ������    
        } else {
            //���� ���� �����������
            if (myBody != null) {
                //������ ���� �������� = ���� * 20
                myBody.linear.set(Vector.getProduct(FORCE.getInvert(), 20d));
                //��������� ������ � ������ ��������������
                addObject(myBody);
                //�������� - ������ ����� �� ���������
                myBody = null;
            }
        }
        //������ �������
        BODIES.forEach((body) -> {
            Build.buildObject(body.vertex, body.color);
        });
    }

    //��������� ������� - ������ �������
    private static final Vector SP = Vector.NULL.copy();
    //���� ������
    private static final Vector FORCE = Vector.NULL.copy();
    //���������������� ����
    private static Body myBody;
    //��� ���������� - ������� ����
    public static boolean mousePressed = false;

    //������������� �� �������
    private static void initObjects() {
        //��������� => � ����������� ���������� => � ����������� ������
        addObject(new Body(0d, -80d, 0d, Util.inscribedInEllipse(Math.PI / 4d, 140d, 10d, 4), 1d / 0d, Color.white));
        addObject(new Body(120d, -20d, 0d, Util.inscribedInCircle(Math.PI / 2d, 20d, 3), 1d / 0d, Color.white));
        addObject(new Body(-120d, -40d, 0d, Util.inscribedInCircle(Math.PI / 2d, 20d, 3), 1d / 0d, Color.white));
        //��� ����� � 6� ��������
        addTown(80d, 20d, 6);
        addTown(-80d, 0d, 6);
    }

    private static void addTown(double x, double y, int count) {
        //��������� => � ����������� ���������� => � ����������� ������
        addObject(new Body(x, y, 0d, Util.inscribedInEllipse(Math.PI / 4d, 20d, 2d, 4), 1d / 0d, Color.white));
        //���������� ������� ���
        for (int i = 1; i <= count; i++) {
            //������ ������� �� ����������� ������
            double size = 20d / Math.pow(1.5, i);
            //������� �� y
            y += size;
            //����� �����
            addObject(new Body(x, y, 0d, Util.inscribedInCircle(Math.PI / 4d, size, 4), 1d, Color.blue));

        }
    }

    //��������� ���������� ������� � ��������������
    public static void addObject(Body body) {
        //����� ������ ��������� ������ � ������
        ContactJoint[][][] newContacts = new ContactJoint[BODIES.size()][BODIES.size()][2];
        //�������� ������ ��������
        for (int i = 0; i < BODIES.size() - 1; i++) {
            System.arraycopy(CONTACTS[i], 0, newContacts[i], 0, CONTACTS[i].length);
        }
        //��������� ����� ��� ������ �������
        for (ContactJoint[][] newContact : newContacts) {
            newContact[newContacts.length - 1] = new ContactJoint[2];
        }
        //���������� � ���� ����
        CONTACTS = newContacts;
        //�������� � ������ ��������
        BODIES.add(body);
    }
}
