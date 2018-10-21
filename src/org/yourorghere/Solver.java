package org.yourorghere;

import java.awt.Color;
import java.util.ArrayList;

public class Solver {

    public static final ArrayList<Body> BODIES = new ArrayList<Body>();
    public static boolean reset = true, draw = false;

    private static final ArrayList<ContactJoint> CONTACTS = new ArrayList<ContactJoint>();
    private static final int IIC = 300, IPC = 300;

    public static void step() {
        if (reset) {
            BODIES.clear();
            CONTACTS.clear();
            initObjects();
            reset = false;
        }
        int it = 0;
        while (it < CONTACTS.size()) {
            ContactJoint contact = CONTACTS.get(it);
            if (!contact.refresh()) {
                CONTACTS.remove(it);
            }
            it++;
        }
        for (int i = 0; i < BODIES.size() - 1; i++) {
            for (int j = i + 1; j < BODIES.size(); j++) {
                Body a = BODIES.get(i);
                Body b = BODIES.get(j);
                boolean contains = false;
                for (ContactJoint contact : CONTACTS) {
                    if (contact.isCorresponds(a, b)) {
                        contains = true;
                        break;
                    }
                }
                if (contains) {
                    continue;
                }
                ContactJoint contact = new ContactJoint(a, b);
                if (contact.refresh()) {
                    CONTACTS.add(contact);
                }
            }
        }
        BODIES.stream().filter((body) -> (body.iMass != 0)).forEachOrdered((Body body) -> {
            body.linear.plus(Vector.getProduct(Vector.GRAVITY, PhysicEngine.PC));
        });
        CONTACTS.forEach((contact) -> {
            contact.preStep();
        });
        //Решение системы на импульсы
        for (int i = 0; i < IIC; i++) {
            CONTACTS.forEach((contact) -> {
                contact.solveImpulse();
            });
        }
        //Решение системы на проникновения
        for (int i = 0; i < IPC; i++) {
            CONTACTS.forEach((contact) -> {
                contact.solvePenetration();
            });
        }
        BODIES.forEach((object) -> {
            object.integrateLocation();
        });
        BODIES.forEach((body) -> {
            Build.buildObject(body.vertex, body.color);
            body.color = Color.white;
        });
        if (draw) {
            CONTACTS.forEach((contact) -> {
                contact.drawSolve();
            });
        }
    }

    public static Body temp;

    private static void initObjects() {
        //статичные => с бесконечной плотностью
        BODIES.add(new Body(0d, -225d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.2d, 0.3d));
        BODIES.add(new Body(-250d, 58d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.2d, 0.3d));
        BODIES.add(new Body(250d, 58d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.2d, 0.3d));
        for (int i = -100; i <= 100; i += 15) {
            for (int j = -60; j <= 160; j += 15) {
                BODIES.add(new Body(i, j, Math.toRadians(0), Util.inscribedInCircle(7.5d, 7), 1d, 0.2d, 0.2d));
            }
        }
        temp = new Body(0d, 0d, Math.toRadians(45), Util.inscribedInCircle(20d, 4), 1d, 0.2d, 0.3d);
//        BODIES.add(temp);
    }
}
