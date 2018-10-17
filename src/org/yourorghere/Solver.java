package org.yourorghere;

import java.awt.Color;
import java.util.ArrayList;

public class Solver {

    private static final ArrayList<Body> BODIES = new ArrayList<Body>();
    private static final ArrayList<ContactJoint> CONTACTS = new ArrayList<ContactJoint>();
    private final static int IIC = 200, IPC = 200;
    public static boolean reset = true, draw = false;

    public static void step() {
        if (reset) {
            BODIES.clear();
            CONTACTS.clear();
            addObjects();
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
            body.linear.plus(Vector.gravity);
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
                contact.drawAfterSolve();
            });
        }
        temp.location.plus(diff);
        diff.setToZero();
    }

    static Vector diff = new Vector(0d, 0d);

    private static void addObjects() {
        //статичные => с бесконечной плотностью
        BODIES.add(new Body(0d, -225d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.5d, 1d));
        BODIES.add(new Body(-250d, 58d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.5d, 1d));
        BODIES.add(new Body(250d, 58d, Math.toRadians(45), Util.inscribedInCircle(200d, 4), 1d / 0d, 0.5d, 1d));
        temp = new Body(0d, 0d, Math.toRadians(45), Util.inscribedInCircle(20d, 4), 1d, 0.3d, 1d);
        BODIES.add(temp);
    }

    public static Body temp;

    public static void addObject(Body object) {
        BODIES.add(object);
    }
}
