package org.yourorghere;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;

public class Solver {

    //массив тел
    private static final ArrayList<Body> BODIES = new ArrayList<Body>();
    //количество итераций решения импульсов и проникновений
    private static final int IIC = 200, IPC = 200;

    //коэффициент упругости
    public static double E = 0.2d;
    //коэффициент трения
    public static double F = 0.2d;
    //лог переменные на cброс, отрисовку и перемещение
    public static boolean reset = true, draw = true, move = true;
    //массив контактов каждого каждым
    private static ContactJoint[][][] CONTACTS;

    //шаг физики
    public static void step() {
        //если вызван сброс
        if (reset) {
            //очищаем массив тел
            BODIES.clear();
            //инициализируем по дефолту
            initObjects();
            reset = false;
        }
        //массив существующих контактов
        ContactJoint[] contactArray = new ContactJoint[0];
        //перепираем тела каждое с каждым
        for (int i = 0; i < BODIES.size() - 1; i++) {
            for (int j = i + 1; j < BODIES.size(); j++) {
                //если столкновение присутствует
                if (Collision.calcCollis(BODIES.get(i), BODIES.get(j), CONTACTS[i][j - 1])) {
                    //если точка не нул
                    if (CONTACTS[i][j - 1][0] != null) {
                        //увеличиваем массив на 1
                        contactArray = Arrays.copyOf(contactArray, contactArray.length + 1);
                        //записываем точку
                        contactArray[contactArray.length - 1] = CONTACTS[i][j - 1][0];
                    }
                    //если точка не нул
                    if (CONTACTS[i][j - 1][1] != null) {
                        //увеличиваем массив на 1
                        contactArray = Arrays.copyOf(contactArray, contactArray.length + 1);
                        //записываем точку
                        contactArray[contactArray.length - 1] = CONTACTS[i][j - 1][1];
                    }
                }
            }
        }
        //для всех тел, исключая тела с бесконечной массой
        BODIES.stream().filter((body) -> (body.iMass != 0)).forEachOrdered((Body body) -> {
            //добавляем графитацию
            body.linear.add(Vector.GRAVITY);
        });
        //Применение накопленного импульса для всех контактов
        for (ContactJoint contact : contactArray) {
            contact.preStep();
        }
        //Решение системы на импульсы
        for (int n = 0; n < IIC; n++) {
            //для всех контактов
            for (ContactJoint contact : contactArray) {
                contact.solveImpulse();
            }
        }
        //Решение системы на проникновения
        for (int n = 0; n < IPC; n++) {
            //для всех контактов
            for (ContactJoint contact : contactArray) {
                contact.solvePenetration();
            }
        }
        //интегрируем положение
        if (move) {
            BODIES.forEach((object) -> {
                object.integrateLocation();
            });
        }
        //если мышь зажата
        if (mousePressed) {
            //если тело не задано
            if (myBody == null) {
                //создаём новое тело
                myBody = new Body(SP.x, SP.y, 0d, Util.inscribedInCircle(0d, 3d, (int) (Math.random() * 5 + 3)), 3d, Color.red);
            }
            //сила разница между начальным положение и текущим положением курсора
            FORCE.set(Vector.getDiff(SP, Listener.CURSOR_LOCATION));
            //максимум длина = 80
            FORCE.set(Vector.getProduct(FORCE.getNormalized(), Math.min(FORCE.len(), 80d)));
            //рисуем польховательский объект
            Build.buildObject(myBody.vertex, myBody.color);
            //рисуем линию силы
            Build.buildLine(new Vector[]{SP, Vector.getSum(SP, FORCE)}, Color.yellow);
            //если мышь не зажата    
        } else {
            //если тело установлено
            if (myBody != null) {
                //придаём телу скорость = сила * 20
                myBody.linear.set(Vector.getProduct(FORCE.getInvert(), 20d));
                //добавляем объект в массив обрабатываемых
                addObject(myBody);
                //обнуляем - теперь ничем не управляем
                myBody = null;
            }
        }
        //рисуем объекты
        BODIES.forEach((body) -> {
            Build.buildObject(body.vertex, body.color);
        });
    }

    //стартовая позиция - откуда каидаем
    private static final Vector SP = Vector.NULL.copy();
    //сила броска
    private static final Vector FORCE = Vector.NULL.copy();
    //пользоваетльское тело
    private static Body myBody;
    //лог переменная - нажатия мыши
    public static boolean mousePressed = false;

    //инициализация по дефолту
    private static void initObjects() {
        //статичные => с бесконечной плотностью => с бесконечной массой
        addObject(new Body(0d, -80d, 0d, Util.inscribedInEllipse(Math.PI / 4d, 140d, 10d, 4), 1d / 0d, Color.white));
        addObject(new Body(120d, -20d, 0d, Util.inscribedInCircle(Math.PI / 2d, 20d, 3), 1d / 0d, Color.white));
        addObject(new Body(-120d, -40d, 0d, Util.inscribedInCircle(Math.PI / 2d, 20d, 3), 1d / 0d, Color.white));
        //две башни с 6ю кубиками
        addTown(80d, 20d, 6);
        addTown(-80d, 0d, 6);
    }

    private static void addTown(double x, double y, int count) {
        //статичные => с бесконечной плотностью => с бесконечной массой
        addObject(new Body(x, y, 0d, Util.inscribedInEllipse(Math.PI / 4d, 20d, 2d, 4), 1d / 0d, Color.white));
        //количество кубиков раз
        for (int i = 1; i <= count; i++) {
            //размер зависит от порядкового номера
            double size = 20d / Math.pow(1.5, i);
            //смещаем по y
            y += size;
            //новый кубик
            addObject(new Body(x, y, 0d, Util.inscribedInCircle(Math.PI / 4d, size, 4), 1d, Color.blue));

        }
    }

    //процедура добавления объекта в обрабатываемые
    public static void addObject(Body body) {
        //новый массив контактов каждый с каждым
        ContactJoint[][][] newContacts = new ContactJoint[BODIES.size()][BODIES.size()][2];
        //копируем старые значения
        for (int i = 0; i < BODIES.size() - 1; i++) {
            System.arraycopy(CONTACTS[i], 0, newContacts[i], 0, CONTACTS[i].length);
        }
        //добавляем новые для нового объекта
        for (ContactJoint[][] newContact : newContacts) {
            newContact[newContacts.length - 1] = new ContactJoint[2];
        }
        //записываем в наше поле
        CONTACTS = newContacts;
        //обавляем в массив объектов
        BODIES.add(body);
    }
}
