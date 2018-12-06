package org.yourorghere;

import com.sun.opengl.util.Animator;
import com.sun.opengl.util.GLUT;
import java.awt.Frame;
import java.awt.Toolkit;
import java.awt.Window;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLCanvas;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.glu.GLU;

public class PhysicEngine implements GLEventListener {

    public static GL gl;
    private static GLU glu;
    private static GLUT glut;

    private static long time;
    private static long lc_time;
    private static int frames = 300;
    private static int fps;

    private static Listener listener;
    private static GLCanvas canvas;
    private static Animator animator;

    public static void main(String[] args) {
        //создаём окно
        Frame frame = new Frame("Physic!");
        frame.setType(Window.Type.UTILITY);
        //получаем размеры экрана
        int size = Toolkit.getDefaultToolkit().getScreenSize().height - 30;
        frame.setSize(size, size);
        frame.setVisible(true);
        frame.setResizable(false);
        frame.setLocationRelativeTo(null);

        //создаём канву - место рисовани¤
        canvas = new GLCanvas();
        //создаЄм и добавляем слушателей
        listener = new Listener();
        canvas.addKeyListener(listener);
        canvas.addMouseListener(listener);
        canvas.addMouseMotionListener(listener);
        canvas.addMouseWheelListener(listener);
        canvas.addGLEventListener(new PhysicEngine2D());
        canvas.setBounds(0, 0, frame.getWidth(), frame.getHeight() - 30);
        //создаЄм управл¤ющий методами отрисовки аниматор
        animator = new Animator(canvas);

        frame.add(canvas);
        //добавл¤ем слушател¤ окна
        frame.addWindowListener(new WindowAdapter() {
            @Override
            //переопредел¤ем операцию закрыти¤ окна
            public void windowClosing(WindowEvent e) {
                new Thread(() -> {
                    //останавливаем аниматор
                    animator.stop();
                    //выходим со статусом 0 (без ошибок)
                    System.exit(0);
                }).start();
            }
        });
        //запускаем аниматор
        animator.start();
    }

    @Override
    public void init(GLAutoDrawable drawable) {
        gl = drawable.getGL();
        glu = new GLU();
        glut = new GLUT();
        //очищаем буфер черным цветом
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        //включаем сглаживание
        gl.glShadeModel(GL.GL_SMOOTH);
    }

    @Override
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
        final float h = (float) width / (float) height;
        gl.glViewport(0, 0, width, height);
        gl.glMatrixMode(GL.GL_PROJECTION);
        gl.glLoadIdentity();
        glu.gluPerspective(45.0f, h, 1.0, 100000.0);
        gl.glMatrixMode(GL.GL_MODELVIEW);
        gl.glLoadIdentity();
    }

    @Override
    public void display(GLAutoDrawable drawable) {
        //очищаем буферы
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        gl.glLoadIdentity();
        //настраиваем камеру 
        //3координаты - положение камеры
        //3координаты - точка наблюдения (сцена)
        //3координаты = направление
        glu.gluLookAt(0d, 0d, 300d, 0d, 0d, 0d, 0d, 1d, 0d);

        //блок для подсчёта fps
        //текущее врем¤
        time = System.currentTimeMillis();
        //если прошло больше секунды
        if (time - lc_time >= 1000) {
            //значение fps равно насчитанным кадрам
            fps = frames;
            //сохран¤ем врем¤ дл¤ вычислени¤ разницы
            lc_time = time;
            //обнул¤ем количество фреймов
            frames = 0;
            //коэфицент частоты интегрирование зависит от фпс
            Body.DT = 0.25d / fps;
        }
        //увеличиваем количество кадров
        frames++;
        //вызываем шаг физики
        Solver.step();

        //надписи
        //цвет надписей
        gl.glColor4f(1f, 1f, 0f, 0.5f);

        gl.glWindowPos2i(10, drawable.getHeight() - 45);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, fps + "");

        gl.glWindowPos2i(10, drawable.getHeight() - 65);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "E = " + Solver.E);

        gl.glWindowPos2i(10, drawable.getHeight() - 85);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "F = " + Solver.F);

        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 45);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Escape to reset place");

        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 65);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Space to pause");
        
        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 85);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Enter to draw calculation");
        
        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 105);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Drag mouse to do shot");

        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 125);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Mouse wheel - change Elasticity");

        gl.glWindowPos2i(drawable.getWidth() - 195, drawable.getHeight() - 145);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_12, "Q/E - change Friction");

        gl.glFlush();
    }

    @Override
    public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {
    }
}
