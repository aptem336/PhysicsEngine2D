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
        //������ ����
        Frame frame = new Frame("Physic!");
        frame.setType(Window.Type.UTILITY);
        //�������� ������� ������
        int size = Toolkit.getDefaultToolkit().getScreenSize().height - 30;
        frame.setSize(size, size);
        frame.setVisible(true);
        frame.setResizable(false);
        frame.setLocationRelativeTo(null);

        //������ ����� - ����� ��������
        canvas = new GLCanvas();
        //������ � ��������� ����������
        listener = new Listener();
        canvas.addKeyListener(listener);
        canvas.addMouseListener(listener);
        canvas.addMouseMotionListener(listener);
        canvas.addMouseWheelListener(listener);
        canvas.addGLEventListener(new PhysicEngine2D());
        canvas.setBounds(0, 0, frame.getWidth(), frame.getHeight() - 30);
        //������ ���������� �������� ��������� ��������
        animator = new Animator(canvas);

        frame.add(canvas);
        //�������� �������� ����
        frame.addWindowListener(new WindowAdapter() {
            @Override
            //������������� �������� ������� ����
            public void windowClosing(WindowEvent e) {
                new Thread(() -> {
                    //������������� ��������
                    animator.stop();
                    //������� �� �������� 0 (��� ������)
                    System.exit(0);
                }).start();
            }
        });
        //��������� ��������
        animator.start();
    }

    @Override
    public void init(GLAutoDrawable drawable) {
        gl = drawable.getGL();
        glu = new GLU();
        glut = new GLUT();
        //������� ����� ������ ������
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        //�������� �����������
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
        //������� ������
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        gl.glLoadIdentity();
        //����������� ������ 
        //3���������� - ��������� ������
        //3���������� - ����� ���������� (�����)
        //3���������� = �����������
        glu.gluLookAt(0d, 0d, 300d, 0d, 0d, 0d, 0d, 1d, 0d);

        //���� ��� �������� fps
        //������� ����
        time = System.currentTimeMillis();
        //���� ������ ������ �������
        if (time - lc_time >= 1000) {
            //�������� fps ����� ����������� ������
            fps = frames;
            //�������� ���� �� ��������� �������
            lc_time = time;
            //������� ���������� �������
            frames = 0;
            //��������� ������� �������������� ������� �� ���
            Body.DT = 0.25d / fps;
        }
        //����������� ���������� ������
        frames++;
        //�������� ��� ������
        Solver.step();

        //�������
        //���� ��������
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
