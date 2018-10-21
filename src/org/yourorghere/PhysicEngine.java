package org.yourorghere;

import com.sun.opengl.util.Animator;
import com.sun.opengl.util.GLUT;
import java.awt.Frame;
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
    private static int frames = 60;
    private static int fps;
    public static double PC;

    private static Listener myListener;
    private static GLCanvas canvas;
    private static Animator animator;

    public static void main(String[] args) {
        Frame frame = new Frame("Physic!");
        frame.setType(Window.Type.UTILITY);
        frame.setSize(800, 800);
        frame.setVisible(true);
        frame.setResizable(false);
        frame.setLocationRelativeTo(null);

        myListener = new Listener();
        canvas = new GLCanvas();
        canvas.addKeyListener(myListener);
        canvas.addMouseListener(myListener);
        canvas.addMouseMotionListener(myListener);
        canvas.addMouseWheelListener(myListener);
        canvas.addGLEventListener(new PhysicEngine());
        canvas.setBounds(0, 0, frame.getWidth(), frame.getHeight() - 30);
        animator = new Animator(canvas);

        frame.add(canvas);

        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                new Thread(() -> {
                    animator.stop();
                    System.exit(0);
                }).start();
            }
        });
        animator.start();
    }

    @Override
    public void init(GLAutoDrawable drawable) {
        gl = drawable.getGL();
        glu = new GLU();
        glut = new GLUT();
        System.err.println("INIT GL IS: " + gl.getClass().getName());
        gl.setSwapInterval(1);
        gl.glClearColor(0.15f, 0.15f, 0.15f, 0.0f);
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
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        gl.glLoadIdentity();
        gl.glTranslatef(0, 0, 0);
        glu.gluLookAt(0, 0, 300, 0, 0, 0, 0, 1, 0);

        time = System.currentTimeMillis();
        if (time - lc_time >= 1000) {
            fps = frames;
            lc_time = time;
            frames = 0;
            PC = 60d / fps;
        }
        frames++;
        Solver.step();

        gl.glColor4f(1f, 1f, 0f, 0.5f);
        gl.glWindowPos2i(10, drawable.getHeight() - 45);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, fps + "");
        gl.glWindowPos2i(drawable.getWidth() - 235, drawable.getHeight() - 45);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, "Space to draw calculations");
        gl.glWindowPos2i(drawable.getWidth() - 235, drawable.getHeight() - 70);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, "Mouse wheel to add object");

        gl.glFlush();
    }

    @Override
    public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {
    }
}
