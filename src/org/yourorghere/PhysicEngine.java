package org.yourorghere;

import com.sun.opengl.util.GLUT;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.glu.GLU;

public class PhysicEngine implements GLEventListener {

    static GL gl;
    static GLU glu;
    static GLUT glut;
    static Interface Interface;

    static long millis1;
    static long millis0;
    static int fps;
    static int frames = 60;

    boolean step = true;

    public static void main(String[] args) {
        Interface = new Interface();
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
        new Thread() {
            @Override
            public void run() {
                while (true) {

                }
            }
        }.start();
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

        frames++;
        millis1 = System.currentTimeMillis();
        if (millis1 - millis0 >= 1000) {
            fps = frames;
            millis0 = millis1;
            frames = 0;
        }

        Solver.step();

        gl.glColor4f(1f, 1f, 0f, 0.5f);
        gl.glWindowPos2i(5, drawable.getHeight() - 20);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, "" + fps);
        gl.glWindowPos2i(drawable.getWidth() - 240, drawable.getHeight() - 20);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, "Space to draw calculations");
        gl.glWindowPos2i(drawable.getWidth() - 240, drawable.getHeight() - 50);
        glut.glutBitmapString(GLUT.BITMAP_HELVETICA_18, "Mouse wheel to add object");

        gl.glFlush();
    }

    @Override
    public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {
    }
}
