package org.yourorghere;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

public class Listener implements KeyListener, MouseListener, MouseMotionListener, MouseWheelListener {

    public static final Vector LOCATION = new Vector(0d, 0d);

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_W) {
            Solver.temp.location.plus(new Vector(0d, 1d));
        }
        if (e.getKeyCode() == KeyEvent.VK_S) {
            Solver.temp.location.plus(new Vector(0d, -1d));
        }
        if (e.getKeyCode() == KeyEvent.VK_A) {
            Solver.temp.location.plus(new Vector(-1d, 0d));
        }
        if (e.getKeyCode() == KeyEvent.VK_D) {
            Solver.temp.location.plus(new Vector(1d, 0d));
        }
        if (e.getKeyCode() == KeyEvent.VK_Q) {
            Solver.temp.angle -= Math.toRadians(5);
        }
        if (e.getKeyCode() == KeyEvent.VK_E) {
            Solver.temp.angle += Math.toRadians(5);
        }
    }

    @Override
    public void keyReleased(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
            Solver.reset = true;
        }
        if (e.getKeyCode() == KeyEvent.VK_SPACE) {
            Solver.draw = !Solver.draw;
        }
    }

    @Override
    public void keyTyped(KeyEvent e) {
    }

    @Override
    public void mouseClicked(MouseEvent e) {
    }

    @Override
    public void mousePressed(MouseEvent e) {
    }

    @Override
    public void mouseReleased(MouseEvent e) {
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        Solver.BODIES.add(new Body(LOCATION.x, LOCATION.y, 0d, Util.inscribedInCircle(Math.random() * 5d + 5d, (int) (Math.random() * 5 + 3)), 1d, 0.2d, 0.3d));
    }

    @Override
    public void mouseDragged(MouseEvent e) {
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        LOCATION.x = (e.getPoint().x - 400d) / 3.05d;
        LOCATION.y = (-e.getPoint().y + 380) / 3.05d;
    }

}
