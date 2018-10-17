package org.yourorghere;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

public class Listener implements KeyListener, MouseListener, MouseWheelListener {

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_W) {
            Solver.diff.plus(new Vector(0d, 1d));
        }
        if (e.getKeyCode() == KeyEvent.VK_S) {
            Solver.diff.plus(new Vector(0d, -1d));
        }
        if (e.getKeyCode() == KeyEvent.VK_A) {
            Solver.diff.plus(new Vector(-1d, 0d));
        }
        if (e.getKeyCode() == KeyEvent.VK_D) {
            Solver.diff.plus(new Vector(1d, 0d));
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
        Solver.addObject(new Body((e.getPoint().x - 400d) / 3.05d, (-e.getPoint().y + 380) / 3.05, Math.toRadians(30), Util.inscribedInCircle(Math.random() * 10d + 10d, (int) (Math.random() * 5 + 3)), 1d, 0.5d, 1d));
    }

}
