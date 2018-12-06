package org.yourorghere;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import static org.yourorghere.Solver.F;
import static org.yourorghere.Solver.E;

public class Listener implements KeyListener, MouseListener, MouseMotionListener, MouseWheelListener {

    public static final Vector CURSOR_LOCATION = new Vector(0d, 0d);

    @Override
    public void keyPressed(KeyEvent e) {
    }

    @Override
    public void keyReleased(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
            Solver.reset = true;
        }
        if (e.getKeyCode() == KeyEvent.VK_SPACE) {
            Solver.move = !Solver.move;
        }
        if (e.getKeyCode() == KeyEvent.VK_ENTER) {
            Solver.draw = !Solver.draw;
        }
        if (e.getKeyCode() == KeyEvent.VK_Q) {
            F -= 0.0625;
            //ограничиваем мужду 0 и 1
            F = Math.min(1d, F);
            F = Math.max(0d, F);
        }
        if (e.getKeyCode() == KeyEvent.VK_E) {
            F += 0.0625;
            //ограничиваем мужду 0 и 1
            F = Math.min(1d, F);
            F = Math.max(0d, F);
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
        Solver.mousePressed = true;
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        Solver.mousePressed = false;
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        E -= e.getPreciseWheelRotation() * 0.0625;
        //ограничиваем мужду 0 и 1
        E = Math.min(0.8d, E);
        E = Math.max(0d, E);
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        CURSOR_LOCATION.x = (e.getPoint().x - 400d) / 3.05d;
        CURSOR_LOCATION.y = (-e.getPoint().y + 380) / 3.05d;
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        CURSOR_LOCATION.x = (e.getPoint().x - 400d) / 3.05d;
        CURSOR_LOCATION.y = (-e.getPoint().y + 380) / 3.05d;
    }

}
