package org.yourorghere;

import java.awt.Color;
import javax.media.opengl.GL;

public class Build {

    public static void buildObject(Vector[] vertexS, Color color) {
        GL gl = PhysicEngine.gl;
        gl.glPushMatrix();
        gl.glColor4f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f, color.getAlpha() / 255f);
        gl.glBegin(GL.GL_LINE_LOOP);
        for (Vector vertex : vertexS) {
            gl.glVertex2d(vertex.x, vertex.y);
        }
        gl.glEnd();
        gl.glPopMatrix();
    }

    public static void buildLine(Vector[] vertex, Color color) {
        GL gl = PhysicEngine.gl;
        gl.glPushMatrix();
        gl.glColor4f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f, color.getAlpha() / 255f);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d(vertex[0].x, vertex[0].y);
        gl.glVertex2d(vertex[1].x, vertex[1].y);
        gl.glEnd();
        gl.glPopMatrix();
    }
    
    public static void buildPoint(Vector point, Color color){
        Vector[] vertex = Util.inscribedInCircle(1, 10);
        for (Vector vector : vertex) {
            vector.plus(point);
        }
        buildObject(vertex, color);
    }
}
