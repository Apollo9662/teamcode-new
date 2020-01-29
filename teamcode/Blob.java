package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static org.firstinspires.ftc.teamcode.MathFunctions.dist2D;
@Disabled
class Blob{
    double minx;
    double miny;
    double maxx;
    double maxy;

    ArrayList<org.opencv.core.Point> verPath = new ArrayList();

    Blob(double x, double y){
        this.minx = x;
        this.miny = y;
        this.maxx = x;
        this.maxy = y;
        verPath.add(new org.opencv.core.Point((float)x,(float)y));
    }

    void add(double x, double y){
        verPath.add(new org.opencv.core.Point((float)x,(float)y));
        this.minx = min(minx,x);
        this.miny = min(miny,y);
        this.maxx = max(maxx,x);
        this.maxy = max(maxy,y);
    }


    boolean isNear(double x, double y){
        double cx = (minx + maxx)/2;
        double cy = (miny + maxy)/2;

        double d = dist2D(new Point(cx,cy),new Point(x,y));
        if(d < 30){
            return true;
        }
        else{
            return false;
        }
    }

    public Point getCenter() {
        double cx = (maxx + minx)/2;
        double cy = (maxy + miny)/2;

        return new Point(cx,cy);
    }
}