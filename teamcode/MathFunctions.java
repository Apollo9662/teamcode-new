package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
@Disabled
public class MathFunctions {
    public static double hypotenuseX;
    public static double hypotenuseY;

    final static double EPSILON = 1e-12;
    /**
     *
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while(angle < - Math.PI){
            angle+= 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle-= 2* Math.PI;
        }
        return angle;
    }
    /**public static ArrayList<Point> linecircleinterction(Point circleCenter, double circleRadius,
                                                        Point linePoint1, Point linePooint2){
        if(Math.abs(linePoint1.y - linePooint2.y) < 0.003){
            linePoint1.y = linePooint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x - linePooint2.x) < 0.003){
            linePoint1.x = linePooint2.x + 0.003;
        }

        double m1 = (linePooint2.y - linePoint1.y)/(linePooint2.x - linePooint2.y);

        double quatricaA = 1.0 + Math.pow(m1,2);

        double x1 =linePoint1.x - circleCenter.x;
        double y1 =linePoint1.y - circleCenter.y;

        double quatricaB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);

        double quatricaC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1) + Math.pow(y1,2) - Math.pow(circleRadius,2);

        ArrayList<Point> allPoints =new ArrayList<>();

        try{
            double xRoot1 = (-quatricaB + Math.sqrt(Math.pow(quatricaB,2) - (4.0 * quatricaA * quatricaC)))/(2.0 * quatricaA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePooint2.x ? linePoint1.x : linePooint2.x;
            double maxX = linePoint1.x > linePooint2.x ? linePoint1.x : linePooint2.x;

            if(xRoot1 > minX && xRoot1 < maxX ){
                allPoints.add(new Point (xRoot1,yRoot1));
            }

            double xRoot2 = (-quatricaB - Math.sqrt(Math.pow(quatricaB,2) - (4.0 * quatricaA * quatricaC)))/(2.0 * quatricaA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX ){
                allPoints.add(new Point (xRoot2,yRoot2));
            }

        }catch (Exception e){

        }
        return allPoints;
    }*/
    /**
     *
     * @param a
     * @param b
     * @return
     */
    public static double dist2D(Point a,Point b){
        return Math. sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
    }

    /**
     *
     * @param center
     * @param radius
     * @param pointA
     * @param pointB
     * @return
     */
    public static List<Point> getCircleLineIntersectionPoint(Point center, double radius,
                                                             Point pointA, Point pointB) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return Collections.emptyList();
        }
        // if disc == 0 ... dealt with later
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        Point p1 = new Point(pointA.x - baX * abScalingFactor1, pointA.y
                - baY * abScalingFactor1);
        if (disc == 0) { // abScalingFactor1 == abScalingFactor2
            return Collections.singletonList(p1);
        }
        Point p2 = new Point(pointA.x - baX * abScalingFactor2, pointA.y
                - baY * abScalingFactor2);
        return Arrays.asList(p1, p2);
    }



    public static double map(double v,
                             double start1, double start2,
                             double end1, double end2) {

        if (Math.abs(end1 - start1) < EPSILON) {
            throw new ArithmeticException("/ 0");
        }

        double offset = start2;
        double ratio = (end2 - start2) / (end1 - start1);
        return ratio * (v - start1) + offset;
    }

    public static boolean inRange(double v,double min,double max){
        if(v > min && v < max){
            return true;
        }else {
            return false;
        }
    }

    public static double calculateAngle(double opposite, double adjacent){
        double radians = Math.atan(opposite/adjacent);
        if (adjacent < 0){
            return radians;
        }else {
            return Math.toRadians(Math.toDegrees(radians) + 180);
        }
    }

    public static double moveX(double hypotenuse, double angle) {
        double advanceX;
        if (hypotenuse == 0) {
            hypotenuseX = 0;
            advanceX = 0;
        } else {
            hypotenuseX += hypotenuse * Math.cos(angle);

            advanceX = hypotenuseX;

            hypotenuseX = hypotenuseX - advanceX;
        }
        return advanceX;
    }
    public static double moveY(double hypotenuse, double angle) {
        double advanceY;
        if (hypotenuse == 0) {
            hypotenuseY = 0;
            advanceY = 0;
        } else {
            hypotenuseY += hypotenuse * Math.sin(angle);

            advanceY = hypotenuseY;

            hypotenuseY = hypotenuseY - advanceY;
        }
        return advanceY;
    }

    public static double between(double value, double min, double max){
        if(value < min){
            value = min;
        }
        else if(value > max){
            value = max;
        }

        return value;
    }
}
