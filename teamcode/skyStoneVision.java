package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;



import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.skyStoneVision.stonePosition.*;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */
@Disabled
public class skyStoneVision  {
    private boolean showContours = true;
    GripSkyStone skyStone;
    private Mat imageRGB;
    private Mat hsv;
    public int red = 0;
    private Mat thresholded;
    private MatOfKeyPoint detectedTargets;
    private Rect targetRect = null;
    private  Rect[] targetRects;
    private boolean findGoldMineral = false;
    private boolean contoursOutputIsReady = false;
    private List<MatOfPoint> contoursGold = new ArrayList<>();
    Point stone = new Point(0,0);
    stonePosition position = left;
    enum stonePosition{
        center,
        left,
        right
    }

    // this is just here so we can expose it later thru getContours.

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public synchronized void getGoldContours(List<MatOfPoint> newContours) {
        if (contoursOutputIsReady) {
            if (!(contoursGold.isEmpty())) {
                newContours.addAll(contoursGold);
            }
        }
        //return contoursGold;
    }

    public synchronized List<MatOfPoint> getGoldContours() {
        return contoursGold;
    }


    public synchronized boolean goldMineralFound () {
        return (findGoldMineral);
    }

    public void createObjects () {
        imageRGB = new Mat();
        hsv = new Mat();
        thresholded = new Mat();
        targetRect = null;
        targetRects = new Rect[2];
        skyStone   = new GripSkyStone();

    }

    public Mat processFrame(Mat rgba) {


        rgba.copyTo(imageRGB);

        skyStone.process(imageRGB);
        contoursOutputIsReady = false;
        //contoursGold = new ArrayList<>();
        contoursGold.clear();
//        contoursGold = gripGold.filterContoursOutput();
        skyStone.filterContoursOutput(contoursGold);
        contoursOutputIsReady = true;
        if ((!(skyStone.blobs.isEmpty())) && (showContours)) {
            stone.y =  skyStone.findBlobsOutput().get(0).y;
            stone.x =  skyStone.findBlobsOutput().get(0).x;
            if(stone.x < 230 + red){
                position = right;
            }else{
                position = center;
            }
            Imgproc.drawContours(imageRGB, contoursGold, -1,  new Scalar(0, 250, 200), 4, 8);
            Log.d("s", String.valueOf(stone.x));
            Imgproc.drawMarker(imageRGB,stone,new Scalar(255, 50,10),Imgproc.MARKER_CROSS,50,10);
            findGoldMineral = true;
        }

        return imageRGB; // display the image seen by the camera
    }

}
