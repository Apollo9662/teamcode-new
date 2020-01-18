package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.Surface;
import android.view.View;
import android.view.ViewGroup;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;

public abstract class OpenCVPipeline implements CameraBridgeViewBase.CvCameraViewListener2 {

    private final String TAG = "OpenCv Pipeline";
    AppUtil appUtil  = AppUtil.getInstance();
    JavaCameraView javaCameraView;
    protected Context contextL;
    View view;
    private boolean initStarted = false;
    private boolean inited = false;

    private BaseLoaderCallback mLoaderCallBack = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS:
                    Log.d(TAG, "callback: OpenCv success");
                    break;
                default:
                    Log.d(TAG, "callback: fail to load opencv");
                    break;
            }
        }
    };


    public void init(final Context context, final int cameraIndex) {
        Log.d(TAG, "Initializing OpenCV");
        this.initStarted = true;
        this.contextL = context;
        final Activity activity = (Activity) context;
        final Context finalContext = context;
        final CameraBridgeViewBase.CvCameraViewListener2 self = this;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JCVs must be instantiated on a UI thread
                javaCameraView = new JavaCameraView(finalContext, cameraIndex);
                javaCameraView.setCameraIndex(cameraIndex);
                javaCameraView.setCvCameraViewListener(self);
                inited = true;
             }
        });
        Log.d(TAG, "Finish Initializing OpenCV");
    };

    public void enable () {

        if (!initStarted) throw new IllegalStateException("init() needs to be called before an OpenCVPipeline can be enabled!");
        // this is an absolute hack
        try {
            while (!inited) Thread.sleep(10);
        } catch (InterruptedException e) { return; }


        javaCameraView.enableView();

        final Activity activity = (Activity) contextL;
        final int resID = contextL.getResources().getIdentifier("RelativeLayout", "id", contextL.getPackageName());
        final View queuedView = javaCameraView;
        activity.runOnUiThread(new Runnable() {
           @Override
           public void run() {
              ViewGroup l = (ViewGroup) activity.findViewById(resID); //R.id.RelativeLayout);
              if (view != null) {
                  l.removeView(view);
              }
             l.addView(queuedView);
             view = queuedView;
          }
        });
        javaCameraView.setVisibility(View.VISIBLE);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.d(TAG, "onCameraViewStarted");
    }

    @Override
    public void onCameraViewStopped() {
        Log.d(TAG, "onCameraViewStopped");
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Log.d(TAG, "onCameraFrame");
        Mat rgba = new Mat();
        Mat gray = new Mat();
        switch (((Activity) contextL).getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_0:
                // this breaks horribly for some reason
                Core.rotate(inputFrame.rgba(), rgba, Core.ROTATE_90_CLOCKWISE);
                Core.rotate(inputFrame.gray(), gray, Core.ROTATE_90_CLOCKWISE);
                break;
            case Surface.ROTATION_90:
                rgba = inputFrame.rgba();
                gray = inputFrame.gray();
                break;
            case Surface.ROTATION_270:
                Core.rotate(inputFrame.rgba(), rgba, Core.ROTATE_180);
                Core.rotate(inputFrame.gray(), gray, Core.ROTATE_180);
                break;
        }
        Log.d(TAG, "onCameraFrame");
        return processFrame(rgba, gray);
    }

    protected abstract Mat processFrame(Mat rgba, Mat gray);


}
