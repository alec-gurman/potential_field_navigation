package org.opencv.samples.colorblobdetect;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import android.graphics.Color;
import android.hardware.Camera;
import android.os.Bundle;
import android.view.View;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;

//SERIAL LIBRARYS

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import java.io.UnsupportedEncodingException;
import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

//**** SERIAL LIBRARYS END


public class ColorBlobDetectionActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "OCVSample::Activity";

    private Mat                  mRgba;
    private ColorBlobDetector    mDetector;
    private aiSystem             mAIsystem;
    private CameraBridgeViewBase mOpenCvCameraView;
    String steve;
    private Boolean ball_found = false;

    private int stopDriving = 0;
    private int ball_ai_reached = 0;
    private int stop = 0;
    private int count = 0;

    //SERIAL PARAMETERS

    public final String ACTION_USB_PERMISSION = "org.opencv.samples.colorblobdetect.USB_PERMISSION";
    Button startButton, sendButton, sendButton2;
    UsbManager usbManager;
    UsbDevice device;
    UsbSerialDevice serialPort;
    UsbDeviceConnection connection;

    //*** SERIAL PARAMETERS END

    //SERIAL CONNECTION FUNCTIONS

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() { //Broadcast Receiver to automatically start and stop the Serial connection.
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(ACTION_USB_PERMISSION)) {
//                Toast.makeText(getBaseContext(), "Got broadcast", Toast.LENGTH_SHORT).show();
                boolean granted = intent.getExtras().getBoolean(UsbManager.EXTRA_PERMISSION_GRANTED);
                if (granted) {
                    //Toast.makeText(getBaseContext(), "Broadcast granted", Toast.LENGTH_SHORT).show();
                    connection = usbManager.openDevice(device);
                    serialPort = UsbSerialDevice.createUsbSerialDevice(device, connection);
                    if (serialPort != null) {
                        if (serialPort.open()) { //Set Serial Connection Parameters.
//                            Toast.makeText(getBaseContext(), "setting up serial", Toast.LENGTH_SHORT).show();
                            serialPort.setBaudRate(9600);
                            serialPort.setDataBits(UsbSerialInterface.DATA_BITS_8);
                            serialPort.setStopBits(UsbSerialInterface.STOP_BITS_1);
                            serialPort.setParity(UsbSerialInterface.PARITY_NONE);
                            serialPort.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
                            //Toast.makeText(getBaseContext(), "serial is ready", Toast.LENGTH_SHORT).show();

                        } else {
                            //Toast.makeText(getBaseContext(), "PORT NOT OPEN", Toast.LENGTH_SHORT).show();
                        }
                    } else {
                        //Toast.makeText(getBaseContext(), "PORT IS NULL", Toast.LENGTH_SHORT).show();
                    }
                } else {
                    //Toast.makeText(getBaseContext(), "PERMISSIONS NOT GRANTED", Toast.LENGTH_SHORT).show();
                }
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
                onClickStart(startButton);
            }
        }

        ;
    };

    //****** SERIAL CONNECTION FUNCTUONS

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public ColorBlobDetectionActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        //SERIAL ON CREATES

        usbManager = (UsbManager) getSystemService(this.USB_SERVICE);
        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);
        startButton = (Button) findViewById(R.id.buttonStart);
        sendButton = (Button) findViewById(R.id.buttonSend);
        sendButton2 = (Button) findViewById(R.id.buttonSend2);
        sendButton.setRotation(90);
        startButton.setRotation(90);
        sendButton2.setRotation(90);

        //*** SERIAL ON CREATES END

    }


    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mAIsystem = new aiSystem();
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public boolean onTouch(View v, MotionEvent event) {
        return false;
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

        //LOAD THE FRAME
        mRgba = inputFrame.rgba();

        //LOAD THE VISION SYTEM
        ball_ai_reached = mAIsystem.ball_reached();
        mDetector.Configure(mRgba, ball_ai_reached);

        //MOTOR DEFAULTS
        String motA = "0";
        String motB = "0";
        String kick = "0";
        String dribble = "1";

        //OBJECT LOCATIONS
        int ballX = mDetector.ball_coordsX();
        int ballY = mDetector.ball_coordsY();
        int goalX = mDetector.goal_coordsX();
        int goalY = mDetector.goal_coordsY();
        int blue_goalX = mDetector.blue_goal_coordsX();
        int blue_goalY = mDetector.blue_goal_coordsY();
        double[] obstacles = mDetector.obstacles();

        //CREATE A STOPPABLE LOOP

        if(stop == 0) {

            //FIND BALL ON PROGRAM START (once true, not used again)
            if (!ball_found) {
                boolean get_ball_stat = mAIsystem.find_ball(ballX, ballY, mRgba, obstacles);
                if (get_ball_stat && count > 30) {
                    ball_found = true;
                    count = 0;
                }
                count++;
            }

            //START MAIN LOOP FOR WHEN BALL HAS BEEN FOUND
            if (ball_found) {
                mAIsystem.state_machine(ballX,ballY,goalX,goalY,mRgba,obstacles);
            }

            //GET THE MOTOR VELOCITIES FROM AI SYSTEM
            int[] motorVels = mAIsystem.wheel_velocities(); //call the AI system here and give it values from the VISION with each frame

            //GET THE KICK STATE FROM AI SYSTEM
            int kickball = mAIsystem.kick_ball();
            if (kickball == 1) {
                stopDriving = 1;
            }

            //IF NOT KICKING PASS THROUGH MOTOR VELOCITIES
            if (stopDriving == 0) {
                motA = Integer.toString(motorVels[0]);
                motB = Integer.toString(motorVels[1]);
            }

            //GET THE DRIBBLE STATE FROM AI SYSTEM
            int dribbleball = mAIsystem.dribble_ball();

            //GIVE THE SERIAL THE MOTOR CONTROL COMMANDS
            kick = Integer.toString(kickball);
            dribble = Integer.toString(dribbleball);
            sendMotorVel(motA, motB, kick, dribble);
        }
        else {
            sendMotorVel(motA, motB, kick, dribble);
        }

        //RETURN THE FRAME
        return mRgba;

    }

    //PROCESSING FUNCTIONS
    public void sendMotorVel(String motA,String motB,String kick,String dribble){
        steve = motA + "," + motB + "," + kick + "," + dribble + ","; //motA,motB,kick
        if(serialPort != null) {
            serialPort.write(steve.getBytes());
        }
    }

    //SERIAL HELPER FUNCTIONS

    public void onClickStart(View view) {
        Runnable runnable = new Runnable() { //CREATE A NEW THREAD FOR SERIAL TO RUN IN
            public void run() {
                //Toast.makeText(getBaseContext(), "Connection starting...", Toast.LENGTH_SHORT).show();
                HashMap<String, UsbDevice> usbDevices = usbManager.getDeviceList();
                if (!usbDevices.isEmpty()) {
                    //Toast.makeText(getBaseContext(), "Found device", Toast.LENGTH_SHORT).show();
                    boolean keep = true;
                    for (Map.Entry<String, UsbDevice> entry : usbDevices.entrySet()) {
                        device = entry.getValue();
                        int deviceVID = device.getVendorId();
                        //String steve = String.format(Locale.ENGLISH, "Vendor ID: %d", deviceVID);
                        //Toast.makeText(getBaseContext(), steve, Toast.LENGTH_SHORT).show();
                        if (deviceVID == 0x1A86 || deviceVID == 0x2341 || deviceVID == 6790)//Arduino Vendor ID
                        {
                            //Toast.makeText(getBaseContext(), "Connected!", Toast.LENGTH_SHORT).show();
                            PendingIntent pi = PendingIntent.getBroadcast(getBaseContext(), 0, new Intent(ACTION_USB_PERMISSION), 0);
                            usbManager.requestPermission(device, pi);
                            keep = false;
                        } else {
                            //Toast.makeText(getBaseContext(), "No device found!", Toast.LENGTH_SHORT).show();
                            connection = null;
                            device = null;
                        }

                        if (!keep)
                            break;
                    }
                } else {
                    //Toast.makeText(getBaseContext(), "No Devices", Toast.LENGTH_SHORT).show();
                }
            }
        };
        Thread mythread = new Thread(runnable);
        mythread.start();

    }

    //ON SCREEN BUTTON FUNCTIONS

    public void onClickSend(View view) {
        stop = 1;
        String motA = "0";
        String motB = "0";
        String kick = "0";
        String dribble = "0";
        sendMotorVel(motA,motB,kick,dribble);
    } //drives foward

    public void onClickSend2(View view) {
        stopDriving = 0;
        stop = 0;
        ball_found = false;
    } //resets a bunch of variables to continue testing. Kicker reset not currently implemented


}
