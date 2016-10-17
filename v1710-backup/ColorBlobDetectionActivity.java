package org.opencv.samples.colorblobdetect;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
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

    private int iterator = 0;
    private int ball_ai_reached = 0;
    private int kickerator = 0;
    private int justkicked = 0;

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
                    Toast.makeText(getBaseContext(), "Broadcast granted", Toast.LENGTH_SHORT).show();
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
                            Toast.makeText(getBaseContext(), "serial is ready", Toast.LENGTH_SHORT).show();

                        } else {
                            Toast.makeText(getBaseContext(), "PORT NOT OPEN", Toast.LENGTH_SHORT).show();
                        }
                    } else {
                        Toast.makeText(getBaseContext(), "PORT IS NULL", Toast.LENGTH_SHORT).show();
                    }
                } else {
                    Toast.makeText(getBaseContext(), "PERMISSIONS NOT GRANTED", Toast.LENGTH_SHORT).show();
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
        sendButton2 = (Button) findViewById(R.id.buttonSend);

        //*** SERIAL ON CREATES END

    }

//    UsbSerialInterface.UsbReadCallback mCallback = new UsbSerialInterface.UsbReadCallback() {
//        //Defining a Callback which triggers whenever data is read.
//        @Override
//        public void onReceivedData(byte[] arg0) {
//            String data = null;
//            try {
//                data = new String(arg0, "UTF-8");
//                Toast.makeText(getBaseContext(), data, Toast.LENGTH_SHORT).show();
//
//            } catch (UnsupportedEncodingException e) {
//                e.printStackTrace();
//            }
//        }
//    };


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
        mRgba = inputFrame.rgba();
        if(justkicked == 1) {
            stopKicking();
        }
//        int ball_ai_reached = mAIsystem.ball_reached();
        mDetector.Configure(mRgba,ball_ai_reached);
//        Toast.makeText(ColorBlobDetectionActivity.this,"Sent string to arduino", Toast.LENGTH_LONG).show();
        int ballX = mDetector.ball_coordsX();
        int ballY = mDetector.ball_coordsY();
        double [] obstacles = mDetector.obstacles();
        int [] motorVels = mAIsystem.find_wheel_velocities(ballX,ballY,mRgba,obstacles); //call the AI system here and give it values from the VISION with each frame
//        if(iterator > 5) {
            String motA = Integer.toString(motorVels[0]);
            String motB = Integer.toString(motorVels[1]);
            String kick = "0";
            //sendMotorVel(motA,motB,kick,mRgba);
//            iterator = 0;
//        }
//        iterator++;
        return mRgba; // in the future u shouldnt have to show the frame to save fps and processing power/memory
    }

    //SERIAL HELPER FUNCTIONS

    public void onClickStart(View view) {
        Toast.makeText(getBaseContext(), "Connection starting...", Toast.LENGTH_SHORT).show();
        HashMap<String, UsbDevice> usbDevices = usbManager.getDeviceList();
        if (!usbDevices.isEmpty()) {
            Toast.makeText(getBaseContext(), "Found device", Toast.LENGTH_SHORT).show();
            boolean keep = true;
            for (Map.Entry<String, UsbDevice> entry : usbDevices.entrySet()) {
                device = entry.getValue();
                int deviceVID = device.getVendorId();
                String steve = String.format(Locale.ENGLISH, "Vendor ID: %d", deviceVID);
                Toast.makeText(getBaseContext(), steve, Toast.LENGTH_SHORT).show();
                if (deviceVID == 0x1A86 || deviceVID == 0x2341 || deviceVID == 6790)//Arduino Vendor ID
                {
                    Toast.makeText(getBaseContext(), "Found Arduino", Toast.LENGTH_SHORT).show();
                    PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
                    usbManager.requestPermission(device, pi);
                    keep = false;
                } else {
                    Toast.makeText(getBaseContext(), "No device found!", Toast.LENGTH_SHORT).show();
                    connection = null;
                    device = null;
                }

                if (!keep)
                    break;
            }
        }
        else {
            Toast.makeText(getBaseContext(), "No Devices", Toast.LENGTH_SHORT).show();
        }

    }

    public void sendMotorVel(String motA,String motB,String kick,Mat rgba){
        steve = motA + "," + motB + "," + kick + ","; //motA,motB,kick
        Imgproc.putText(rgba, steve, new Point(40, 240), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0,255,0), 2);
        if(serialPort != null) {
            serialPort.write(steve.getBytes());
        }
//        else {
//            Toast.makeText(getBaseContext(), "USB SERIAL ERROR! - NULL", Toast.LENGTH_SHORT).show();
//        }
    }

    public void onClickSend(View view) {
        Toast.makeText(getBaseContext(), "Sending Test String", Toast.LENGTH_SHORT).show();

        String string = "100,120,0";
        if(serialPort != null) {
            serialPort.write(string.getBytes());
            Toast.makeText(getBaseContext(), "string was sent!", Toast.LENGTH_SHORT).show();
        }
        else {
            Toast.makeText(getBaseContext(), "Serial port is null!", Toast.LENGTH_SHORT).show();
        }

//        if(serialPort !=null) {
//            Toast.makeText(getBaseContext(), "i am reading...", Toast.LENGTH_SHORT).show();
//            serialPort.read(mCallback);
//        }
    }

    public void stopKicking(){
        String string = "0,0,0";
        if(kickerator > 8) {
            if (serialPort != null) {
                serialPort.write(string.getBytes());
            }
            kickerator = 0;
            justkicked = 0;
        }
        kickerator++;
    }

    public void onClickSend2(View view) {
        String string = "0,0,1";
        if(serialPort != null) {
            serialPort.write(string.getBytes());
        }
        justkicked = 1;
    }

    public void onClickStop(View view) {
        serialPort.close();
    }


    //**** SERIAL HELPER FUNCTIONS END


}
