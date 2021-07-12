//Autonomous methods to be used in run programs

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Scanner;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

//for reading gyro

//@Config
public class AutonomousMethods extends LinearOpMode {

    public boolean RingIn = false;
    public boolean intakingRing = true;
    public double rings = 0;

    private static final String TAG = "Webcam: ";
    //How long we are to wait to be granted permission to use the camera before giving up. Here,we wait indefinitely
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    // State regarding our interaction with the camera
    public CameraManager cameraManager;
    public WebcamName cameraName;
    public Camera camera;
    public CameraCaptureSession cameraCaptureSession;
    //The queue into which all frames from the camera are placed as they become available. Frames which are not processed by the OpMode are automatically discarded. */
    public EvictingBlockingQueue<Bitmap> frameQueue;
    //State regarding where and how to save frames when the 'A' button is pressed.
    public int captureCounter = 0;
    public File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    //A utility object that indicates where the asynchronous callbacks from the camera infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}if you're curious): no knowledge of multi-threading is needed here.
    private Handler callbackHandler;
    public Bitmap bmp;

    public Hardware robot = new Hardware(true);
    private final double gearRatio = 1;
    private final double wheelDiameter = 3.78;
    public final double wheelCircumference = wheelDiameter*Math.PI;
    public final double encoderCounts = 384.5; //counts per one rotation of output shaft
    public final double wheelDiameterEnc = 1.37795;
    public final double wheelCircumferenceEnc = wheelDiameterEnc*Math.PI;
    public final double encoderCountsEnc = 8192; //counts per one rotation of output shaft
    public final double yOffset = 7.65;//7.75*
    public final double xOffset = 2.125;//2
    public final double yMiddleOffset = .1;
    public final double encCircY = Math.PI*yOffset*2;//Math.PI*Math.sqrt(Math.pow(yOffset, 2)+Math.pow(yMiddleOffset, 2))*2;
    public final double encCircX = Math.PI*xOffset*2;

    double countsPerRotation = encoderCounts*gearRatio;
    double distPerSquare = 24;//inches/square
    double robotLength = 18;
    double startOffset = 1.5;
    public static double startingAngle = -14;//-10 -14*
    public static double incrementRpm = 4.274;//6, 5, 4.274
    public static double incrementRpm2 = 1.0033;//6 1.0033
    public static double incrementAngle = 0;//-(2.0/24); //2.0*
    double angleMult = 1.0;//009;//8;//1.0093;//1.0071
    double yMult = 1.0;//35;//0877;
    double xMult = 1.0;//35;//106;

    public static double staticShooterRpm = 2100;//2190 *2140 2161 1840
    public static double staticShooterRpm2 = 1516;
    public double shooterRpm = staticShooterRpm;
    public double powerShotRpm = 1980;//1870
    public double powerShotRpm2 = 1980;//1925
    public double powerShotRpm3 = 1980;//1925
    public double shooterPower = (shooterRpm*28)/60.0;
    public double shootingAngle = -23;
    public double powerShotPower = (powerShotRpm*28)/60.0;
    public double powerShotPower2 = (powerShotRpm2*28)/60.0;
    public double powerShotPower3 = (powerShotRpm3*28)/60.0;
    public double p = 200;
    public double i = 0;
    public double d = 0;
    public double f = 14;

    double desiredHeading = 0;

    //public double kFactor = Math.sqrt(2);//1.09
    public double kFactor = 1.15;

    public double resetAngle = 0;
    public double resetAngle2 = 0;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtime2 = new ElapsedTime();
    public ElapsedTime runtime3 = new ElapsedTime();
    public ElapsedTime intaking = new ElapsedTime();
    public ElapsedTime shooting = new ElapsedTime();
    public double currentXPosition = robotLength/2;
    public double currentYPosition = robotLength/2;
    double max;

    boolean down = false;

    public void initializeRobot() {
        robot.initializeHardware(hardwareMap);
        resetAngle=getHeading();
        resetAngle2=getHeading2();

        //initializing camera
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        robot.encoders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        if (!OpenCVLoader.initDebug()) {
            error("Error: Cannot load OpenCV Library");
        } else {
            telemetry.addLine("Loaded OpenCV");
            telemetry.update();
        }

        try {
            openCamera();
            if (camera == null) return;

            //picture();
            takePic();
            if (cameraCaptureSession == null) return;
            //sleep(5000);
            telemetry.addLine(magic8());
            while(!isStarted()) {
                telemetry.addData("system", robot.imu.getSystemStatus());
                telemetry.addData("status", robot.imu.getCalibrationStatus());
                telemetry.addData("i", i);
                // Get the calibration data
                //BNO055IMU.CalibrationData calibrationData = robot.imu.readCalibrationData();

                // Save the calibration data to a file. You can choose whatever file
                // name you wish here, but you'll want to indicate the same file name
                // when you initialize the IMU in an opmode in which it is used. If you
                // have more than one IMU on your robot, you'll of course want to use
                // different configuration file names for each.
                //String filename = "AdafruitIMUCalibration.json";
                //File file = AppUtil.getInstance().getSettingsFile(filename);
                //ReadWriteFile.writeFile(file, calibrationData.serialize());
                //telemetry.log().add("saved to '%s'", filename);
                telemetry.update();
            }
            //shoot
            robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
            robot.shooter.setVelocity(shooterPower);
            waitForStart();
            stopAndResetEncoders();
            savePic();


        }
        finally {
            closeCamera();
        }

    }

    //Basic
    //moving forward distance (inch) with power [0, 1]
    public void forward(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//24
        double scaleBottom = 4;//4
        if (max>distance-scaleBottom){
            max = distance-scaleBottom;
        }
        double rampUpScale = 4;//4
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        runWithEncoders();
        int counts = (int) ((distance / wheelCircumference) * countsPerRotation);
        double initialAngle = desiredHeading;//getHeading();
        double distanceGone = 0;
        while (distanceGone<distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            if (intakingRing){
                ringIn();
            }
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            distanceGone = (distanceGoneBl+distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double anglePower = errorToPower(Math.abs(calcAngle), 5,0,.1, 0);
            double powerRaw = errorToPower(distance-distanceGone, max, scaleBottom, power, .05);

            if (distanceGoneFl<rampUpScale){
                setAllMotorsTo(errorToPower(distanceGone, rampUpScale, 0, power, .05));
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(powerRaw + anglePower, powerRaw + anglePower, powerRaw - anglePower, powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(powerRaw - anglePower, powerRaw - anglePower, powerRaw + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        //stopAndResetEncoders();
        //sleep(1000);
        //telemetry.addData("dist", wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation));
        //telemetry.update();
    }
    public void forward(double power, double squares, double inches, double max, double scaleBottom, double rampUpScale) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        if (max>distance-scaleBottom){
            max = distance-scaleBottom;
        }
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        runWithEncoders();
        int counts = (int) ((distance / wheelCircumference) * countsPerRotation);
        double initialAngle = desiredHeading;//getHeading();
        double distanceGone = 0;
        while (distanceGone<distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            if (intakingRing){
                ringIn();
            }
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            distanceGone = (distanceGoneBl+distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double anglePower = errorToPower(Math.abs(calcAngle), 5,0,.1, 0);
            double powerRaw = errorToPower(distance-distanceGone, max, scaleBottom, power, .05);

            if (distanceGoneFl<rampUpScale){
                setAllMotorsTo(errorToPower(distanceGone, rampUpScale, 0, power, .05));
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(powerRaw + anglePower, powerRaw + anglePower, powerRaw - anglePower, powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(powerRaw - anglePower, powerRaw - anglePower, powerRaw + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        //stopAndResetEncoders();
        //sleep(1000);
        //telemetry.addData("dist", wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation));
        //telemetry.update();
    }

    //moving backward distance (inch) with power [0, 1]
    public void backward(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//48
        double scaleBottom = 4;
        if (max>distance-scaleBottom){
            max = distance-scaleBottom;
        }
        double rampUpScale = 4;
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        runWithEncoders();
        int counts = (int) -((distance / wheelCircumference) * countsPerRotation);
        double initialAngle = desiredHeading;//getHeading();

        double distanceGone = 0;

        while (distanceGone>-distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            if (intakingRing){
                ringIn();
            }
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition())/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            distanceGone = (distanceGoneBl+distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double anglePower = errorToPower(Math.abs(calcAngle), 5, 0,.1, 0);
            double powerRaw = -errorToPower(distance -(-distanceGone), max, scaleBottom, power, .05);

            if (-distanceGone<rampUpScale){
                setAllMotorsTo(-errorToPower(-distanceGone, rampUpScale, 0,power, .05));
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(powerRaw + anglePower, powerRaw + anglePower, powerRaw - anglePower, powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(powerRaw - anglePower, powerRaw - anglePower, powerRaw + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(1000);
    }
    public void backward(double power, double squares, double inches, double max, double scaleBottom, double rampUpScale) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        if (max>distance-scaleBottom){
            max = distance-scaleBottom;
        }
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        runWithEncoders();
        int counts = (int) -((distance / wheelCircumference) * countsPerRotation);
        double initialAngle = desiredHeading;//getHeading();

        double distanceGone = 0;

        while (distanceGone>-distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            if (intakingRing){
                ringIn();
            }
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition())/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            distanceGone = (distanceGoneBl+distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double anglePower = errorToPower(Math.abs(calcAngle), 5, 0,.1, 0);
            double powerRaw = -errorToPower(distance -(-distanceGone), max, scaleBottom, power, .05);

            if (-distanceGone<rampUpScale){
                setAllMotorsTo(-errorToPower(-distanceGone, rampUpScale, 0,power, .05));
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(powerRaw + anglePower, powerRaw + anglePower, powerRaw - anglePower, powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(powerRaw - anglePower, powerRaw - anglePower, powerRaw + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(1000);
    }
    //strafing left distance (inch) with power [0, 1]
    public void strafeLeft(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//48
        double scaleBottom = 4;
        if (max>distance-scaleBottom){
            max = (distance-scaleBottom)*.75;
        }
        runWithEncoders();
        int counts = (int) ((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = 4;
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        double initialAngle = desiredHeading;//getHeading();

        double distanceGone = 0;

        while (distanceGone>-distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            double distanceGoneBl = wheelCircumference * ((robot.backLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneBr = wheelCircumference * ((robot.backRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFr = wheelCircumference * ((robot.frontRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);

            distanceGone = (-distanceGoneBl-distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double powerRaw = errorToPower(distance - (-distanceGone), max, scaleBottom, power, .05);
            double anglePower = errorToPower(Math.abs(calcAngle), 5,0,.1, 0);
            if (-distanceGoneFl<rampUpScale){
                powerRaw=errorToPower(-distanceGone, rampUpScale, 0, power, .05);
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(powerRaw + anglePower, -powerRaw + anglePower, -powerRaw - anglePower, powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(powerRaw - anglePower, -powerRaw - anglePower, -powerRaw + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(1000);
    }
    //strafing right distance (inch) with power [0, 1]
    public void strafeRight(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//48
        double scaleBottom = 4;
        if (max>distance-scaleBottom){
            max = distance-scaleBottom;
        }
        runWithEncoders();
        int counts = (int) -((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = 4;
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        double initialAngle = desiredHeading;//getHeading();

        double distanceGone = 0;

        while (distanceGone<distance) {
            double calcAngle = adjust(getHeading(), initialAngle);
            double distanceGoneBl = wheelCircumference * ((robot.backLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneBr = wheelCircumference * ((robot.backRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFr = wheelCircumference * ((robot.frontRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);

            distanceGone = (-distanceGoneBl-distanceGoneFr+distanceGoneBr+distanceGoneFl)/4;

            double anglePower = errorToPower(Math.abs(calcAngle), 5, 0,.1, 0);
            double powerRaw = errorToPower(distance - distanceGoneFl, max, scaleBottom, power, .05);
            if (distanceGoneFl<rampUpScale){
                powerRaw=errorToPower(distanceGoneFl, rampUpScale, 0, power, .05);
            }
            if(calcAngle<=0) {
                setPowerOfMotorsTo(-powerRaw + anglePower, powerRaw + anglePower, powerRaw - anglePower, -powerRaw - anglePower);
            }
            else if (calcAngle>0) {
                setPowerOfMotorsTo(-powerRaw - anglePower, powerRaw - anglePower, powerRaw + anglePower, -powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
    }
    //moving diagonal distance (inch) with power [0, 1]
    public void diagonalRight(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//48
        double scaleBottom = power * 6;
        if (max>distance+6+scaleBottom){
            max = (distance-scaleBottom)*.75;
        }
        runWithEncoders();
        int counts = (int) ((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = 12*power;
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        double initialAngle = getHeading();

        while (robot.frontLeftMotor.getCurrentPosition()<counts) {
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            double anglePower = 0;//errorToPower(Math.abs(initialAngle-getHeading()), 5,.1, 0);
            double powerRaw = errorToPower(distance-distanceGoneFl, max, scaleBottom, power, .1);

            if (distanceGoneFl<rampUpScale){
                setPowerOfMotorsTo(0, errorToPower(distanceGoneFl, rampUpScale, 0, power, .1),errorToPower(distanceGoneFl, rampUpScale, 0, power, .1), 0);
            }
            else {
                setPowerOfMotorsTo(0 - anglePower, powerRaw - anglePower, powerRaw + anglePower, 0 + anglePower);
            }
        }
        setAllMotorsTo(0);
        //stopAndResetEncoders();
        //sleep(250);
    }
    //moving diagonal distance (inch) with power [0, 1]
    public void diagonalLeft(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 24;//48
        double scaleBottom = power * 6;
        if (max>distance+6+scaleBottom){
            max = (distance-scaleBottom)*.75;
        }
        runWithEncoders();
        int counts = (int) -((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = 12*power;
        if(distance-rampUpScale-scaleBottom<max){
            rampUpScale = distance-max-scaleBottom;
        }
        double initialAngle = getHeading();

        while (robot.frontLeftMotor.getCurrentPosition()<counts) {
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);

            double anglePower = 0;//errorToPower(Math.abs(initialAngle-getHeading()), 5,.1, 0);
            double powerRaw = errorToPower(distance-distanceGoneFr, max, 6, power, .1);

            if (distanceGoneFr<rampUpScale){
                setPowerOfMotorsTo(errorToPower(distanceGoneFr, rampUpScale, 0,power, .1), 0, 0, errorToPower(distanceGoneFr, rampUpScale,0, power, .1));
            }
            else {
                setPowerOfMotorsTo(powerRaw - anglePower, 0 - anglePower, 0 + anglePower, powerRaw + anglePower);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(250);
    }
    //going to any angle
    public void toAngle(double angle, double speed){
        runWithEncoders();
        double calcAngle = adjust(angle, getHeading());
        double scale = 90*speed;
        double scaleBottom = 6;
        boolean right = calcAngle>0;
        double power;
        if (right){
            while (calcAngle>0){
                power = errorToPower(calcAngle, scale, scaleBottom, speed, .05);
                setPowerOfMotorsTo(power, power , -power , -power );
                calcAngle = adjust(angle, getHeading());
            }
        }
        else {
            while (calcAngle<0){
                power = errorToPower(Math.abs(calcAngle), scale, scaleBottom, speed, .05);
                setPowerOfMotorsTo(-power, -power , power , power );
                calcAngle = adjust(angle, getHeading());
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        desiredHeading = angle;
    }
    //returns angling power
    public double toAngle2(double angle, double speed){
        runWithEncoders();
        double calcAngle = adjust(angle, getHeading());
        double scale = 45*speed;
        double scaleBottom = 2;
        boolean right = calcAngle>0;
        double power;
        if (right){
            if (calcAngle>0){
                power = errorToPower(calcAngle, scale, scaleBottom, speed, .05);
                return power;
            }
        }
        else {
            if (calcAngle<0){
                power = errorToPower(-calcAngle, scale, scaleBottom, speed, .05);
                return -power;
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        desiredHeading = angle;
        return  0;
    }
    //goes to a position and angle on the field
    public void goToPosition(double angle, double currentX, double currentY, double x, double y) {
        runWithEncoders();
        double deltaY1 = y-currentY;

        double deltaX1 = x-currentX;

        double thetaX = -Math.PI/4;
        double thetaY = Math.PI/4;

        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        //double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngleX =  thetaX-gyroAngle;
        double calculationAngleY =  thetaY-gyroAngle;

        double deltaY2 = Math.sin(calculationAngleY) * deltaY1 + Math.sin(calculationAngleX)*deltaX1;
        double deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX)*deltaX1;

        deltaY2+=deltaX2;
        deltaX2+=deltaX2;

        // setting scale factor to the biggest value
        // dividing by this when setting powers will make sure that speeds are in the format 0-1
        // will also insure speeds are proportional to distance needed to travel in each direction so robot can move at angle theta
        double scaleFactor = deltaX2;
        if (deltaY2 > deltaX2) {
            scaleFactor = deltaY2;
        }

        int countsX = (int) ((deltaX2/wheelCircumference)*countsPerRotation);
        int countsY = (int) ((deltaY2/wheelCircumference)*countsPerRotation);
        setTargetPosition(robot.backLeftMotor.getCurrentPosition()+countsY, robot.frontLeftMotor.getCurrentPosition()+countsX, robot.backRightMotor.getCurrentPosition()+countsX, robot.frontRightMotor.getCurrentPosition()+countsY);
        runToPosition();
        while(robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()){
            double distanceGoneX = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneY = (wheelCircumference) * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double powerX = errorToPower(deltaX2-distanceGoneX, 24, 12, deltaX2 / scaleFactor, .05);
            double powerY = errorToPower(deltaY2-distanceGoneY, 24, 12, deltaY2 / scaleFactor, .05);
            setPowerOfMotorsTo(powerY, powerX, powerX, powerY);
            if (deltaX2-distanceGoneX<=.25){
                break;
            }
        }

        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        toAngle(angle, 1);
        currentYPosition=y;
        currentXPosition=x;
    }
    //goes in any direction
    public void translate(double angle, double x, double y){
        goToPosition(angle, 0, 0, x, y);
    }

    //Tools
    //sets the power of motors to inputted powers
    public void setPowerOfMotorsTo(double bl, double fl, double br, double fr) {
        //set diagonal wheels to prevent jerking
        robot.backLeftMotor.setVelocity((435*bl*384.5)/60);
        robot.frontRightMotor.setVelocity((435*fr*384.5)/60);
        robot.frontLeftMotor.setVelocity((435*fl*384.5)/60);
        robot.backRightMotor.setVelocity((435*br*384.5)/60);
    }
    //sets all motors to the same velocity
    public void setAllMotorsTo(double power) {
        robot.backLeftMotor.setVelocity((435*power*384.5)/60);
        robot.frontRightMotor.setVelocity((435*power*384.5)/60);
        robot.frontLeftMotor.setVelocity((435*power*384.5)/60);
        robot.backRightMotor.setVelocity((435*power*384.5)/60);

    }
    //resets all wheel encoders to 0
    public void stopAndResetEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //set mode to run with encoders
    public void runWithEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //set mode to run without encoders
    public void runWithoutEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //set mode to run to Position
    public void runToPosition() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //set target position
    public void setTargetPosition(int bl, int fl, int br, int fr) {
        robot.backLeftMotor.setTargetPosition(bl);
        robot.backRightMotor.setTargetPosition(br);
        robot.frontRightMotor.setTargetPosition(fr);
        robot.frontLeftMotor.setTargetPosition(fl);
    }
    //takes error and finds appropriate speed
    public double errorToPower(double error, double scale, double scaleBottom, double maxValue, double minValue){
        //proportional
        if(error<0){
            error=0;
        }
        double speed = minValue;
        if(error>scaleBottom) {
            speed = (((error - scaleBottom) / scale) * (maxValue - minValue) + minValue);
        }
        if(speed>maxValue){
            speed=maxValue;
        }
        if(error<0){
            speed = 0;
        }
        return speed;
    }
    //adjusting heading on a scale of [-180,180] from a scale of [0, 360]
    public double adjust(double angle) {
        if (angle>180){
            angle-=360;
        }
        if (angle<-180){
            angle+=360;
        }
        return angle;
    }
    //adjusting heading on a scale of [-180,180] by zeroing out original
    public double adjust(double angle, double original){
        double adjusted = angle-original;
        if (adjusted>180){
            adjusted = adjusted-360;
        }
        else if (adjusted<-180){
            adjusted = adjusted+360;
        }
        return adjusted;
    }
    //gets angle in degrees from deadwheels
    public double getHeading2() {
        double leftWheelInches = (robot.intake2.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*yMult;
        double rightWheelInches = (robot.encoders.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*xMult;
        double diff = (leftWheelInches-rightWheelInches)/2;
        double theta = (Math.PI/2)-Math.atan(yOffset/yMiddleOffset);
        //telemetry.addData("t", theta);
        double diffAdjutsed = diff*Math.cos(theta);
        double diffAngle = ((diffAdjutsed)/encCircY)*360*angleMult;
        if((int)(Math.abs(diffAngle)/180)%2==0 && Math.abs(diffAngle)>180){
            return diffAngle%180;
        }
        else if((int)(diffAngle/180)%2==1){
            return -180+(Math.abs(diffAngle)%180);
        }
        else if((int)(diffAngle/180)%2==-1){
            return 180+(diffAngle%180);
        }
        return diffAngle;
    }
    public double getHeadingComp() {
        double leftWheelInches = (robot.intake2.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*yMult;
        double rightWheelInches = (robot.encoders.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*xMult;
        double diff = (leftWheelInches-rightWheelInches)/2;
        double diffAngle = ((diff)/encCircY)*360*angleMult;
        if((int)(Math.abs(diffAngle)/180)%2==0 && Math.abs(diffAngle)>180){
            return diffAngle%180;
        }
        else if((int)(diffAngle/180)%2==1){
            return -180+(Math.abs(diffAngle)%180);
        }
        else if((int)(diffAngle/180)%2==-1){
            return 180+(diffAngle%180);
        }
        return diffAngle;
    }
    //returns raw Heading
    public double getHeadingRawComp() {
        double leftWheelInches = (robot.intake2.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*yMult;
        double rightWheelInches = (robot.encoders.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*xMult;
        double diff = (leftWheelInches-rightWheelInches);
        double diffAngle = ((diff/2)/encCircY)*360*angleMult;
        return diffAngle;
    }
    public double getHeadingRaw2() {
        double leftWheelInches = (robot.intake2.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*yMult;
        double rightWheelInches = (robot.encoders.getCurrentPosition()/encoderCountsEnc)*Math.PI*wheelDiameterEnc*xMult;
        double diff = (leftWheelInches-rightWheelInches)/2;
        double theta = (Math.PI/2)-Math.atan(yOffset/yMiddleOffset);
        //telemetry.addData("t", theta);
        double diffAdjutsed = diff*Math.cos(theta);
        double diffAngle = ((diffAdjutsed)/encCircY)*360*angleMult;
        return diffAngle;
    }
    //gets the angle in degrees from imu (control hub)
    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle-resetAngle; // left [0,-180] right[0,180]
    }
    //gets the angle in degrees from imu 2 (expansion hub)
    public double getHeading3() {
        Orientation angles = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle-resetAngle2; // left [0,-180] right[0,180]
    }
    //plays back teleop code written to file
    public void playBack(String filename){
        ArrayList<String> lines = new ArrayList<String>();
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            Scanner reader = new Scanner(file);
            while (reader.hasNextLine()) {
                String data = reader.nextLine();
                lines.add(data);
            }
            reader.close();
            telemetry.addLine(lines.toString());
        } catch (FileNotFoundException e) {
            telemetry.addLine("An error occurred.");
            e.printStackTrace();
        }

        runtime3.reset();

        LinkedHashMap<Double, Double> speedsBl= stringToMap(lines.get(0));
        LinkedHashMap<Double, Double> speedsBr= stringToMap(lines.get(1));
        LinkedHashMap<Double, Double> speedsFr= stringToMap(lines.get(2));
        LinkedHashMap<Double, Double> speedsFl= stringToMap(lines.get(3));
        LinkedHashMap<Double, Double> speedsIntake= stringToMap(lines.get(4));
        LinkedHashMap<Double, Double> speedsFlywheel= stringToMap(lines.get(5));
        ArrayList<Double> drop= stringToArrayList(lines.get(6));
        ArrayList<Double> pickUp= stringToArrayList(lines.get(7));
        ArrayList<Double> indexer= stringToArrayList(lines.get(8));
        LinkedHashMap<Double, Double> toAngle= stringToMap(lines.get(9));
        telemetry.addLine("complete");
        double stoptime = Double.parseDouble(lines.get(10));

        double currentTime = runtime3.seconds();
        while (stoptime>currentTime){
                currentTime = runtime3.seconds();
                double closest = Double.MAX_VALUE;
                for (double i : speedsBl.keySet()){
                    if (Math.abs(currentTime-i)<Math.abs(currentTime-closest)){
                        closest = i;
                    }
                }
                double calcTime = closest;
                telemetry.addData("playback", currentTime);
                telemetry.addData("speed", speedsBl.get(currentTime));
                telemetry.update();

                robot.backLeftMotor.setVelocity(speedsBl.get(calcTime));
                robot.frontRightMotor.setVelocity(speedsFr.get(calcTime));
                robot.frontLeftMotor.setVelocity(speedsFl.get(calcTime));
                robot.backRightMotor.setVelocity(speedsBr.get(calcTime));
                robot.intake.setPower(speedsIntake.get(calcTime));
                robot.intake2.setPower(-speedsIntake.get(calcTime));
                robot.shooter.setVelocity(speedsFlywheel.get(calcTime));
                if (indexer.size()>0) {
                    if (indexer.get(0) <= currentTime) {
                        shootRings(3);
                        indexer.remove(0);
                    }
                }
                if (pickUp.size()>0) {
                    if (pickUp.get(0) <= currentTime) {
                        pickUpClaw(0);
                        pickUp.remove(0);
                    }
                }
                if(drop.size()>0) {
                    if (drop.get(0) <= currentTime) {
                        dropClaw();
                        drop.remove(0);
                    }
                }
                if(toAngle.size()>0) {
                    for (double t :toAngle.keySet())
                        if (t <= currentTime) {
                            toAngle(toAngle.get(t), 1);
                            toAngle.remove(t);
                        }
                }
            }
        setAllMotorsTo(0);
        runtime3.reset();
    }
    //converts strings stores in file
    public LinkedHashMap stringToMap(String s) {
        //new HashMap object
        LinkedHashMap<Double, Double> convert = new LinkedHashMap<Double, Double>();

        //split the String by a comma
        String parts[] = s.split(", |=|\\{|\\}");

        //iterate the parts and add them to a map
        for (String part : parts) {

            //split the employee data by : to get id and name
            String empdata[] = part.split("=");

            Double timeStamp = Double.parseDouble(empdata[0].trim());
            Double speed = Double.parseDouble(empdata[1].trim());

            //add to map
            convert.put(timeStamp, speed);

        }
        return convert;
    }
    public ArrayList stringToArrayList(String s) {
        //new HashMap object
        ArrayList<Double> convert = new ArrayList<Double>();
        //split the String by a comma
        String parts[] = s.split(", |\\]|\\[");
        //iterate the parts and add them to a map
        for (String part : parts) {
            Double timeStamp = Double.parseDouble(part.trim());
            convert.add(timeStamp);
        }
        return convert;
    }
    //formats hash map for user interface
    public String hashToString(LinkedHashMap<String, Integer> hash, int index){
        String string = "";
        int num = 0;
        for( String i : hash.keySet()){
            string+="\n"+i+": "+hash.get(i);
            if(index==num){
                string+="<";
            }
            num++;
        }
        return string;
    }
    //user interface

    //ultimate goal specific
    //sets power of intake
    public void setIntakePower(double power){
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setPower(power);
        robot.intake2.setPower(-power);
    }
    //set indexer position
    public void controlIndexServo(double position){
        robot.IndexerServo.setPosition(position);
        sleep(100);

    }
    //set blocker position
    public void controlBlocker(double position) {
        robot.Blocker.setPosition(position);
    }
    //Indexer
    public void shootRings(double x){
        for (i=0; i<x; i++) {
            controlIndexServo(.8);
            controlIndexServo(1);
            if(x>1) {
                //sleep(250);//400
            }
        }
    }
    //set claw position
    public void controlClawServo(double position){
        robot.clawServo.setPosition(position);
    }
    //arm position
    public void controlArmServo(double position){
        robot.armServo.setPosition(position);
    }
    //flywheel
    public void setShooterPower(double velocity){
        robot.shooter.setVelocity(velocity);
    }
    //shoots all three rings at the same angle
    public void shoot(double a, double power){
        setShooterPower(power);
        toAngle(a, 1);
        setIntakePower(1);
        shootRings(3);
        setIntakePower(0);
    }
    //shoots all three rings at 3 different angles
    public void powerShot(double a1, double a2, double a3, double power, double powerE){
        setIntakePower(1);
        setShooterPower(power);
        toAngle(a1,1);//.3
        shootRings(1);

        setShooterPower(powerShotPower2);
        toAngle(a2, 1);
        shootRings(1);

        setShooterPower(powerShotPower3);
        toAngle(a3, 1);
        shootRings(1);

        setShooterPower(powerE);
        setIntakePower(0);
    }
    //dropping wobble goal
    public void dropWobbleGoal(){
        controlArmServo(1);//drop
        sleep(1000);
        controlClawServo(.7);//open
        controlArmServo(.25);//up
        sleep(500);
    }
    //picking up wobble goal
    public void pickUpWobbleGoal(double distanceLeft) {
        strafeLeft(.5, 0, distanceLeft);
        controlArmServo(1);//drop
        sleep(1000);
        controlClawServo(.25);//close
        sleep(500);
        controlArmServo(.25);//up
    }
    //dropping wobble goal
    public void dropClaw(){
        controlArmServo(1);//drop
        sleep(1000);
        controlClawServo(.7);//open
        sleep(500);
    }
    //picking up wobble goal
    public void pickUpClaw(double distanceLeft) {
        strafeLeft(.5, 0, distanceLeft);
        controlClawServo(.25);//close
        sleep(500);
        controlArmServo(.25);//up
    }
    //magic stuff
    public String magic8() {
        int magic8 = (int)(Math.random()*(19)+1);
        switch(magic8){
            case 1:
                return "As I see it, yes.";
            case 2:
                return "Ask again later.";
            case 3:
                return "Better not tell you now";
            case 4:
                return "Cannot predict now.";
            case 5:
                return "Concentrate and ask again.";
            case 6:
                return "Don’t count on it.";
            case 7:
                return "It is certain.";
            case 8:
                return "It is decidedly so.";
            case 9:
                return "Most likely.";
            case 10:
                return "My reply is no.";
            case 11:
                return "My sources say no.";
            case 12:
                return "Outlook not so good.";
            case 13:
                return "Outlook good.";
            case 14:
                return "Reply hazy, try again.";
            case 15:
                return "Signs point to yes.";
            case 16:
                return "Very doubtful.";
            case 17:
                return "Without a doubt.";
            case 18:
                return "Yes.";
            case 19:
                return "You may rely on it.";
            case 20:
                return "Yes – definitely.";
        }
        return "error";
    }
    public void updateShootingParameters(){
        //2190
        //2200
        //2250
        //2280
        //2380
        //2435
        double curveAngle = (startingAngle+incrementAngle*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((36-currentXPosition),2))-60));
        shootingAngle = Math.toDegrees(Math.atan((36-currentXPosition)/(144-(currentYPosition))))+curveAngle;//-35 44.1
        shooterRpm = (staticShooterRpm+incrementRpm*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((36-currentXPosition),2))-60));
        shooterPower = (shooterRpm*28)/60.0;
        setShooterPower(shooterPower);
    }
    public void updateShootingParameters2(){
        double curveAngle = (startingAngle+incrementAngle*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2))-60));
        shootingAngle = Math.toDegrees(Math.atan((108-currentXPosition)/(144-(currentYPosition))))+curveAngle;
        shooterRpm = (staticShooterRpm+incrementRpm*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2))-60));
        shooterPower = (shooterRpm*28)/60.0;
        setShooterPower(shooterPower);
    }
    public void updateShootingParameters3(){
        double curveAngle = (startingAngle+incrementAngle*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2))-60));
        shootingAngle = Math.toDegrees(Math.atan((108-currentXPosition)/(144-(currentYPosition))))+curveAngle;
        shooterRpm = (staticShooterRpm2*Math.pow(incrementRpm2,(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2)))));
        shooterPower = (shooterRpm*28)/60.0;
        setShooterPower(shooterPower);
    }
    public void updateShootingParameters4(){
        double curveAngle = (startingAngle+incrementAngle*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((36-currentXPosition),2))-60));
        shootingAngle = Math.toDegrees(Math.atan((36-currentXPosition)/(144-(currentYPosition))))+curveAngle;//-35 44.1
        shooterRpm = (staticShooterRpm2*Math.pow(incrementRpm2,(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((36-currentXPosition),2)))));
        shooterPower = (shooterRpm*28)/60.0;
        setShooterPower(shooterPower);
    }
    public void updateShootingParameters6(double powerX){
        double curveAngle = (startingAngle+incrementAngle*(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2))-72));
        shootingAngle = Math.toDegrees(Math.atan((powerX-currentXPosition)/(144-(currentYPosition))))+curveAngle;
        shooterRpm = (staticShooterRpm2*Math.pow(incrementRpm2,(Math.sqrt(Math.pow((144-(currentYPosition)),2)+Math.pow((108-currentXPosition),2)))));
        shooterPower = (shooterRpm*28)/60.0;
        setShooterPower(shooterPower);
    }

    //Vision
    public void savePic(){
        bmp = frameQueue.poll();
        frameQueue.clear();

    }
    private void initializeFrameQueue(int capacity) {
        // The frame queue will automatically throw away bitmap frames if they are not processed quickly by the OpMode. This avoids a buildup of frames in memory
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }
    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            //error("camera not found or permission to use not granted: %s", cameraName);
        }
    }
    private void takePic() {
        if (cameraCaptureSession != null) return; // be idempotent

        //YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only* image format supported by a camera
        final int imageFormat = ImageFormat.YUY2;

        //Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            //error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        //Some of the logic below runs asynchronously on other threads. Use of the synchronizer here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            //Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(CameraCaptureSession session) {
                    try {
                        //The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame( CameraCaptureSession session, CameraCaptureRequest request, CameraFrame cameraFrame) {
                                        //A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it for the duration of the callback. So we copy here manually.
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        //error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            //error("exception starting camera");
            synchronizer.finish(null);
        }

        //Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        //Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }
    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }
    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }
    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
    public int findNumRings(Bitmap bitmap) {

        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));

        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                //telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            //error("exception saving %s", file.getName());
        }
        Mat input = Imgcodecs.imread("/sdcard/FIRST/data/webcam-frame-0.jpg");

        int rings = 0;
        int h = input.height();
        int w = input.width();
        Imgproc.rectangle(input, new Point(0,0), new Point(640,375), new Scalar(255,0,0), -1);
        Imgproc.cvtColor(input,input,Imgproc.COLOR_BGR2HSV);
        Core.inRange(input, new Scalar(5, 75, 75),new Scalar(17, 255, 255),input);//(input, new Scalar(150, 0, 0),new Scalar(225, 360, 360),input)  new Scalar(5, 100, 0),new Scalar(35, 360, 360) (input,new Scalar(0,75,0),new Scalar(35,360,360),input)
        int pixels = Core.countNonZero(input);

//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        telemetry.addData("pixels", pixels);
//        for (int i = 0; i < contours.size(); i++) {
//            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
//            Imgproc.drawContours(input, contours, i, color, 2, Core.LINE_8, hierarchy, 0, new Point());
//        }

        if(pixels > 3500){
            rings = 4;

        }else if (pixels > 1000){
            rings = 1;
        }else{
            rings = 0;
        }
        return rings;
    }
    public void picture(){
        try {
            openCamera();
            if (camera == null) return;
            takePic();
            if (cameraCaptureSession == null) return;
            savePic();
        }
        finally {
            closeCamera();
        }
    }
    public void ringIn(){
        if(!RingIn&&robot.distance.getDistance(DistanceUnit.CM)<5){
            RingIn = true;
        }
        if(RingIn&&robot.distance.getDistance(DistanceUnit.CM)>5){
            RingIn= false;
            if(intakingRing) {
                rings++;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}

