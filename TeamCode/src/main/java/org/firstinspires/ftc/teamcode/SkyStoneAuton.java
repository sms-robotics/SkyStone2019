package org.firstinspires.ftc.teamcode;

//import android.graphics.Color;
//import java.lang.reflect.Array;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import java.util.Locale;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
//import org.firstinspires.ftc.teamcode.SmsJSON;

public class SkyStoneAuton extends LinearOpMode {
    private NerdBOT myNerdBOT ;

    private  double speed = 0.8;
    boolean debugFlag = true;
    
    private static final int TIME_DRIVE = 1;
    private static final int ENCODER_DRIVE_OLD = 2;
    private static final int ENCODER_DRIVE = 92;
    private static final int NERD_DRIVE = 42;
    private static final int TURN = 3;
    private static final int FOUNDATION = 4;
    private static final int CLAW = 5;
    private static final int EXPAND_ARM = 6;
    private static final int UNLATCH_ARM = 9;
    private static final int IDENTIFY_SKYSTONE = 55;
    private static final int DONE = 0;
    private static final int OPEN = 0;
    private static final int CLOSE = 1;
    
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY = "AXKT92L/////AAAAGS+ZuWWofUShhb1MB4+Zbic/fnrONEJsEKNCY4RE1F7X8GaFg4EQYqHF4GMlj35ZJdzZ/LQlnXVV2WlhqhHR5IDlScqWtishwl2yPBRzCXAWYP5MCphLOigzPcshkggMYEKQWxwlhvoc2lsN+54KexfxlI0ss9cMq+unSD8ZZ5Of5OuY0lX7DWAEEPh1KsdeEU7EkCGP96f5TQI518LsriyHeg73KgDLCcGd0yBUSuGWTTV3o/cTRziN+Ac1sYNzw1sEddiBS2TfCdjRlY2qMmgyAMARQhYEbcqbzGz8jcDNOsX/gS/knjAZ9UYPZl7mYFyq3Acg3089CTN+EXkFEMJysFU0XQW9P2YzICsAivi5";

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private double currentAngle = 0;

    private static final float mmPerInch        = 25.4f;
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    double stone = 0.0d;
    double turnLimit1 = 0.2;
    double turnLimit2 = 0.1;
    SkyStoneHardware robot = new SkyStoneHardware();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
//    static final double P_TURN_COEFF = 0.15;     // Larger is more responsive, but also less stable
    //DcMotor leftMotor = null;
//DcMotor rightMotor = null;
//BNO055IMU imu = null;                    // Additional Gyro device
    int CryptoBoxOffset = 0;
//VuforiaLocalizer RobotVision; // The Vuforia application.
//VuforiaTrackables relicTrackables; // The Relic image resource file.
//VuforiaTrackable relicTemplate; // The image referenced in the resource file.

    @Override
    public void runOpMode() {
        myNerdBOT = new NerdBOT(this);

        myNerdBOT.setDebug(debugFlag);

            //Initialize Hardware
        myNerdBOT.initializeHardware();
        //Initialize the PID Calculators
        myNerdBOT.initializeXPIDCalculator(0.0025, 0.005, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.005, 0.0,debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.3, 0.3, 0.0,debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.015, 0.000, 1.4,debugFlag);//0.02535
        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0,1.0);


        robot.init(hardwareMap, true);

        double[] posArray;
        posArray = new double[10];
        posArray[0] = +0.00; // Foundation Servo 1 Open
        posArray[1] = +0.35; // Foundation Servo 1 Close`
        posArray[2] = +0.00; // Foundation Servo 2 Open
        posArray[3] = +0.35; // Foundation Servo 1 Close
        posArray[4] = +0.00; // CapStone Hold
        posArray[5] = +0.50; // CapStone Release
        posArray[6] = +0.40; // Arm Unlock
        posArray[7] = +0.05; // Arm Lock
        posArray[8] = +0.30; // Claw Open Speed
        posArray[9] = -0.30; // Claw Close Speed

        if (robot.teamID == "10644") {
            posArray[2] = +0.18; // Foundation Servo 2 Open
            posArray[3] = +0.53; // Foundation Servo 1 Close
            posArray[6] = +0.40; // Arm Unlock
            posArray[7] = +0.00; // Arm Lock
        }

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        //allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


// Initialize the variables needed to store our JSON auton parameters
        int ArraySize = 50;
        double[] Param3;
        Param3 = new double[ArraySize];
        double[] Param2;
        Param2 = new double[ArraySize];
        double[] Param1;
        Param1 = new double[ArraySize];
        int[] v_state;
        v_state = new int[ArraySize];
        int v_state_current = 0;

// 0 - do nothing
// 1 - timed drive
// 2 - drive
// 3 - turn
// 4 - move foundation servo
// 5 - open/close stone grabber
// 9 - release arm


// Shared Code Below
        OpModeManagerImpl opModeManager = (OpModeManagerImpl) this.internalOpModeServices; //Store OpModeManagerImpl
        String OpModeName = robot.teamID + opModeManager.getActiveOpModeName();

        v_state[v_state_current]=UNLATCH_ARM;Param1[v_state_current]=500;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
        v_state[v_state_current]=EXPAND_ARM;Param1[v_state_current]=0.5;Param2[v_state_current]=300;Param3[v_state_current]=-0.1;v_state_current++;

        switch (opModeManager.getActiveOpModeName()) {
            case "UNLATCH":
                v_state[v_state_current]=DONE;
                break;
            case "NERD_DRIVE":
                v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=72;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=DONE;
                break;


            case "BLUE_FOUNDATION":
                turnLimit2 = 0.3;
//                v_state[v_state_current]=UNLATCH_ARM;Param1[v_state_current]=8000;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0.0;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=UNLATCH_ARM;Param1[v_state_current]=1.0;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE_OLD;Param1[v_state_current]=0.6;Param2[v_state_current]=-1200;Param3[v_state_current]=1200;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=-1050;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=1.0;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=575;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=80;Param2[v_state_current]=-0.70;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=2000;Param2[v_state_current]=-0.6;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0.0;Param2[v_state_current]=0.0;Param3[v_state_current]=750;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=180;Param2[v_state_current]=-0.5;Param3[v_state_current]=0;;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=1300;Param2[v_state_current]=-0.8;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=2200.0;v_state_current++;

                //v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.6;Param2[v_state_current]=0.0;Param3[v_state_current]=-4000;v_state_current++;
                v_state[v_state_current]=DONE;
                break;

            case "RED_FOUNDATION":
                turnLimit2 = 0.3;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=UNLATCH_ARM;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE_OLD;Param1[v_state_current]=0.6;Param2[v_state_current]=-1200;Param3[v_state_current]=-1200;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=-1200;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
                //v_state[v_state_current]=TURN;Param1[v_state_current]=10;Param2[v_state_current]=-1.0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=875;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=270;Param2[v_state_current]=0.70;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=2000;Param2[v_state_current]=-0.6;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=750;v_state_current++;

                v_state[v_state_current]=TURN;Param1[v_state_current]=0;Param2[v_state_current]=-0.5;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=1300;Param2[v_state_current]=0.8;Param3[v_state_current]=0.0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=2500.0;v_state_current++;
                v_state[v_state_current]=DONE;
                break;

            case "RED_FOUNDATION_STRAIGHT":
                turnLimit2 = 0.3;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=UNLATCH_ARM;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE_OLD;Param1[v_state_current]=0.6;Param2[v_state_current]=-1200;Param3[v_state_current]=-1200;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=-1200;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=10;Param2[v_state_current]=-1.0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TIME_DRIVE;Param1[v_state_current]=2500;Param2[v_state_current]=0.6;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=FOUNDATION;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=750;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=0;Param2[v_state_current]=1.0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=4000;v_state_current++;
                v_state[v_state_current]=DONE;
                break;

            case "RED_STONE":
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.4;Param2[v_state_current]=-800;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=IDENTIFY_SKYSTONE;Param1[v_state_current]=-600;Param2[v_state_current]=-1200;Param3[v_state_current]=20;v_state_current++;
               v_state[v_state_current]=DONE;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=1200;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=270;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=900;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=CLOSE;Param2[v_state_current]=0;Param3[v_state_current]=300;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=-500;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=180;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=3800;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=OPEN;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=-0.7;Param2[v_state_current]=0;Param3[v_state_current]=-5200;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.4;Param2[v_state_current]=-200;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=270;Param2[v_state_current]=-0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=900;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=CLOSE;Param2[v_state_current]=0;Param3[v_state_current]=1000;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=-300;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=180;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=5500;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=OPEN;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=-0.7;Param2[v_state_current]=0;Param3[v_state_current]=-1100;v_state_current++;
                v_state[v_state_current]=DONE;
                break;

            case "BLUE_STONE":



                //v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=20;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
//                v_state[v_state_current]=DONE;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.4;Param2[v_state_current]=-1100;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=IDENTIFY_SKYSTONE;Param1[v_state_current]=1200;Param2[v_state_current]=600;Param3[v_state_current]=-10;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=(-750+1200);v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=270;Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=900;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=CLOSE;Param2[v_state_current]=0;Param3[v_state_current]=300;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=-500;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=0;Param2[v_state_current]=-0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=8;Param2[v_state_current]=(72-20);Param3[v_state_current]=0;v_state_current++;

//                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=3800;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=OPEN;Param2[v_state_current]=0;Param3[v_state_current]=-400;v_state_current++;
                v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=-12;Param2[v_state_current]=(-96+20);Param3[v_state_current]=0;v_state_current++;
                //v_state[v_state_current]=DONE;v_state_current++;

                //v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=-0.7;Param2[v_state_current]=0;Param3[v_state_current]=-5400;v_state_current++;
                //v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.4;Param2[v_state_current]=200;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=(255+15);Param2[v_state_current]=0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=900;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=CLOSE;Param2[v_state_current]=0;Param3[v_state_current]=1000;v_state_current++;
                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.5;Param2[v_state_current]=0;Param3[v_state_current]=-900;v_state_current++;
                v_state[v_state_current]=TURN;Param1[v_state_current]=0;Param2[v_state_current]=-0.8;Param3[v_state_current]=0;v_state_current++;
                v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=8;Param2[v_state_current]=(96-12);Param3[v_state_current]=0;v_state_current++;

//                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=0.7;Param2[v_state_current]=0;Param3[v_state_current]=5500;v_state_current++;
                v_state[v_state_current]=CLAW;Param1[v_state_current]=OPEN;Param2[v_state_current]=0;Param3[v_state_current]=-400;v_state_current++;
                v_state[v_state_current]=NERD_DRIVE;Param1[v_state_current]=0;Param2[v_state_current]=-12;Param3[v_state_current]=0;v_state_current++;

//                v_state[v_state_current]=ENCODER_DRIVE;Param1[v_state_current]=-0.7;Param2[v_state_current]=0;Param3[v_state_current]=-1100;v_state_current++;
                v_state[v_state_current]=DONE;
                break;

            default:
                v_state[v_state_current]=DONE;
                break;



        }

        telemetry.addData("IMu",OpModeName);
        telemetry.addData("Status", v_state_current);
        telemetry.update();
        v_state_current = 0;

// last line - end of state machine


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 6.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        //for (VuforiaTrackable trackable : stoneTarget) {
            ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        //}

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // 

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
                    if (robot.armGrab != null) {robot.armGrab.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}

        double intRunTime = 0;
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("vStAtE", v_state[v_state_current]);

            switch (v_state[v_state_current]) {

                case DONE:
                    break;

                case TIME_DRIVE: // Drive straight (in any of 4 directions) for a given amount of time

                    robot.frontRightDrive.setPower(Param3[v_state_current] + Param2[v_state_current]);
                    robot.frontLeftDrive.setPower(Param3[v_state_current] - Param2[v_state_current]);
                    robot.rearRightDrive.setPower(Param3[v_state_current] - Param2[v_state_current]);
                    robot.rearLeftDrive.setPower(Param3[v_state_current] + Param2[v_state_current]);
                    intRunTime = runtime.milliseconds() + Param1[v_state_current];
                    while (runtime.milliseconds() < intRunTime) {
                        telemetry.addData("drive-l", Param3[v_state_current]);
                        telemetry.addData("drive-r", Param2[v_state_current]);
                        telemetry.update();
                        idle();
                    }
                    robot.frontRightDrive.setPower(0.0);
                    robot.frontLeftDrive.setPower(0.0);
                    robot.rearRightDrive.setPower(0.0);
                    robot.rearLeftDrive.setPower(0.0);

                    v_state_current++;
                    break;

                case ENCODER_DRIVE_OLD: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                    //encoderDrive(Param3[v_state_current], Param2[v_state_current], Param1[v_state_current]);
                    encoderDriveNew(Param3[v_state_current], Param2[v_state_current], Param1[v_state_current]);

                    v_state_current++;
                    break;

                case ENCODER_DRIVE: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                    //encoderDriveNew(Param3[v_state_current], Param2[v_state_current], Param1[v_state_current]);
                    encoderDriveMaster(Param3[v_state_current], Param2[v_state_current], Param1[v_state_current]);

                    v_state_current++;
                    break;
                case NERD_DRIVE:
                    //myNerdBOT.setMinMaxSpeeds(0,1);
                    // x which is r/l , y which is f/b, angle to mai
                            myNerdBOT.setMinMaxSpeeds(0.0,1.0);

                    myNerdBOT.nerdPidDriveWithRampUpDown((int)Param1[v_state_current], (int)Param2[v_state_current], (int)Param3[v_state_current]);
                    //myNerdBOT.nerdPidTurn(90);
                    v_state_current++;
                    break;

                case TURN: // Turn using the IMU
                    while (opModeIsActive() && !onHeading(Param1[v_state_current], Param2[v_state_current], Param3[v_state_current])) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    v_state_current++;
                    break;
                case FOUNDATION: // Open/Close FoundationServo
                    if (robot.servoFound1 != null) robot.servoFound1.setPosition(posArray[0 + (int)Param1[v_state_current]]);
                    if (robot.servoFound2 != null) robot.servoFound2.setPosition(posArray[2 + (int)Param1[v_state_current]]);
                    sleep((int)Param3[v_state_current]);
                    v_state_current++;
                    break;        

                case CLAW: // Open/Close SkyStone Claw
                    if (robot.armGrab != null) {
                        robot.armGrab.setPower(posArray[8 + (int)Param1[v_state_current]]);
                    }
                    sleep((int)Math.abs(Param3[v_state_current]));
                    if (Param3[v_state_current] <= 0d) {
                        robot.armGrab.setPower(0);
                    }
                    v_state_current++;
                    break;        

                case EXPAND_ARM: // Open/Close FoundationServo
                    if (robot.armShoulder != null) {
                        robot.armShoulder.setPower(Param1[v_state_current]);
                        sleep((int)Param2[v_state_current]);
                        robot.armShoulder.setPower(Param3[v_state_current]);
                    }
                    v_state_current++;
                    break;     
                case UNLATCH_ARM:
                    if (robot.servoArm1 != null) {
                        robot.servoArm1.setPosition(posArray[6]);
                        sleep((int)Param1[v_state_current]);
                        robot.servoArm1.setPosition(posArray[7]);
                    }
                    v_state_current++;
                    break;
                    
                case IDENTIFY_SKYSTONE:
                    // Added to stop unlock_arm
                    robot.armShoulder.setPower(0);
                    
                    //robot.frontRightDrive.setPower(-0.2);
                    //robot.frontLeftDrive.setPower(-0.2);
                    //robot.rearRightDrive.setPower(-0.2);
                    //robot.rearLeftDrive.setPower(-0.2);
                    
                    intRunTime = runtime.milliseconds() + ((int)Math.abs(Param1[v_state_current]) + (int)Math.abs(Param2[v_state_current]));//*3;
            double OffSet = 0d;

        while (!targetVisible && (intRunTime > runtime.milliseconds())) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            //for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
                    telemetry.addData("Visible Target",  stoneTarget.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    //break;
                }
            //}
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                stone = translation.get(1) * mmPerInch - 1000;
                if (stone < 0) { OffSet=Param1[v_state_current];}
                if (stone > 0) { OffSet=Param2[v_state_current];}
            } 
        }
                    //        robot.frontRightDrive.setPower(0);
                    //robot.frontLeftDrive.setPower(0);
                    //robot.rearRightDrive.setPower(0);
                    //robot.rearLeftDrive.setPower(0);

        //Param3[v_state_current+1]+=OffSet;
        //Param3[v_state_current+7]-=Math.abs(OffSet);
        if (OffSet == 0) {
            //Param3[v_state_current+9]+=600;
            //Param1[v_state_current+11]+=Param3[v_state_current]; 
        } else {
            //Param3[v_state_current+9]+=Math.abs(OffSet);
        }
        //Param3[v_state_current+16]-=Math.abs(OffSet);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();

                    v_state_current++;
                    break;


                default:
                    v_state_current++;
                    break;
            }
            telemetry.addData("current", v_state_current);
            telemetry.addData("stone", stone);
            telemetry.update();

            idle();
        }

    }

    /**
     *
     * IMU based turning using either all wheels or one side
     *
     * @param turnspeed - defualt speed of turn
     * @param fractionspeed - divide speed by this when close to target angle
     * @param angle - angle that you want to head toward (in degrees)
     * @param turnmode - 0:all wheels, -value:front wheels, +value:rear wheels
     * @return - true when you are within treshold, false otherwise
     */

    boolean onHeading(double angle, double turnspeed, double turnmode) {

        boolean onTarget = false;
        double minspeed = turnmode !=0 ? turnLimit1 : turnLimit2;
        // determine turn power based on +/- error
        double error = angle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            turnspeed = 0.0;
            currentAngle = angle;
            onTarget = true;
        } else {
            if (Math.abs(error) <= 30) { turnspeed = Range.clip(turnspeed,-minspeed,minspeed);}// / (1+Math.abs(turnmode)); }
            if (Math.abs(error) <= 45) { turnspeed = Range.clip(turnspeed,-minspeed*2,minspeed*2);}// / (1+Math.abs(turnmode)); }

        }

        // Send desired speeds to motors
        if (turnmode <=0) {
            robot.frontRightDrive.setPower(-turnspeed);
            robot.frontLeftDrive.setPower(+turnspeed);
        }
        if (turnmode >=0) {
            robot.rearRightDrive.setPower(-turnspeed);
            robot.rearLeftDrive.setPower(+turnspeed);
        }

        return onTarget;
    }


    boolean onColor(double leftspeed, double rightspeed,double distance) {
        boolean onTarget = false;
        if (String.format(Locale.US, "%.02f", robot.colorRange.getDistance(DistanceUnit.CM))=="NaN") {
            onTarget = false;
        } else if (robot.colorRange.getDistance(DistanceUnit.CM) > distance) {
            onTarget = false;
        } else {
            onTarget = true;
        }
        if (!onTarget) { encoderDrive(leftspeed,rightspeed,0.1);}
        return onTarget;



    }
    boolean onToF(double leftspeed, double forwardspeed, double distance, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;

        // determine turn power based on +/- error
        error = getToFerror(distance);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftspeed = 0.0;
            forwardspeed = 0.0;
            onTarget = true;
        } else {
            steer = 1.0;
            if (Math.abs(error) <= HEADING_THRESHOLD * 5) { steer = 0.1 / (Math.abs(leftspeed) + Math.abs(forwardspeed)); }
            leftspeed = leftspeed * steer * -(error)/Math.abs((error));
            forwardspeed = forwardspeed * steer * -(error)/Math.abs((error));
        }

        // Send desired speeds to motors.
        robot.frontRightDrive.setPower(leftspeed + forwardspeed );
        robot.frontLeftDrive.setPower(leftspeed - forwardspeed);
        robot.rearRightDrive.setPower(leftspeed - forwardspeed);
        robot.rearLeftDrive.setPower(leftspeed + forwardspeed );

        // Display it for the driver
        telemetry.addData("Target", "%5.2f", distance);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftspeed, forwardspeed);

        return onTarget;
    }


    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (in degrees relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //;;robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        //robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getToFerror(double targetDistance) {

        double robotError;
        robotError = targetDistance - robot.sensorRange.getDistance(DistanceUnit.CM);
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void encoderDrive(double countx, double county, double speed) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int FrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(countx + county);
        int FrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(countx - county);
        int RearRightTarget = robot.rearRightDrive.getCurrentPosition() + (int)(countx - county);
        int RearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(countx + county);
        int errDrive = 1;
        robot.frontRightDrive.setTargetPosition(FrontRightTarget);
        robot.frontLeftDrive.setTargetPosition(FrontLeftTarget);
        robot.rearRightDrive.setTargetPosition(RearRightTarget);
        robot.rearLeftDrive.setTargetPosition(RearLeftTarget);
        // start motion
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        double intRunTime = runtime.milliseconds() + 2000;
        
        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (errDrive > 0) && (runtime.milliseconds() < intRunTime) && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
        if (runtime.milliseconds() > 2000) { robot.armShoulder.setPower(0);}

            // Update telemetry & Allow time for other processes to run
            errDrive = 0;
            if (Math.abs(robot.frontLeftDrive.getCurrentPosition() - FrontLeftTarget) > 20) errDrive+=1;
            if (Math.abs(robot.frontRightDrive.getCurrentPosition() - FrontRightTarget) > 20) errDrive+=1;
            if (Math.abs(robot.rearLeftDrive.getCurrentPosition() - RearLeftTarget) > 20) errDrive+=1;
            if (Math.abs(robot.rearRightDrive.getCurrentPosition() - RearRightTarget) > 20) errDrive+=1;
            idle();
        }

        // Stop all motion;
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDriveNew(double countx, double county, double speed) {

        double intRunTime2 = (speed < 0) ? runtime.milliseconds() + 300 : runtime.milliseconds() + 30000;
        speed = Math.abs(speed);
        int FrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(countx + county);
        int FrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(countx - county);
        int RearRightTarget = robot.rearRightDrive.getCurrentPosition() + (int)(countx - county);
        int RearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(countx + county);
        double FrontRightSpeed = (countx + county > 0) ? speed : -speed;
        double FrontLeftSpeed = (countx - county > 0) ? speed : -speed;
        double RearRightSpeed = (countx - county > 0) ? speed : -speed;
        double RearLeftSpeed = (countx + county > 0) ? speed : -speed;
        int errDrive = 4;
        // start motion
        //robot.frontRightDrive.setPower(speed);
        //robot.frontLeftDrive.setPower(speed);
        //robot.rearRightDrive.setPower(speed);
        //robot.rearLeftDrive.setPower(speed);
        double intRunTime = runtime.milliseconds() + 10000;
        
        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (errDrive > 1) ) {
                    if (runtime.milliseconds() > 2000) { robot.armShoulder.setPower(0);}

            // Update telemetry & Allow time for other processes to run
            //errDrive = 0;
           errDrive = 4;
            if (runtime.milliseconds() > intRunTime2) { if (robot.armGrab != null) { robot.armGrab.setPower(0);}};
            if (Math.abs(robot.frontRightDrive.getCurrentPosition() - FrontRightTarget) < 800*speed)  {FrontRightSpeed = Range.clip(FrontRightSpeed,-0.2,0.2);}
            if (Math.abs(robot.frontLeftDrive.getCurrentPosition() - FrontLeftTarget) < 800*speed) {FrontLeftSpeed = Range.clip(FrontLeftSpeed,-0.2,0.2);}
            if (Math.abs(robot.rearRightDrive.getCurrentPosition() - RearRightTarget) < 800*speed)  {RearRightSpeed = Range.clip(RearRightSpeed,-0.2,0.2);}
            if (Math.abs(robot.rearLeftDrive.getCurrentPosition() - RearLeftTarget) < 800*speed)  { RearLeftSpeed = Range.clip(RearLeftSpeed,-0.2,0.2);}

            if (Math.abs(robot.frontRightDrive.getCurrentPosition() - FrontRightTarget) < 50)  {errDrive-=1; FrontRightSpeed = 0;}
            if (Math.abs(robot.frontLeftDrive.getCurrentPosition() - FrontLeftTarget) < 50) {errDrive-=1; FrontLeftSpeed = 0;}
            if (Math.abs(robot.rearRightDrive.getCurrentPosition() - RearRightTarget) < 50)  {errDrive-=1; RearRightSpeed = 0;}
            if (Math.abs(robot.rearLeftDrive.getCurrentPosition() - RearLeftTarget) < 50)  {errDrive-=1; RearLeftSpeed = 0;}
            robot.frontRightDrive.setPower(FrontRightSpeed);
            robot.frontLeftDrive.setPower(FrontLeftSpeed);
            robot.rearRightDrive.setPower(RearRightSpeed);
            robot.rearLeftDrive.setPower(RearLeftSpeed);
            
            telemetry.addData("fl", "%5d/%5d", Math.abs(robot.frontLeftDrive.getCurrentPosition() ) , FrontLeftTarget);
            telemetry.addData("fr", "%5d/%5d", Math.abs(robot.frontRightDrive.getCurrentPosition() ) , FrontRightTarget);
            telemetry.addData("rl", "%5d/%5d", Math.abs(robot.rearLeftDrive.getCurrentPosition() ) , RearLeftTarget );
            telemetry.addData("rr", "%5d/%5d", Math.abs(robot.rearRightDrive.getCurrentPosition() ) , RearRightTarget);
            telemetry.update();

            idle();
        }

        // Stop all motion;
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
    }
    
    
    
    public void encoderDriveMaster(double countx, double county, double speed) {

        // logic added to allow opening the grabber while moving
        double intRunTimeGrab = (speed < 0) ? runtime.milliseconds() + 300 : runtime.milliseconds() + 30000;
        speed = Math.abs(speed);

        // Set Target Distance for both sets of wheels (we drive 2 wheels to move diagonal, all four wheels to go F,B,L,R)
        int DistFRRL = (int)(countx + county);
        int DistFLRR = (int)(countx - county);    
        // Set Speed Direction based on Direction of Travel
        double SpeedFRRL = (DistFRRL > 0) ? speed : -speed;
        double SpeedFLRR = (DistFLRR > 0) ? speed : -speed;
        // But Set it to 0 of we are moving diagonal
        if (DistFRRL == 0) {SpeedFRRL = 0;};
        if (DistFLRR == 0) {SpeedFLRR = 0;};

        int FrontRightTarget= robot.frontRightDrive.getCurrentPosition() + DistFRRL;
        int FrontLeftTarget= robot.frontLeftDrive.getCurrentPosition() + DistFLRR;
        int RearRightTarget= robot.rearRightDrive.getCurrentPosition() + DistFLRR;
        int RearLeftTarget= robot.rearLeftDrive.getCurrentPosition() + DistFRRL;
        
        double dblMaster = 100;
        double kp = 0.001d; // 0.001 means 100 clicks off increases speed by 0.1
        double intRunTime = runtime.milliseconds() + 10000;
        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (dblMaster > 50) ) {
        if (runtime.milliseconds() > 2000) { robot.armShoulder.setPower(0);}

            // Stop Grabber from Opening Too Far
            if (runtime.milliseconds() > intRunTimeGrab) { if (robot.armGrab != null) { robot.armGrab.setPower(0);}};

            int FrontRight = robot.frontRightDrive.getCurrentPosition();
            int FrontLeft = robot.frontLeftDrive.getCurrentPosition();
            int RearRight = robot.rearRightDrive.getCurrentPosition();
            int RearLeft = robot.rearLeftDrive.getCurrentPosition();
            
            dblMaster = (SpeedFRRL > 0) ? (Math.abs(FrontRight - FrontRightTarget)) : (Math.abs(FrontLeft - FrontLeftTarget));
            
            // Slow down when we get close
            if (dblMaster < 800*speed) { SpeedFRRL = Range.clip(SpeedFRRL,-0.2,0.2); SpeedFLRR = Range.clip(SpeedFLRR,-0.2,0.2); }
            if (dblMaster < 50) {SpeedFRRL = 0; SpeedFLRR = 0; kp = 0.0d;}
            
            // Avoid div by 0 error
            if (dblMaster ==0 ) {dblMaster = 1;}
            
            if (DistFRRL != 0) {
                robot.frontRightDrive.setPower(SpeedFRRL *  Range.clip((Math.abs(FrontRight - FrontRightTarget)/dblMaster),0.9d,1.1d));
                robot.rearLeftDrive.setPower(SpeedFRRL * Range.clip((Math.abs(RearLeft - RearLeftTarget)/dblMaster),0.9d,1.1d));
                
                //robot.frontRightDrive.setPower(SpeedFRRL + (dblMaster - (Math.abs(FrontRight - FrontRightTarget)) * kp * DistFRRL / math.Abs(DistFRRL)));
                //robot.rearLeftDrive.setPower(SpeedFRRL + (dblMaster - (Math.abs(RearLeft - RearLeftTarget)) * kp * DistFRRL / math.Abs(DistFRRL)));
            }
            if (DistFLRR != 0) {
                robot.frontLeftDrive.setPower(SpeedFLRR *  Range.clip((Math.abs(FrontLeft - FrontLeftTarget)/dblMaster),0.9d,1.1d));
                robot.rearRightDrive.setPower(SpeedFLRR * Range.clip((Math.abs(RearRight- RearRightTarget)/dblMaster),0.9d,1.1d));

                //robot.frontLeftDrive.setPower(speedFLRR + (dblMaster - (Math.abs(FrontLeft - FrontLeftTarget)) * kp * DistFLRR / math.Abs(DistFLRR)));
                //robot.rearRightDrive.setPower(speedFLRR + (dblMaster - (Math.abs(RearRight - RearRightTarget)) * kp * DistFLRR / math.Abs(DistFLRR)));
            }
    
            telemetry.addData("fl", "%5d/%5d", Math.abs(robot.frontLeftDrive.getCurrentPosition() ) , FrontLeftTarget);
            telemetry.addData("fr", "%5d/%5d", Math.abs(robot.frontRightDrive.getCurrentPosition() ) , FrontRightTarget);
            telemetry.addData("rl", "%5d/%5d", Math.abs(robot.rearLeftDrive.getCurrentPosition() ) , RearLeftTarget );
            telemetry.addData("rr", "%5d/%5d", Math.abs(robot.rearRightDrive.getCurrentPosition() ) , RearRightTarget);
            telemetry.update();

            idle();
        }

        // Stop all motion;
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
    }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void servoCorrection(double servoPOS, double targetAngle) {

        // Update telemetry & Allow time for other processes to run
        double imuError = getError(targetAngle);
        double newPOS = (servoPOS-(imuError/180));
        robot.sensorAxis.setPosition(newPOS);
        telemetry.addData("error", imuError);
        telemetry.addData("sErVo", newPOS);
        telemetry.update();
        idle();

    }



}

