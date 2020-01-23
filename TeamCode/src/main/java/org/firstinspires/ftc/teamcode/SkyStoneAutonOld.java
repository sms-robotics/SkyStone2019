package org.firstinspires.ftc.teamcode;

//import android.graphics.Color;
//import java.lang.reflect.Array;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.Arrays;
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

public class SkyStoneAutonOld extends LinearOpMode {

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
    String stone = "N/A";

    enum ActionType {
        TIME_DRIVE(3),
        OLD_ENCODER_DRIVE(3),
        ENCODER_DRIVE(3),
        TURN(3),
        UNLATCH_ARM(2),
        CAPTURE_FOUNDATION(2),
        RELEASE_FOUNDATION(3),
        MOVE_ARM(0),
        ID_SKYSTONE(0);

        int numParams;

        ActionType(int numParams) {
            this.numParams = numParams;
        }
    }

    class Action {
        ActionType type;
        private double[] params;

        Action(ActionType type, double ... params) {
            this.type = type;
            if (params.length != type.numParams) {
                telemetry.addData("Warning: Wrong number of params: ", "%s, expected %d, got %d", type.toString(), type.numParams, params.length);
            }
            this.params = params;
        }

        double getParam(int index) {
            if (index >= params.length) {
                telemetry.addData("Warning: Bad param index: ", "%s, has max %d, tried for #%d", type.toString(), type.numParams, index + 1);
                return 0;
            }
            return params[index];
        }

        double getParam1() {
            return getParam(0);
        }
        double getParam2() {
            return getParam(1);
        }
        double getParam3() {
            return getParam(2);
        }
    }


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

        double[] posArray;
        posArray = new double[10];
        posArray[0] = 0.0;
        posArray[1] = 0.35;
        posArray[2] = 0.0;
        posArray[3] = 0.35;
        posArray[4] = 0.1;
        posArray[5] = 0.6;
        posArray[6] = 0.4;
        posArray[7] = 0.0;
        posArray[8]=  0.4;
        posArray[9]=  0.05;

        robot.init(hardwareMap, true);
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


// 0 - do nothing
// 1 - timed drive
// 2 - drive
// 3 - turn
// 4 - move foundation servo
// 5 - open/close stone grabber
// 9 - release arm


// Shared Code Below
        OpModeManagerImpl opModeManager = (OpModeManagerImpl) this.internalOpModeServices; //Store OpModeManagerImpl
        String OpModeName = robot.teamID + opModeManager.getActiveOpModeName() + ".json";

        Action[] actions;

        switch (opModeManager.getActiveOpModeName()) {
            case "BLUE_FOUNDATION":

                actions = new Action[] {
                        new Action(ActionType.UNLATCH_ARM, 0, 0),
                        new Action(ActionType.MOVE_ARM),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 1000, -1000, 0.6),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 0, -1500, 0.5),
                        new Action(ActionType.UNLATCH_ARM, 1, 750),
                        new Action(ActionType.TURN, 350, 0, 0.3),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 0, 4000, 0.6),
                        new Action(ActionType.UNLATCH_ARM, 0, 750),
                        new Action(ActionType.OLD_ENCODER_DRIVE, -4000, 0, 0.6)
                };

//                v_state[v_state_current]=4;Param1[v_state_current]=0.0;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;v_state_current++;
//                v_state[v_state_current]=9;Param1[v_state_current]=1.0;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=-1000;Param3[v_state_current]=1000;timeOut[v_state_current]=4500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.5;Param2[v_state_current]=-1500;Param3[v_state_current]=0.0;timeOut[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=4;Param1[v_state_current]=1.0;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=3;Param1[v_state_current]=350;Param2[v_state_current]=0.0;Param3[v_state_current]=0.3;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=4000;Param3[v_state_current]=0.0;v_state_current++;
//                v_state[v_state_current]=4;Param1[v_state_current]=0.0;Param2[v_state_current]=0.0;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0.0;Param3[v_state_current]=-4000;v_state_current++;
//                v_state[v_state_current]=0;
                break;

            case "BLUE_STONE":

                actions = new Action[] {
                        new Action(ActionType.MOVE_ARM),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 500, -500, 0.6),
                        new Action(ActionType.TURN, 270, 0, 0.3),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 1300, 0, 0.6),
                        new Action(ActionType.CAPTURE_FOUNDATION, -0.2, 750),
                        new Action(ActionType.TURN, 0, 0, -0.3),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 2200, 0, 0.6),
                        new Action(ActionType.RELEASE_FOUNDATION, -0.3, 0, 0),
                        new Action(ActionType.TIME_DRIVE, 2000, 0, 0.3),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0, 100),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0.3, 900),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0, 100),
                        new Action(ActionType.TIME_DRIVE, 2500, 0, -0.3)
                };
//                v_state[v_state_current]=9;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=-500;Param3[v_state_current]=500;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=3;Param1[v_state_current]=270;Param2[v_state_current]=0.0;Param3[v_state_current]=0.3;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=1300;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=-0.2;Param2[v_state_current]=0;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=3;Param1[v_state_current]=0;Param2[v_state_current]=0.0;Param3[v_state_current]=-0.3;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=2200;timeOut[v_state_current]=3000;v_state_current++;                //v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=1600;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=6;Param1[v_state_current]=-0.3;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;timeOut[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=1;Param1[v_state_current]=2000;Param2[v_state_current]=0;Param3[v_state_current]=0.3;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=100;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.3;Param2[v_state_current]=0;Param3[v_state_current]=900;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=100;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=1;Param1[v_state_current]=2500;Param2[v_state_current]=0;Param3[v_state_current]=-0.3;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=0;
                break;

            case "RED_STONE":


                actions = new Action[] {
                        new Action(ActionType.MOVE_ARM),
                        new Action(ActionType.RELEASE_FOUNDATION, -0.3, 0, 0),
                        new Action(ActionType.RELEASE_FOUNDATION, 0.3, 0, 0),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 700, -700, 0.6),
                        new Action(ActionType.TURN, 280, 0, 0.3),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 1300, 0, 0.6),
                        new Action(ActionType.CAPTURE_FOUNDATION, -0.2, 750),
                        new Action(ActionType.TURN, 180, 0, 0.3),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 2200, 0, 0.6),
                        new Action(ActionType.RELEASE_FOUNDATION, -0.3, 0, 0),
                        new Action(ActionType.TIME_DRIVE, 2000, 0, 0.3),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0, 100),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0.3, 900),
                        new Action(ActionType.CAPTURE_FOUNDATION, 0, 100)
                };
//                v_state[v_state_current]=9;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=6;Param1[v_state_current]=-0.3;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;timeOut[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=6;Param1[v_state_current]=0.3;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;timeOut[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=-700;Param3[v_state_current]=700;timeOut[v_state_current]=1500;v_state_current++;
//                v_state[v_state_current]=3;Param1[v_state_current]=280;Param2[v_state_current]=0.0;Param3[v_state_current]=0.3;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=1300;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=-0.2;Param2[v_state_current]=0;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=3;Param1[v_state_current]=180;Param2[v_state_current]=0.0;Param3[v_state_current]=0.3;timeOut[v_state_current]=1000;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=2200;timeOut[v_state_current]=3000;v_state_current++;                //v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=1600;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=6;Param1[v_state_current]=-0.3;Param2[v_state_current]=0.0;Param3[v_state_current]=0.0;timeOut[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=1;Param1[v_state_current]=2000;Param2[v_state_current]=0;Param3[v_state_current]=0.3;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=100;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.3;Param2[v_state_current]=0;Param3[v_state_current]=900;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=5;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=100;timeOut[v_state_current]=100;v_state_current++;
//                v_state[v_state_current]=0;
                break;


            case "RED_FOUNDATION_saved":


                actions = new Action[] {
                        new Action(ActionType.UNLATCH_ARM, 0, 0),
                        new Action(ActionType.MOVE_ARM),
                        new Action(ActionType.OLD_ENCODER_DRIVE, -1000, -1000, 0.6),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 0, -1500, 0.5),
                        new Action(ActionType.UNLATCH_ARM, 1, 750),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 0, 4000, 0.6),
                        new Action(ActionType.UNLATCH_ARM, 0, 750),
                        new Action(ActionType.OLD_ENCODER_DRIVE, 4000, 0, 0.6)
                };
//                v_state[v_state_current]=4;Param1[v_state_current]=0;Param2[v_state_current]=0;Param3[v_state_current]=0;v_state_current++;
//                v_state[v_state_current]=9;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=-1000;Param3[v_state_current]=-1000;timeOut[v_state_current]=4500;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.5;Param2[v_state_current]=-1500;Param3[v_state_current]=0;timeOut[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=4;Param1[v_state_current]=1;Param2[v_state_current]=500;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=4000;Param3[v_state_current]=0;timeOut[v_state_current]=5000;v_state_current++;
//                v_state[v_state_current]=4;Param1[v_state_current]=0.0;Param2[v_state_current]=0;Param3[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=2;Param1[v_state_current]=0.6;Param2[v_state_current]=0;Param3[v_state_current]=4000;timeOut[v_state_current]=3000;v_state_current++;
//                v_state[v_state_current]=0;
                break;

            case "RED_FOUNDATION":

                actions = new Action[] {
                        new Action(ActionType.ENCODER_DRIVE, 0, -600, 0.5),
                        new Action(ActionType.ID_SKYSTONE)
                };
//                //v_state[v_state_current]=3;Param1[v_state_current]=180;Param2[v_state_current]=0.8;Param3[v_state_current]=1;timeOut[v_state_current]=1000;v_state_current++;
//                //v_state[v_state_current]=92;Param1[v_state_current]=1.0;Param2[v_state_current]=2000;Param3[v_state_current]=-2000;timeOut[v_state_current]=10000;v_state_current++;
//                //v_state[v_state_current]=92;Param1[v_state_current]=0.5;Param2[v_state_current]=-1000;Param3[v_state_current]=0;timeOut[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=92;Param1[v_state_current]=0.5;Param2[v_state_current]=-600;Param3[v_state_current]=0;timeOut[v_state_current]=750;v_state_current++;
//
//                v_state[v_state_current]=55;Param1[v_state_current]=0.5;Param2[v_state_current]=-1500;Param3[v_state_current]=0;timeOut[v_state_current]=750;v_state_current++;
//                v_state[v_state_current]=0;
                break;

            default:
                actions = new Action[] {
                };

                telemetry.addData("Warning: Unknown Active Op Mode: ", OpModeName);
                break;

        }

        telemetry.addData("IMu",OpModeName);
        telemetry.update();

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

        double intRunTime = 0;
        waitForStart();
        runtime.reset();

        int index = 0;
        while (opModeIsActive()) {

            Action currAction;
            if (index < actions.length) {
                currAction = actions[index];

                telemetry.addData("Current Action: ", currAction.type.toString());

                switch (currAction.type) {
                    case TIME_DRIVE: // Drive straight (in any of 4 directions) for a given amount of time

                        robot.frontRightDrive.setPower(currAction.getParam3() + currAction.getParam2());
                        robot.frontLeftDrive.setPower(currAction.getParam3() - currAction.getParam2());
                        robot.rearRightDrive.setPower(currAction.getParam3() - currAction.getParam2());
                        robot.rearLeftDrive.setPower(currAction.getParam3() + currAction.getParam2());
                        intRunTime = runtime.milliseconds() + currAction.getParam1();
                        while (runtime.milliseconds() < intRunTime) {
                            telemetry.addData("drive-l", currAction.getParam3());
                            telemetry.addData("drive-r", currAction.getParam2());
                            telemetry.update();
                            idle();
                        }
                        robot.frontRightDrive.setPower(0.0);
                        robot.frontLeftDrive.setPower(0.0);
                        robot.rearRightDrive.setPower(0.0);
                        robot.rearLeftDrive.setPower(0.0);


                        break;

                    case OLD_ENCODER_DRIVE: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                        encoderDrive(currAction.getParam1(), currAction.getParam2(), currAction.getParam3());

                        break;

                    case ENCODER_DRIVE: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                        encoderDriveNew(currAction.getParam1(), currAction.getParam2(), currAction.getParam3());

                        index++;
                        break;

                    case TURN: // Turn using the IMU
                        while (opModeIsActive() && !onHeading(currAction.getParam1(), currAction.getParam2(), currAction.getParam3())) {
                            // Update telemetry & Allow time for other processes to run
                            telemetry.update();
                            idle();
                        }

                        break;
                    case UNLATCH_ARM: // Open/Close FoundationServo
                        if (robot.servoFound1 != null)
                            robot.servoFound1.setPosition(posArray[0 + (int) currAction.getParam1()]);
                        if (robot.servoFound2 != null)
                            robot.servoFound2.setPosition(posArray[2 + (int) currAction.getParam1()]);
                        if (robot.servoFound3 != null)
                            robot.servoFound3.setPosition(posArray[4 + (int) currAction.getParam1()]);
                        sleep((int) currAction.getParam2());

                        break;

                    case CAPTURE_FOUNDATION: // Open/Close FoundationServo
                        if (robot.armGrab != null) {
                            robot.armGrab.setPower(0);
                            robot.armGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            robot.armGrab.setPower(currAction.getParam1());
                        }
                        sleep((int) currAction.getParam2());

                        break;

                    case RELEASE_FOUNDATION: // Open/Close FoundationServo
                        if (robot.armShoulder != null) {
                            robot.armShoulder.setPower(currAction.getParam1());
                            sleep((int) currAction.getParam2());
                            robot.armShoulder.setPower(currAction.getParam3());
                        }

                        break;
                    case MOVE_ARM:
                        if (robot.servoArm1 != null) {
                            robot.servoArm1.setPosition(posArray[6]);
                            sleep(250);
                            robot.servoArm1.setPosition(posArray[7]);
                        }
                        if (robot.servoArm3 != null) {
                            robot.servoArm3.setPosition(posArray[8]);
                            sleep(250);
                            robot.servoArm3.setPosition(posArray[9]);
                        }

                        break;

                    case ID_SKYSTONE:
                        intRunTime = runtime.milliseconds() + 1500;
                        while (!targetVisible && (intRunTime > runtime.milliseconds())) {

                            // check all the trackable targets to see which one (if any) is visible.
                            targetVisible = false;
                            //for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", stoneTarget.getName());
                                targetVisible = true;

                                // getUpdatedRobotLocation() will return null if no new information is available since
                                // the last time that call was made, or if the trackable is not currently visible.
                                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                //break;
                            }
                            //}

                            // Provide feedback as to where the robot is located (if we know).
                            if (targetVisible) {
                                //telemetry.addData("Visible Target", "stone");
                                // express position (translation) of robot in inches.
                                VectorF translation = lastLocation.getTranslation();

                                if (translation.get(1) < -3 * mmPerInch) {
                                    stone = "Left";
                                }
                                if (translation.get(1) > 3 * mmPerInch) {
                                    stone = "Right";
                                }
                                if (Math.abs(translation.get(1)) <= 3 * mmPerInch) {
                                    stone = "Center";
                                }
                                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                            }
                            //telemetry.update();

                        }

                        // Disable Tracking when we are done;
                        targetsSkyStone.deactivate();
                        index++;
                        break;


                    default:
                        telemetry.addData("Unknown Action: ", currAction.type.toString());
                        break;
                }
            }
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
     * @param angle - angle that you want to head toward (in degrees)
     * @param turnmode - 0:all wheels, -value:front wheels, +value:rear wheels
     * @return - true when you are within treshold, false otherwise
     */

    boolean onHeading(double angle, double turnspeed, double turnmode) {

        boolean onTarget = false;
        double minspeed = turnmode !=0 ? 0.1 : 0.05;
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
        double intRunTime = runtime.milliseconds() + 5000;

        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (errDrive > 0) && (runtime.milliseconds() < intRunTime) && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
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
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double intRunTime = -1;//runtime.milliseconds() + 10000;

        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (errDrive > 1) ) {
            errDrive = 4;

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
            // Max timeout is 1 sec (0.2 speed of one wheel * 5000)
            if (intRunTime > runtime.milliseconds() + 1000) {intRunTime = runtime.milliseconds() + (Math.abs(FrontRightSpeed) + Math.abs(FrontLeftSpeed) + Math.abs(RearLeftSpeed) + Math.abs(RearRightSpeed)) * 5000;}
            // Stop of we are taking too long
            if (runtime.milliseconds() > intRunTime) { errDrive = 0; }
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

