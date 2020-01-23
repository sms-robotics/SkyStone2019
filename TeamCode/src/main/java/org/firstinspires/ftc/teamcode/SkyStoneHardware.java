package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This class is NOT an opmode, it is used to define all the specific hardware for a single robot.

public class SkyStoneHardware
{
    /* DriveTrain */
    public DcMotor frontLeftDrive  = null;
    public DcMotor frontRightDrive = null;
    public DcMotor rearLeftDrive   = null;
    public DcMotor rearRightDrive  = null;

    /* Manipulator */

    public DcMotor armShoulder = null;
    public DcMotor armGrab = null;
    public Servo servoCapStone = null;
    public Servo servoFound1 = null;
    public Servo servoFound2 = null;
    public Servo servoFound3 = null;
    public Servo servoArm1 = null;
    public Servo servoArm3 = null;
    
    /* Auton */
    public Servo sensorAxis = null;
    public NormalizedColorSensor colorSensor = null;
    public DistanceSensor sensorRange = null; // used for 2m
    public DistanceSensor colorRange = null; // used for REV Distance+Color sensor
    public BNO055IMU imu = null;

    public String teamID = "";

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean Auton) {

        if (Auton == true) {
            // Initialize the bot-specific distance sensor
            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10644");
                teamID = "10644";
            }
            catch (Exception p_exception) { };

            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10645");
                teamID = "10645";
            }
            catch (Exception p_exception) { };

            // Embedded IMU
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            try {
                imu = ahwMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);
            } catch (Exception p_exception) { };

            // 2m Color Sensor Servo
            try
            {
                sensorAxis = ahwMap.get(Servo.class, "2maxis");
            }
            catch (Exception p_exception) { };

        }

        // CapStone placement servo (10645)
        try
        {
            servoCapStone = ahwMap.get(Servo.class, "capstone");
        }
        catch (Exception p_exception) { };

        // CapStone placement servo (10645)
        try
        {
            servoFound1 = ahwMap.get(Servo.class, "found1");
        }
        catch (Exception p_exception) { };
        try
        {
            servoFound2 = ahwMap.get(Servo.class, "found2");
        }
        catch (Exception p_exception) { };
        try
        {
            servoFound3 = ahwMap.get(Servo.class, "found3");
        }
        catch (Exception p_exception) { };

        try
        {
            servoArm3 = ahwMap.get(Servo.class, "arm1");
        }
        catch (Exception p_exception) { };

        try
        {
            servoArm3 = ahwMap.get(Servo.class, "arm3");
        }
        catch (Exception p_exception) { };

        // Define and Initialize Motors
        try
        {
            frontRightDrive = ahwMap.get(DcMotor.class, "fr");
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setTargetPosition(0);}
        catch (Exception p_exception) { };

        try
        {
            rearRightDrive = ahwMap.get(DcMotor.class, "rr");
            rearRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            frontLeftDrive = ahwMap.get(DcMotor.class, "fl");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            rearLeftDrive = ahwMap.get(DcMotor.class, "rl");
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftDrive.setTargetPosition(0);
        }
        catch (Exception p_exception) { };

        try
        {
            armShoulder = ahwMap.get(DcMotor.class, "am");
            armShoulder.setDirection(DcMotor.Direction.REVERSE);
            //armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //if (Auton == true) { armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armShoulder.setTargetPosition(0);
            armShoulder.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

        try
        {
            armGrab = ahwMap.get(DcMotor.class, "ag");
            armGrab.setDirection(DcMotor.Direction.FORWARD);
            armGrab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (Auton == true) { armGrab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
            armGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armGrab.setTargetPosition(0);
            armGrab.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

    }
}

