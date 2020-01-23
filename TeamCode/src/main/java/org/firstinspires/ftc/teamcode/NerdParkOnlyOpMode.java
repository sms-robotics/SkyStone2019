
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="NerdParkOnlyOpMode", group="Linear Opmode")
@Disabled
public class NerdParkOnlyOpMode extends LinearOpMode {
    private NerdBOT myNerdBOT ;

    private  double speed = 0.4;
    boolean debugFlag = true;

    //Things to be changed depending on dominant alliance partner (for parking)
    private final long SLEEP_TIME = 0;
    private final double X_DISTANCE = 24.0;
    private final double Y_DISTANCE = 24.0;
    private final double Z_ANGLE = 0.0;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
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
        myNerdBOT.setMinMaxSpeeds(0.0,0.5);


        telemetry.addData("Init", "Completed");
        telemetry.update();


        waitForStart();
        sleep(SLEEP_TIME);

     myNerdBOT.setMinMaxSpeeds(0,1);
        //UNITS ARE IN INCHES
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run1");
     //   myNerdBOT.nerdPidDrive(  X_DISTANCE, Y_DISTANCE, Z_ANGLE, true, false);
     // myNerdBOT.nerdPidDriveWithRampUpDown(-85,0,0);
       // myNerdBOT.nerdPidDriveWithRampUpDown(-96,-8,0);
       myNerdBOT.nerdPidDriveWithRampUpDown(-96,0,0);
       myNerdBOT.nerdPidDriveWithRampUpDown(96,0,0);


        // myNerdBOT.nerdPidTurn(90);
       // myNerdBOT.nerdPidDriveWithRampUpDownWithArmAction(0, -40, 0, false,false,4);

    }
}
