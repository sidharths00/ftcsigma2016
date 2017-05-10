package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */

public class HardwareSigma2016
{
    static final double COUNTS_PER_MOTOR_REV = 1310 * 1.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1 / 1.6;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public int groundbrightness_test1=0;
    public int groundbrightness_test2=0;
    public int groundbrightness_test3=0;
    public int groundbrightnessAVG = 0;
    public final double CENTER_LIGHT_THRESH = 3.5;
    public int maxLineBrightnessReading = 0;

    // speed
    public static final double kMaxLinearSpeed = 32.0; // inch per second
    public double targetAngleSpeed = 0; // degree per second
    public double targetSpeed = 0;      // inch per second
    public double currentAngleSpeed = 0; // degree per second
    public double currentSpeed = 0;     // inch per second
    public static final double P_DRIVE_COEFF = 0.015;     // Larger is more responsive, but also less stable

    public static final double TURN_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    public static final double maxLeftRightSpeedDifferentialAtDrive = 0.5;
    public static final double P_WALL_APPROACHING_COEFF = 0.05;

    public static final double MIN_TURN_SPEED = 1.6; // inch per second
    public static final int ENCODER_TARGET_THRESHOLD = 10;
    public static final double POWER_ADJ_STEP = 0.01;
    public static final double SPEED_HYSTERESIS_RANGE = 0.2;

    public static final int SPEED_CHECK_INTERVAL = 100;

    /* Public OpMode members. */
    public DcMotor  LeftMotor = null;
    public DcMotor  RightMotor = null;
    public DcMotor  CapLifter = null;
    public DcMotor  flicker = null;
    public DcMotor  intake = null;
    public Servo    pusherL    = null;
    public Servo    pusherR   = null;
    public Servo    Storage   = null;
    public ColorSensor lineLightSensor = null;
    public ColorSensor beaconColorSensor = null;
    public UltrasonicSensor ultra_front = null;
    public UltrasonicSensor ultra_back = null;

    public static final double PUSHER_L_IN  =  1.0 ;
    public static final double PUSHER_R_IN  =  0.0 ;
    public static final double PUSHER_L_OUT  =  0.0 ;
    public static final double PUSHER_R_OUT  =  1.0 ;
    public static final double PUSHER_STOP = 0.5;
    public static final double STORAGE_DOWN = 0.3;
    public static final double STORAGE_UP = 0.05;
    public static double manualDownCapPower = -0.4;
    public static double manualUpCapPower = 0.4;

    public long pusherRetractionStartTime = 0;

    public int opModeLiveCheck = 1;

    ModernRoboticsI2cGyro gyro = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSigma2016(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) throws InterruptedException {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftMotor = hwMap.dcMotor.get("LeftMotor");
        RightMotor  = hwMap.dcMotor.get("RightMotor");

        flicker = hwMap.dcMotor.get("flicker");
        // get flicker ready
        flicker.setPower(-1.0);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        flicker.setPower(0.0);

        intake = hwMap.dcMotor.get("intake");

        CapLifter = hwMap.dcMotor.get("CapLift"); //hi my name is bob and I like pie

        LeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        RightMotor.setPower(0);
        LeftMotor.setPower(0);

        // Define and initialize ALL installed servos.
        pusherL = hwMap.servo.get("pusher_l");
        pusherR = hwMap.servo.get("pusher_r");
        pusherL.setPosition(PUSHER_L_IN);
        pusherR.setPosition(PUSHER_R_IN);
        Thread.sleep(1500);
        pusherL.setPosition(PUSHER_STOP);
        pusherR.setPosition(PUSHER_STOP);

        Storage = hwMap.servo.get("Storage");
        Storage.setPosition(STORAGE_UP/2);
        Thread.sleep(500);
        Storage.setPosition(STORAGE_DOWN);

        // gyro
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        // light sensor on the robot bottom
        lineLightSensor = hwMap.colorSensor.get("line_light");
        lineLightSensor.enableLed(true);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        groundbrightness_test1 = lineLightSensor.red() + lineLightSensor.green() + lineLightSensor.blue();

        // color sensor on beacon pusher
        beaconColorSensor = hwMap.colorSensor.get("beacon_color");
        beaconColorSensor.enableLed(false);

        // ultrasonic sensor
        ultra_back = hwMap.ultrasonicSensor.get("ultra_back");
        ultra_back.getUltrasonicLevel();  // make ultrasonic sensor ready and give stable output when needed.
        ultra_front = hwMap.ultrasonicSensor.get("ultra_front");
        ultra_front.getUltrasonicLevel(); // make ultrasonic sensor ready and give stable output when needed.

        System.out.println("Sigma2016 -- hardware is initialized!");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

