package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileOutputStream;
import android.content.Context;
import android.os.Environment;
import android.widget.Toast;

/**
 * Created by sidharth on 10/10/16.
 */


@TeleOp(name="Writer Tele Op", group="Linear Opmode")


public class WriteToOtherFile extends LinearOpMode{

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    GyroSensor gyro;
    File root = new File(Environment.getExternalStorageDirectory() + File.separator + "Auto", "Report Files");
    String fileName = "file.txt";
    //Instantiating all variables including the new text file root




    public void runOpMode() throws InterruptedException {
        myinit();
        waitForStart();
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        if (!root.exists())
        {
            root.mkdirs();
        }





        while (opModeIsActive()) {
            float leftThrottle;
            float rightThrottle;
            boolean a;
            boolean b;
            boolean x;
            boolean y;

            //Getting gyro values and encoder values
            int currentHeading = gyro.getHeading();

            //leftPos IS CURRENTLY READING ZERO -- NEEDS TO BE FIXED
            double leftPos = backLeft.getCurrentPosition();
            double rightPos = frontRight.getCurrentPosition();

            leftThrottle = gamepad1.left_stick_y;
            rightThrottle = gamepad1.right_stick_y;
            a = gamepad1.a;
            b = gamepad1.b;
            x = gamepad1.x;
            y = gamepad1.y;

            float right = rightThrottle;
            float left = leftThrottle;

            right = (float) scaleInput(right);
            left = (float) scaleInput(left);

            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);
            //Drive motors



            if(a){
                Thread.sleep(500);
                writeToFile("leftThrottle-" + String.format("%.2f", leftPos));
                writeToFile("rightThrottle-" + String.format("%.2f", rightPos));
                writeToFile("gyro-" + String.format("%d", currentHeading));
                writeToFile("/");
                reset_encoders();
                revert_encoders();
                //Writing to file, resetting and reverting encoders
               // writeToFile("a");
            }

            telemetry.addData("leftpos",leftPos);
            telemetry.addData("rightpos",rightPos);
            telemetry.addData("gyro",currentHeading);
            telemetry.update();
        }



    }

    public void myinit(){
        frontLeft = hardwareMap.dcMotor.get("motor_1");
        frontRight  = hardwareMap.dcMotor.get("motor_3");
        backLeft  = hardwareMap.dcMotor.get("motor_2");
        backRight = hardwareMap.dcMotor.get("motor_4");
        frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);




        gyro = hardwareMap.gyroSensor.get("gyro");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.calibrate();
    }
    public void writeToFile(String note) {
        //File writing
        try {

            File gpxfile = new File(root, fileName);


            FileWriter writer = new FileWriter(gpxfile,true);
            String body = note + "\n\n";
            writer.append(body);
            writer.flush();
            writer.close();
        }
        catch(IOException e) {
            e.printStackTrace();

        }
    }
    public void reset_encoders(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void revert_encoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.1, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        int index = (int) (dVal * 16.0);
        if(index < 0){
            index = -index;
        } else if(index > 16){
            index = 16;
        }

        double dScale = 0.0;

        if(dVal < 0){
            dScale = -scaleArray[index];
        } else{
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
