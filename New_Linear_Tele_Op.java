package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sidharth on 9/28/16.
 */


//THE LINEAR TELE OP FOR SIGMAS 2016 with drive and beacon pushers.

@TeleOp(name="Linear Tele Op New 3", group="Linear Opmode")

public class New_Linear_Tele_Op extends LinearOpMode{

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor flicker;
    DcMotor intake;

    Servo beaconL;
    Servo beaconR;

    float leftThrottle;
    float rightThrottle;
    float intakeThrottle;
    float flickerThrottle;

    boolean y;
    boolean a;
    boolean up;
    boolean down;
    double beaconLPos;
    double beaconRPos;
    int increment;
    int direction = 1;
    boolean straight = false;
    boolean normal = true;

    HardwareSigma2016 robot = new HardwareSigma2016();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        myinit();
        increment = 5;
        waitForStart();

        while (opModeIsActive()) {

            leftThrottle = gamepad1.left_stick_y;
            rightThrottle = gamepad1.right_stick_y;
            intakeThrottle = gamepad2.left_stick_y;
            flickerThrottle = gamepad2.right_stick_y;

            intake.setPower(intakeThrottle);
            flicker.setPower(flickerThrottle);

            a = gamepad2.a;
            y = gamepad2.y;
            up = gamepad2.dpad_up;
            down = gamepad2.dpad_down;




            if(gamepad1.dpad_down) {
                direction *= -1;
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if(gamepad1.x){
                straight = false;
                normal = true;
            }
            else if(gamepad1.b){
                straight = true;
                normal = false;
            }

            if(straight){
                driveStraight();
            }
            else if(normal){
                notDriveStraight();
            }



            if(y){
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                beaconR.setPosition(1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconR.setPosition(-1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconR.setPosition(0.5);
            }
            else{
                beaconR.setPosition(0.5);

            }

            if(up){
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                beaconL.setPosition(-1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconL.setPosition(1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconL.setPosition(0.5);
            }
            else{
                beaconL.setPosition(0.5);
            }
        }
    }
    public void myinit(){
        frontRight = hardwareMap.dcMotor.get("motor_3");
        frontLeft = hardwareMap.dcMotor.get("motor_1");
        backRight = hardwareMap.dcMotor.get("motor_4");
        backLeft = hardwareMap.dcMotor.get("motor_2");
        beaconL = hardwareMap.servo.get("pusher_l");
        beaconR = hardwareMap.servo.get("pusher_r");

        flicker = hardwareMap.dcMotor.get("flicker");
        intake = hardwareMap.dcMotor.get("intake");

        //beaconL.scaleRange(0.2, 0.9);
        //beaconR.scaleRange(0.2, 0.9);



        //beaconL.setPosition(1.0);
        //beaconR.setPosition(0.0);
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
    public void driveStraight(){
        float right = rightThrottle;
        float left = leftThrottle;

        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        double power = increment/10;

        //Drive motors
        if(left > 0 && right > 0 || left < 0 && right < 0){
            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(-left);
            backRight.setPower(-left);
        }
        else if(left > 0 && right < 0 || left < 0 && right > 0){
            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(-right);
            backRight.setPower(-right);
        }
        else if(left == 0 && right == 0){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }


        if (increment < 3){
            increment = 3;
        }

        if (increment > 10){
            increment = 10;
        }

        if(gamepad1.right_trigger == 1) {
            increment -= 1;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (gamepad1.left_trigger == 1){
            increment += 1;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (gamepad1.left_bumper){
            increment  = 10;
        }

        if (gamepad1.right_bumper){
            increment  = 5;
        }
        telemetry.addData("increment:", increment);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();
    }
    public void notDriveStraight(){

        float right = 0;
        float left = 0;

        if(direction == 1){
            right = -rightThrottle;
            left = -leftThrottle;
        }
        else{
            right = -leftThrottle;
            left = -rightThrottle;
        }


        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        if (increment == 11){
            frontLeft.setPower(left * direction);
            backLeft.setPower(left * direction);
            frontRight.setPower(-right * direction);
            backRight.setPower(-right * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 10){
            frontLeft.setPower(left * direction);
            backLeft.setPower(left * direction);
            frontRight.setPower(-right * direction);
            backRight.setPower(-right * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 9){
            frontLeft.setPower(left * 0.9 * direction);
            backLeft.setPower(left * 0.9 * direction);
            frontRight.setPower(-right * 0.9 * direction);
            backRight.setPower(-right * 0.9 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 8){
            frontLeft.setPower(left * 0.8 * direction);
            backLeft.setPower(left * 0.8 * direction);
            frontRight.setPower(-right * 0.8 * direction);
            backRight.setPower(-right * 0.8 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 7){
            frontLeft.setPower(left * 0.7 * direction);
            backLeft.setPower(left * 0.7 * direction);
            frontRight.setPower(-right * 0.7 * direction);
            backRight.setPower(-right * 0.7 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 6){
            frontLeft.setPower(left * 0.6 * direction);
            backLeft.setPower(left * 0.6 * direction);
            frontRight.setPower(-right * 0.6 * direction);
            backRight.setPower(-right * 0.6 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 5){
            frontLeft.setPower(left * 0.5 * direction);
            backLeft.setPower(left * 0.5 * direction);
            frontRight.setPower(-right * 0.5 * direction);
            backRight.setPower(-right * 0.5 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 4){
            frontLeft.setPower(left * 0.4 * direction);
            backLeft.setPower(left * 0.4 * direction);
            frontRight.setPower(-right * 0.4 * direction);
            backRight.setPower(-right * 0.4 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 3){
            frontLeft.setPower(left * 0.3 * direction);
            backLeft.setPower(left * 0.3 * direction);
            frontRight.setPower(-right * 0.3 * direction);
            backRight.setPower(-right * 0.3 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        if (increment == 2){
            frontLeft.setPower(left * 0.3 * direction);
            backLeft.setPower(left * 0.3 * direction);
            frontRight.setPower(-right * 0.3 * direction);
            backRight.setPower(-right * 0.3 * direction);
            telemetry.addData("increment:", increment);
            telemetry.update();
        }

        /**
         * The tolerance for the variable
         */

        if (increment < 3){
            increment = 3;
        }

        if (increment > 10){
            increment = 10;
        }

        if(gamepad1.right_trigger == 1) {
            increment -= 1;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (gamepad1.left_trigger == 1){
            increment += 1;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (gamepad1.left_bumper){
            increment  = 10;
        }

        if (gamepad1.right_bumper){
            increment  = 5;
        }
    }
}
