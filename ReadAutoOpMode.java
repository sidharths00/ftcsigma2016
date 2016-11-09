package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.util.ArrayList;

import android.os.Environment;
import android.util.Log;
import android.widget.TextView;

/**
 * Created by sidharth on 10/26/16.
 */

@TeleOp(name="Reader Tele Op", group="Linear Opmode")


public class ReadAutoOpMode extends ReadMethods{

    FileInputStream is;
    BufferedReader reader;
    final File file = new File(Environment.getExternalStorageDirectory() + File.separator + "Auto/Report Files/file.txt");
    String fullFile;
    int pos;
    ArrayList<String> commands = new ArrayList<>();
    String arrayItem;
    double leftThrottle;
    double rightThrottle;
    double gyro;
    double leftInch;
    double rightInch;
    BlueNearAutoOpSigma2016 test = new BlueNearAutoOpSigma2016();


    @Override
    public void runOpMode() throws InterruptedException {
        myinit();
        readFile(); //Finds file on the phone and puts the whole file in a string called fullFile
        seperateCommand(); //Cuts the fullFile variable into array items in the array called commands. Cutting = separating at the slash
        waitForStart();
        while (opModeIsActive()) {
            for (int i = 0; i < commands.size(); i++){
                /*telemetry.addData("indside for loop", "true");
                telemetry.addData("Arraylist contents", commands.get(i));
                telemetry.update();*/
                arrayItem = commands.get(i); //Gets the array item
                telemetry.addData("arrayItem:", arrayItem);
                telemetry.update();
                sleep(1000);

                //Find and parse the leftthrottle value
                leftThrottle = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1, arrayItem.indexOf("rightThrottle")));
                telemetry.addData("leftThrottle:", leftThrottle);

                arrayItem = arrayItem.substring(arrayItem.indexOf("rightThrottle"));

                //Find and parse the rightthrottle value
                rightThrottle = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1, arrayItem.indexOf("gyro")));
                telemetry.addData("rightThrottle:", rightThrottle);

                arrayItem = arrayItem.substring(arrayItem.indexOf("gyro"));

                //Find and parse the gyro value
                gyro = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1));
                telemetry.addData("gyro:", gyro);
                telemetry.update();
                sleep(5000);

                //Finds distance value for throttles.
                leftInch = leftThrottle/2250;
                rightInch = rightThrottle/2250;

                //Finds if going straight or turning. Currently a PROBLEM -- see above
                if(leftThrottle > 0 && rightThrottle > 0){
                    test.gyroDrive(0.7, leftInch, gyro);
                }
                if(leftThrottle < 0 && rightThrottle < 0){
                    test.gyroDrive(0.7, leftInch, gyro);
                }
                if((leftThrottle > 0 && rightThrottle < 0) || (leftThrottle < 0 && rightThrottle > 0)){
                    test.gyroTurn(0.5, gyro);
                }
            }
            break;
        }
    }
    public void readFile(){
        if (file.exists()) {
            try {
                is = new FileInputStream(file);
                reader = new BufferedReader(new InputStreamReader(is));
                String line = "";
                fullFile = "";
                while (line != null) {
                    line = reader.readLine();
                    fullFile += line;
                    //telemetry.addData("Line:", fullFile);
                    //telemetry.update();
                }
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        else {
            telemetry.addData("File","Not Found");
            telemetry.update();
        }
        //sleep(20000);
    }

    public void seperateCommand(){
        while (fullFile.length() != 1){
            pos = fullFile.indexOf("/");

            telemetry.addData("arrayItem:", fullFile.substring(0,pos));
            telemetry.addData("length:", fullFile.length());
            telemetry.update();
            sleep(200);

            commands.add(fullFile.substring(0,pos));
            if(pos < fullFile.length() - 8){
                telemetry.addData("arrayItem:", fullFile.substring(0,pos));
                telemetry.addData("length:", fullFile.length());
                telemetry.addData("pos:", pos);
                telemetry.addData("fullFile.length() - 1:", fullFile.length() - 1);
                telemetry.addData("insideIf:", "true");
                telemetry.update();

                fullFile = fullFile.substring(pos + 1);

            }
            else{
                break;
            }
        }
    }
}