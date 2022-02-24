package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Синий.Утка,Доставка,Сейф", group="С")
public class BlueUtkaGruzSafe extends LinearOpMode { //YOU SHOULD CHANGE HERE TO YOUR FILE NAME
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.back(90);
        R.rotate(-90);
        R.drop();
        R.rotate(90);
        R.go(90);
        R.rotate(90);
        R.go(60);
        R.rotate(-10);
        R.go(30);
        R.duckVoid(1);
        R.back(30);
        R.rotate(10);
        R.go(60);
        R.rotate(-90);
        R.back(60);
        R.rotate(90);
        R.back(60);


    }

}
