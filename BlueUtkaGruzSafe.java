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
        R.back(120);
        R.rotate(-80);
        R.back(25);
        R.drop();
        R.go(80);
        R.back(15);
        R.rotate(80);
        R.go(70);
        R.rotate(20);
        R.go(40);
        R.duckVoid(1);
        R.back(80);

    }

}
