package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Синий.Груз,Склад", group="С")
public class BlueGruzSklad extends LinearOpMode { 
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.delay(5500);
        R.back(50);
        R.drop();
        R.back(10);
        R.rotate(-80);
        R.goForward(1200, -0.9);

    }

}
