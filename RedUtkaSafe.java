package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Красный.Утка,Сейф", group="К")
public class RedUtkaSafe extends LinearOpMode { 
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.go(10);
        R.rotate(-80);
        R.go(90);
        R.duckVoid(-1);
        R.back(10);
        R.rotate(45);
        R.go(80);


    }

}
