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
        R.rotate(-15);
        R.back(30);
        R.duckVoid(1);
        R.go(30);
        R.rotate(-50);
        R.goForward(300, 0.8);
        R.back(120);
        R.rotate(90);
        R.rotate(90);
        R.rotate(90);
        R.VL.setPower(-0.8);
        R.delay(400);
        R.VL.setPower(0);
        R.back(45);
        R.drop();
        R.go(100);
        R.goForward(300, -0.2);
        R.rotate(90);
        R.go(50);

    }

}
