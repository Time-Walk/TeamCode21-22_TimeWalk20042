package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Синий.Утка,Сейф", group="С")
public class BlueUtkaSafe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.back(120);
        R.rotate(-80);
        R.back(20);
        R.drop();
        R.go(65);
        R.rotateForTime(1000, 0.5);
        R.go(80);
        R.rotate(-20);
        R.go(60);
        R.duckVoid(1);
        R.back(80);
	


    }

}
