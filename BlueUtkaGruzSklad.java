package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Синий.Утка,Груз,Склад", group="С")
public class BlueUtkaGruzSklad extends LinearOpMode {
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
		R.go(60);
		R.rotateForTime(1000, 0.5);
		R.go(80);
		//R.rotate(-10);
		R.go(60);
		R.rotate(15);
		R.duckVoid(1);
		R.rotate(-95);
		R.goForward(2200, -0.8);


    }

}
