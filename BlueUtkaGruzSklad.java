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
		R.back(25);
		R.drop();
		R.go(80);
		R.back(15);
		R.rotate(80);
		R.go(70);
		R.rotate(19);
		R.go(45);
		R.duckVoid(1);
		R.back(20);
		R.rotate(-90);
		R.goForward(2400, -0.8);


    }

}
