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
	R.go(30);
	R.rotate(-100);
	R.back(60);
	R.duckVoid(1);
	R.go(60);
	R.rotate(10);
	R.rotate(-90);
	R.go(90);
	R.rotate(90);
	R.drop();
	R.rotate(90);
	R.go(30);
	R.rotate(-90);
	R.back(210);


    }

}
