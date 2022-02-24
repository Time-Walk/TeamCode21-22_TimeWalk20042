package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Красный.Утка,Груз,Склад", group="К")
public class RedUtkaGruzSklad extends LinearOpMode { 
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
	R.go(30);
	R.rotate(-80);
	R.go(60);
	R.duckvoid(-1);
	R.back(60);
	R.rotate(-10);
	R.rotate(90);
	R.go(90);
	R.rotate(-90);
	R.drop();
	R.rotate(-90);
	R.go(30);
	R.rotate(90);
	R.back(210);

    }

}
