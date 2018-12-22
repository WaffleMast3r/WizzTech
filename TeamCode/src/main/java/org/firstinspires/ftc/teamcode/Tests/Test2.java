package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;
import org.opencv.android.OpenCVLoader;

@Autonomous(name = "Test 2 - OpenCV", group = "tests")
public class Test2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVLoader.loadOpenCV();

        waitForStart();

        telemetry.addData("OpenCv Loaded", OpenCVLoader.initDebug());
        telemetry.update();

        Robot.disable();
        MotorEncoderController.disable();
    }
}
