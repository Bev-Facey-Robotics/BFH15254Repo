package org.firstinspires.ftc.teamcode;

//Call all the imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;





@TeleOp(name = "ClawTest")

public class GibWrist extends LinearOpMode {


    public Servo wheelSpinner;
    public Servo vertiServo;
    public Servo horiServo;

    double vertPowar;
    double horiPower;
    double spinnyBoi;

    public double horiServoPosRotMod;
    public double horiServoNegRotMod;
    public double whereHoriServo;


    @Override
    public void runOpMode() {

        initBeforeYouGoGo();

        waitForStart();

        while (opModeIsActive()) {
            horiServoLogic();
            tellMeWhereUWent();
        }

    }


    public void initBeforeYouGoGo() {

        //Go go hardware finders
        wheelSpinner = hardwareMap.get(Servo.class, "Spinny");
        vertiServo = hardwareMap.get(Servo.class, "RotVert");
        horiServo = hardwareMap.get(Servo.class, "RotHori");

        //init arm telemetry
        tellMeWhereUWent();


    }

    public void horiServoLogic() {

        //when game pad two presses down left trigger, spin the spinny boi
        //wheelSpinner.setPosition(gamepad2.left_trigger);

        //Set the modifiers for increments, both up and down
        horiServoPosRotMod = 0.1;
        horiServoNegRotMod = 0.1;

        //Set the servo to be at a default position at 0.3
        //9horiServo.setPosition(0.3);

        //Make a variable for where the servo currently is
        whereHoriServo = horiServo.getPosition();

        //logic for moving in increments (I hope it works)9
        if (gamepad2.dpad_left) {
            horiServo.setPosition(whereHoriServo + horiServoPosRotMod);
        }

        if (gamepad2.dpad_right) {
            horiServo.setPosition(whereHoriServo - horiServoNegRotMod);
        }
    }

    public void tellMeWhereUWent() {
        telemetry.addData("horiservo", String.valueOf(whereHoriServo), 1);

        telemetry.update();

    }



}



