package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import though bore enocoder
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Import the IMU specific things
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleopPain")

public class TeleopPain extends LinearOpMode {

    //TO DO LIST



    //Call the IMU Class

    //Make Motor Happy With Variables
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;


    //IMU Varibles
    private BNO055IMU imu;

    private double imuX;
    private double imuY;
    private double imuZ;


    @Override
    public void runOpMode() {

        //Call important methods
        turnMeOnBeforeYouGoGo();

        while (!isStarted()) {
            //Display the IMU telemetry
            imuTelemetry();
        }

        // Wait for the game to start
        waitForStart();


        while (opModeIsActive()) {
            imuTelemetry();
            makeMeMove();
        }
    }

    public void turnMeOnBeforeYouGoGo() {


        // Initialize the motors
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");


        //Don't look for encoders please, it does funky shit
        //Initialize the IMU
        initIMU();
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration";

        //Find the damn IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Error handling if the IMU is not found
        if (imu != null) {
            imu.initialize(parameters);
        } else {
            telemetry.addData("Error", "IMU not found");
            telemetry.update();
        }

        imu.initialize(parameters);
        imuX = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        imuY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        imuZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

    }

    public void makeMeMove() {
        // Make deadzones for the joysticks
        float leftStickY = Math.abs(gamepad1.left_stick_y) > 0.4 ? gamepad1.left_stick_y : 0;
        float leftStickX = Math.abs(gamepad1.left_stick_x) > 0.4 ? gamepad1.left_stick_x : 0;
        float rightStickX = Math.abs(gamepad1.right_stick_x) > 0.4 ? gamepad1.right_stick_x : 0;

        // Calculate motor powers for straight movement
        double powerFR = leftStickY;
        double powerFL = -leftStickY;
        double powerBR = -leftStickY;
        double powerBL = leftStickY;


        // Adjust motor powers for turning
        powerFR -= leftStickX;
        powerFL -= leftStickX;
        powerBR += leftStickX;
        powerBL += leftStickX;

        // Adjust motor powers for strafing
        powerFR += rightStickX;
        powerFL += rightStickX;
        powerBR += rightStickX;
        powerBL += rightStickX;

        // Set the power of the motors
        FR.setPower(powerFR);
        FL.setPower(powerFL);
        BR.setPower(powerBR);
        BL.setPower(powerBL);
    }


    public void imuTelemetry() {

        // Get the current orientation of the robot
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        imuX = angles.firstAngle;
        imuY = angles.secondAngle;
        imuZ = angles.thirdAngle;

        // Display the orientation of the robot
        telemetry.addData("Pitch-X", "%.2f", imuX); //USB Ports to Servo Ports
        telemetry.addData("Roll-Y", "%.2f", imuY); //Motor Ports to Sensor Ports
        telemetry.addData("Yaw-Z", "%.2f", imuZ); // Top to Bottom


    }

}







