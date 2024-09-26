package org.firstinspires.ftc.teamcode;

//import the auto class thats only a little important


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Find the expansion hub
import com.qualcomm.robotcore.hardware.HardwareMap;

//Import IMU specific things and angle thingys
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;


    @TeleOp(name = "ObvZoneRedAuto", group = "Concept")
    public class ObvZoneRedAuto extends LinearOpMode {

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


        //Run the auto
        public void runOpMode() {

            //Call important methods
            turnMeOnBeforeUMoveMove();

            // Wait for the game to start
            while (opModeIsActive()) {
                imuTelemetry();
            }

        }




        public void turnMeOnBeforeUMoveMove(){
            //Init the IMU
            initIMU();


        }


        public void recordWhereIMove() {
            //Find the motor encoder
            
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



