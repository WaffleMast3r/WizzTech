package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {

    private static Robot instance = null;
    private HardwareMap hardwareMap;
    private WizzTechDcMotor leftMotorUp, rightMotorUp, leftMotorDown, rightMotorDown, extendCollectorMotorArm;
    private ServoFromDcMotor collectorMotor;
    private CRServo collectorServo;
    private BNO055IMU imu;
    private Orientation angles;
    private TeamSide side = TeamSide.UNKNOWN;

    private Robot() {
    }

    public static Robot getInstance() {
        if (instance == null) instance = new Robot();
        return instance;
    }

    public static void disable() {
        instance = null;
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        //Init of the chassis motor
        //modificare suspicioasa
        leftMotorUp = new WizzTechDcMotor("m1", hardwareMap.dcMotor.get("m1"));
        rightMotorUp = new WizzTechDcMotor("m2", hardwareMap.dcMotor.get("m2"));
        leftMotorDown = new WizzTechDcMotor("m3", hardwareMap.dcMotor.get("m3"));
        rightMotorDown = new WizzTechDcMotor("m4", hardwareMap.dcMotor.get("m4"));

//        init of the ArmCollector
//        extendCollectorMotorArm = hardwareMap.dcMotor.get("m5");
//        collectorMotor = new ServoFromDcMotor("m6", hardwareMap.dcMotor.get("m6")) {
//            @Override
//            public ArrayList<Integer> setPositions() {
//                return new ArrayList<>(Arrays.asList(50,40));
//            }
//        };
//        collectorServo = hardwareMap.crservo.get("cs1");
        initGyro(BNO055IMU.AngleUnit.DEGREES);
    }

    public void initVuforia() {
        // TODO: 12/1/2018 vuforia init
    }

    public void initGyro(BNO055IMU.AngleUnit angle) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = angle;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getAngle(Axis axis) {
        switch (axis) {
            case X:
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            case Y:
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            case Z:
                return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            default:
                return 0;
        }
    }

    public WizzTechDcMotor getLeftMotorUp() {
        return leftMotorUp;
    }

    public WizzTechDcMotor getRightMotorUp() {
        return rightMotorUp;
    }

    public WizzTechDcMotor getLeftMotorDown() {
        return leftMotorDown;
    }

    public WizzTechDcMotor getRightMotorDown() {
        return rightMotorDown;
    }

    public WizzTechDcMotor getExtendCollectorMotorArm() {
        return extendCollectorMotorArm;
    }

    public WizzTechDcMotor getCollectorMotor() {
        return collectorMotor;
    }

    public CRServo getCollectorServo() {
        return collectorServo;
    }

    public TeamSide getSide() {
        return side;
    }
    public void setSide(TeamSide side) {
        this.side = side;
    }

    @Deprecated
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public enum Axis {X, Y, Z}

    public enum TeamSide {RIGHT, LEFT, UNKNOWN}
}
