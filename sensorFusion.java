package frc.robot.subsystems.odomentry;
import frc.robot.subsystems.odomentry.Gyro;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.camera.Limelight;
import edu.wpi.first.math.filter.Debouncer;

import java.lang.Thread;

public class KalmanFilter  {
    private Gyro gyro;
    private SwerveDrive swerveDrive;
    private Limelight limeLight;

    private double deltaTime;
    private double olderTime;

    private double complementaryPositionRatio = 0.2;
    private double fusionPositionRatio = 0.2;
    private double fusionVelocityRatio = 0.3;

    private states gyroVXStates;//speed in dps, it is not position, it is speed
    private states gyroVYStates;
    private states gyroVZStates;

    private states accelXStates;//acceleration in g
    private states accelYStates;
    private states accelZStates;

    private states SwerveVXStates;//speed in m/s
    private states SwerveVYStates;
    private states SwerveRotationStates;

    private states limeXStates;//position in meters
    private states limeYStates;
    private states limeRotationStates;
    //robot states epic
    private states angleXStates;//position in degrees
    private states angleYStates;

    private states robotRotationStates;
    private states robotXStates;//position in meters
    private states robotYStates;

    private boolean limeLightStatus = false;

    private class states {
        public double position;
        public double velocity;
        public double acceleration;
        public double error;
    
        public states(double p, double v, double a) {
          this.position = p;
          this.velocity = v;
          this.acceleration = a;
          this.error = 0;
        }
        public double getPosition() {
          return position-error;
        }
        public void reset(double p){
            this.position = p;
            this.velocity = 0;
            this.acceleration = 0;
        }
    }

    public KalmanFilter(Gyro gyro, SwerveDrive swerb, Limelight limeLight) {
        this.gyro = gyro;
        this.limeLight = limeLight;
        this.swerveDrive = swerb;
        deltaTime = 0.05;
        olderTime = System.currentTimeMillis();
        
        gyroVXStates = new states(gyro.getXGyroRate(), 0, 0);
        gyroVYStates = new states(gyro.getYGyroRate(), 0, 0);
        gyroVZStates = new states(gyro.getZGyroRate(), 0, 0);

        accelXStates = new states(gyro.getXAccel(), 0, 0);
        accelYStates = new states(gyro.getYAccel(), 0, 0);
        accelZStates = new states(gyro.getZAccel(), 0, 0);

        limeXStates = new states(limeLight.getRobotXPosition(), 0, 0);
        limeYStates = new states(limeLight.getRobotYPosition(), 0, 0);
        limeRotationStates = new states(limeLight.getRobotYRotation(), 0, 0);

        SwerveVXStates = new states(swerveDrive.getChassisXSpeed(), 0, 0);
        SwerveVYStates = new states(swerveDrive.getChassisYSpeed(), 0, 0);
        SwerveRotationStates = new states(swerveDrive.getChassisRotationSpeed(), 0, 0);

        angleXStates = new states(getXAccelAngle(), 0, 0);
        angleYStates = new states(getYAccelAngle(), 0, 0);

        robotXStates = new states(0, 0, 0);
        robotYStates = new states(0, 0, 0);
        robotRotationStates = new states(0, 0, 0);
        calibrateGyro();
    }

    private static double linearInterpolation(double pastDisplacement, double pastVelocity, double pastAcc, double sensorDisplacement, double bias, double deltaTime) {
        double predictedDisplacement = pastDisplacement + pastVelocity * deltaTime + 0.5 * pastAcc * deltaTime * deltaTime;
        return predictedDisplacement * (1-bias) + bias * sensorDisplacement;
    }
    private static void updateLerp(states state, double sensorPosition, double deltaTime){
        double newPosition = linearInterpolation(state.position, state.velocity, state.acceleration, sensorPosition, 0.5, deltaTime);
        double newVelocity = (newPosition - state.position) / deltaTime;
        state.acceleration = (newVelocity - state.velocity) / deltaTime;
        state.velocity = newVelocity;
        state.position = newPosition;
    }

    private double valueWrapAround(double num, double divider){
        double ratio = num / divider;
        if (Math.abs(ratio) >= 1){
            ratio -= Math.signum(ratio) * Math.round(ratio);
            if (Math.signum(ratio) * ratio < 0){
                ratio += Math.signum(ratio) * 1;
            }
        }
        return ratio * num;
    }

    private void lerpAccel(){
        updateLerp(accelXStates, gyro.getXAccel(), deltaTime);
        updateLerp(accelYStates, gyro.getYAccel(), deltaTime);
        updateLerp(accelZStates, gyro.getZAccel(), deltaTime);
    }
    private void lerpGyro(){
        updateLerp(gyroVXStates, gyro.getXGyroRate(), deltaTime);
        updateLerp(gyroVYStates, gyro.getYGyroRate(), deltaTime);
        updateLerp(gyroVZStates, gyro.getZGyroRate(), deltaTime);
    }
    private void lerpSwerve(){
        updateLerp(SwerveVXStates, swerveDrive.getChassisXSpeed(), deltaTime);
        updateLerp(SwerveVYStates, swerveDrive.getChassisYSpeed(), deltaTime);
        updateLerp(SwerveRotationStates, swerveDrive.getChassisRotationSpeed(), deltaTime);
    }
    private void lerpLime(){
        if (limeLightStatus==false && limeLight.getStatus()){
            limeLightStatus = true;
            limeXStates.reset(limeLight.getRobotXPosition());
            limeYStates.reset(limeLight.getRobotYPosition());
            limeRotationStates.reset(limeLight.getRobotYRotation());
        }
        if (limeLight.getStatus()==false){
            limeLightStatus = false;
        }
        if (limeLightStatus){
            updateLerp(limeXStates, limeLight.getRobotXPosition(), deltaTime);
            updateLerp(limeYStates, limeLight.getRobotYPosition(), deltaTime);
            updateLerp(limeRotationStates, limeLight.getRobotYRotation(), deltaTime);
        }
    }

    private void complementryFilter(){
        double newXAngle = (1 - complementaryPositionRatio) * (angleXStates.position + gyroVXStates.getPosition() * deltaTime) + complementaryPositionRatio * getXAccelAngle();
        double newYAngle = (1 - complementaryPositionRatio) * (angleYStates.position + gyroVYStates.getPosition() * deltaTime) + complementaryPositionRatio * getYAccelAngle();
        updateLerp(angleXStates, newXAngle, deltaTime);
        updateLerp(angleYStates, newYAngle, deltaTime);
    }
    @Deprecated
    private void sensorFusion(){
        double newXPosition=0;
        double newYPosition=0;
        double newRotation=0;

        if (limeLightStatus){
            newXPosition = fusionVelocityRatio * (robotXStates.position + SwerveVXStates.getPosition() * deltaTime)+ (1-fusionPositionRatio-fusionVelocityRatio)*((0.5 * getFieldXAccel() * deltaTime+robotXStates.velocity) * deltaTime + robotXStates.position) + fusionPositionRatio * limeXStates.getPosition();
            newYPosition = fusionVelocityRatio * (robotYStates.position + SwerveVYStates.getPosition() * deltaTime)+ (1-fusionPositionRatio-fusionVelocityRatio)*((0.5 * getFieldYAccel() * deltaTime+robotYStates.velocity) * deltaTime + robotYStates.position) + fusionPositionRatio * limeYStates.getPosition();
            newRotation = fusionVelocityRatio * (getHeading() + SwerveRotationStates.getPosition() * deltaTime)+ (1-fusionPositionRatio-fusionVelocityRatio)*(gyroVZStates.getPosition() * deltaTime + getHeading()) + fusionPositionRatio * limeRotationStates.getPosition();
        }else{
            newXPosition = fusionVelocityRatio * (robotXStates.position + SwerveVXStates.getPosition() * deltaTime)+ (1-fusionVelocityRatio)*((0.5 * getFieldXAccel() * deltaTime+robotXStates.velocity) * deltaTime + robotXStates.position);
            newYPosition = fusionVelocityRatio * (robotYStates.position + SwerveVYStates.getPosition() * deltaTime)+ (1-fusionVelocityRatio)*((0.5 * getFieldYAccel() * deltaTime+robotYStates.velocity) * deltaTime + robotYStates.position);
            newRotation = fusionVelocityRatio * (getHeading() + SwerveRotationStates.getPosition() * deltaTime)+ (1-fusionVelocityRatio)*(gyroVZStates.getPosition() * deltaTime+ getHeading());
        }
        updateLerp(robotXStates, newXPosition, deltaTime);
        updateLerp(robotYStates, newYPosition, deltaTime);
        updateLerp(robotRotationStates, newRotation, deltaTime);
    }

    public void update() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        if (deltaTime > 0.5) {
            complementaryPositionRatio = 1;
            fusionPositionRatio = 1;
            fusionVelocityRatio = 0;
            reset();
        }else{
            complementaryPositionRatio = 0.2;
            fusionPositionRatio = 0.2;
            fusionVelocityRatio = 0.3;
        }
        lerpAccel();
        lerpGyro();
        lerpSwerve();
        lerpLime();
        complementryFilter();
        sensorFusion();
    }

    private double getXAccelAngle() {
        return Math.toDegrees(Math.atan2(accelXStates.getPosition(), Math.sqrt(Math.pow(accelYStates.getPosition(), 2) + Math.pow(accelZStates.getPosition(), 2))));
    }
    private double getYAccelAngle(){
        return Math.toDegrees(Math.atan2(accelYStates.getPosition(), Math.sqrt(Math.pow(accelXStates.getPosition(), 2) + Math.pow(accelZStates.getPosition(), 2))));
    }

    public double getXAngle() {
        return angleXStates.getPosition();
    }
    public double getYAngle() {
        return angleYStates.getPosition();
    }
    public double getHeading() {
        return valueWrapAround(robotRotationStates.getPosition(), 360);
    }
    public double getXPosition(){
        return robotXStates.getPosition();
    }
    public double getYPosition(){
        return robotYStates.getPosition();
    }
    //easy no gravity hack
    public double getXAccel() {
        return 9.81 * (accelXStates.getPosition() - Math.sin(Math.toRadians(getXAngle())) * Math.cos(Math.toRadians(getXAngle())) * 1);
    }
    public double getYAccel() {
        return 9.81 * (accelYStates.getPosition() - Math.cos(Math.toRadians(getXAngle())) * Math.sin(Math.toRadians(getXAngle())) * 1);
    }
    public double getZAccel() {
        return 9.81 * (accelZStates.getPosition() - Math.cos(Math.toRadians(getXAngle())) * Math.cos(Math.toRadians(getXAngle())) * 1);
    }

    public double getFieldXAccel() {
        double vector1 = Math.cos(Math.toRadians(getHeading())) * (Math.cos(Math.toRadians(getXAngle())) * getXAccel() + Math.sin(Math.toRadians(getXAngle())) * Math.cos(Math.toRadians(getYAngle())) * getZAccel());
        double vector2 = Math.sin(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getYAngle())) * getYAccel();
        return vector1 + vector2;
    }
    public double getFieldYAccel() {
        double vector1 = Math.cos(Math.toRadians(getHeading())) * (Math.cos(Math.toRadians(getYAngle())) * getYAccel() + Math.sin(Math.toRadians(getYAngle())) * Math.cos(Math.toRadians(getXAngle())) * getZAccel());
        double vector2 = Math.sin(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getXAngle())) * getXAccel();
        return vector1 + vector2;
    }
    public double getFieldZAccel() {
        double vector1 = Math.cos(Math.toRadians(getYAngle())) * (Math.cos(Math.toRadians(getXAngle())) * getZAccel() + Math.sin(Math.toRadians(getXAngle())) * getXAccel());
        double vector2 = Math.sin(Math.toRadians(getYAngle())) * Math.cos(Math.toRadians(getXAngle())) * getYAccel();
        return vector1 + vector2;
    }
    
    public double getFieldXEncoderSpeed() {
        double vector1 = Math.cos(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getXAngle())) * SwerveVXStates.getPosition();
        double vector2 = Math.sin(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getYAngle())) * SwerveVYStates.getPosition();
        return vector1 + vector2;
    }
    public double getFieldYEncoderSpeed() {
        double vector1 = Math.sin(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getXAngle())) * SwerveVXStates.getPosition();
        double vector2 = Math.cos(Math.toRadians(getHeading())) * Math.cos(Math.toRadians(getYAngle())) * SwerveVYStates.getPosition();
        return vector1 + vector2;
    }

    public void calibrate3DStates(states xState, states yState, states zState){
        try{
            double[] sensorXErrors = new double[10];
            double[] sensorYErrors = new double[10];
            double[] sensorZStates = new double[10];

            double averageXError = 0;
            double averageYError = 0;
            double averageZError = 0;

            for (int i = 0; i < 10; i++) {
                update();
                sensorXErrors[i] = xState.position;
                sensorYErrors[i] = yState.position;
                sensorZStates[i] = zState.position;
                Thread.sleep(20);
            }
            for (int i = 0; i < 10; i++) {
                averageXError += sensorXErrors[i];
                averageYError += sensorYErrors[i];
                averageZError += sensorZStates[i];
            }
            xState.error = averageXError / 10;
            yState.error = averageYError / 10;
            zState.error = averageZError / 10;

        } catch (Exception e) {
            System.out.println("Error calibrating gyro");
        }
    }
    public void calibrateGyro(){
        calibrate3DStates(gyroVXStates, gyroVYStates, gyroVZStates);
        calibrate3DStates(angleXStates, angleYStates, robotRotationStates);
    }
    public void reset(){
        gyroVXStates.reset(gyro.getXGyroRate());
        gyroVYStates.reset(gyro.getYGyroRate());
        gyroVZStates.reset(gyro.getZGyroRate());

        accelXStates.reset(gyro.getXAccel());
        accelYStates.reset(gyro.getYAccel());
        accelZStates.reset(gyro.getZAccel());

        angleXStates.reset(getXAccelAngle());
        angleYStates.reset(getYAccelAngle());

        SwerveVXStates.reset(swerveDrive.getChassisXSpeed());
        SwerveVYStates.reset(swerveDrive.getChassisYSpeed());
        SwerveRotationStates.reset(swerveDrive.getChassisRotationSpeed());

        limeXStates.reset(limeLight.getRobotXPosition());
        limeYStates.reset(limeLight.getRobotYPosition());
    }
}
