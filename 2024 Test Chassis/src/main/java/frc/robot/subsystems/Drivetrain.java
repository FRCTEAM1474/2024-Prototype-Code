package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/*
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
 */
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
/*
import frc.robot.Constants.OperatorConstants.DriveMode;
*/
//import frc.robot.commands.BobDrive;
/*
import frc.robot.utils.DriveSignal;
*/


public class Drivetrain extends SubsystemBase {
/*
    private DriveMode drivemode = Constants.OperatorConstants.DriveMode.Normal;
*/
  
    // Motors
    public  WPI_TalonSRX leftLead = new WPI_TalonSRX(32);
    public  WPI_TalonSRX leftFollow1 = new WPI_TalonSRX(31);
  
    public  WPI_TalonSRX rightLead = new WPI_TalonSRX(33);
    public  WPI_TalonSRX rightFollow1 = new WPI_TalonSRX(30);
  
    public  MotorControllerGroup m_leftControllerGroup = new MotorControllerGroup(leftLead, leftFollow1);
    public  MotorControllerGroup m_rightControllerGroup = new MotorControllerGroup(rightLead, rightFollow1);
  //  public DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);
  

    private DifferentialDriveOdometry odometry;
    /*
    Rotation2d heading = new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
    */
    Rotation2d heading;

    // TODO: Check polarity
  
    private static final double rampRate = 0.25;
    private static double previousPitch = 0.0;
    private static double deltaPitch = 0.0;
  
    /** Creates a new Drivetrain. */
    public Drivetrain() {
  
      setMotorConfigsToDefault();
      setMotorInversions();
      setDefaultMotorNeutralModes();
      setMotorRampRates();
      setMotorCurrentLimits();
      setFollowers();
    } 
  
  
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      /*
      odometry.update(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
      */
      //updateDeltaPitch();
    }
    
    private void setMotorConfigsToDefault() {
      
      leftLead.configFactoryDefault();
      leftFollow1.configFactoryDefault();
      
      rightLead.configFactoryDefault();
      rightFollow1.configFactoryDefault();
    }
  
    private void setMotorInversions() {
      
      leftLead.setInverted(false);
      leftFollow1.setInverted(false);
  
      rightLead.setInverted(true);
      rightFollow1.setInverted(true);
    }
  
    // Unsure if we should remove this. - L.S
    private void setDefaultMotorNeutralModes() {
      
      leftLead.setNeutralMode(NeutralMode.Coast);
      leftFollow1.setNeutralMode(NeutralMode.Coast);
  
      rightLead.setNeutralMode(NeutralMode.Coast);
      rightFollow1.setNeutralMode(NeutralMode.Coast);
      
    }
  
    public void setNeutralMode(NeutralMode neutralMode) {
          this.leftLead.setNeutralMode(neutralMode);
          this.rightLead.setNeutralMode(neutralMode);
      }
  
  
    private void setMotorRampRates() {
  
      leftLead.configOpenloopRamp(rampRate);
      leftFollow1.configOpenloopRamp(rampRate);
  
      rightLead.configOpenloopRamp(rampRate);
      rightFollow1.configOpenloopRamp(rampRate);
      
    }
  
    private void setMotorCurrentLimits(){
      SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 35, 35, 0.0);
      leftLead.configSupplyCurrentLimit(currentLimit);
      leftFollow1.configSupplyCurrentLimit(currentLimit);
  
      rightLead.configSupplyCurrentLimit(currentLimit);
      rightFollow1.configSupplyCurrentLimit(currentLimit);
      
    }
    // Sets left and right controlmodes to left and right
    public void drive(ControlMode controlMode, double left, double right) {
      this.leftLead.set(controlMode, left);
      this.rightLead.set(controlMode, right);
    }
    
    /*
    // Getting the drive signals to get the left and right signals
    public void drive(ControlMode controlMode, DriveSignal driveSignal) {
      this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
    }
  
    public void tankDriveVolts(double leftVoltage, double rightVoltage) {
      leftLead.set(TalonSRXControlMode.PercentOutput, leftVoltage);
      rightLead.set(TalonSRXControlMode.PercentOutput, rightVoltage);
    }
    */
  
    // Sets followers to lead
    public void setFollowers() {
      leftFollow1.follow(leftLead);
      
  
      rightFollow1.follow(rightLead);
      
    }
  
    // Gets left and right lead motorspeeds
    public double getLeftMotorSpeed() {
      double leftmotorspeed = leftLead.getSelectedSensorVelocity() * .00011;
      return leftmotorspeed;
    }
  
    public double getRightMotorSpeed() {
      double rightmotorspeed = rightLead.getSelectedSensorVelocity() * .00011;
      return rightmotorspeed;
    }
  
    // Get left and right lead driving distance in meters
    public double getLeftLeadDriveDistanceMeters() {
      return 3*(this.leftLead.getSelectedSensorPosition() * 0.0011);
    }
  
    public double getRightLeadDriveDistanceMeters() {
      return 3*(this.rightLead.getSelectedSensorPosition() * 0.0011);
    }
  
    // Get left lead and right lead driving distance in tick
    public double getLeftLeadDriveDistanceTicks() {
      return -(this.leftLead.getSelectedSensorPosition());
    }
  
    public double getRightLeadDriveDistanceTicks() {
      return this.rightLead.getSelectedSensorPosition();
    }
  
    private Rotation2d getinvertedrotation2d() {
      /*
      return Rotation2d.fromDegrees(gyro.getAngle());
      */
      return Rotation2d.fromDegrees(0);
  
  
    }
  
    public Rotation2d getHeading() {
      heading = Rotation2d.fromDegrees(MathUtil.inputModulus((getinvertedrotation2d()).getDegrees(), -180, 180));
      return (heading);
    }
  
    public double getHeadingDegrees() {
      return (MathUtil.inputModulus((getinvertedrotation2d()).getDegrees(), -180, 180));
    }
  
    public void resetHeading() {
      /*
      gyro.reset();
      */
      this.getHeading();
    }
  
    //public double getPitch() {
      //return gyro.getPitch();
    //}
  
    // TODO: Make getPitch work, possible by changing everything to the navx
  
   //public double getDeltaPitch() {
      //return deltaPitch;
    //}
  
    //public void updateDeltaPitch() {
      //double currentPitch = getPitch();
      //double changeOfPitch = previousPitch - currentPitch;
      //previousPitch = currentPitch;
      //deltaPitch = Math.abs(changeOfPitch);
    //}
  
    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }
  
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : These should be scaled with the distance travelled per 'pulse'
          return new DifferentialDriveWheelSpeeds(getLeftMotorSpeed(), getRightMotorSpeed());
    }
  
    /*
    public DriveMode getDriveMode() {
      return this.drivemode;
    }
  
    public void setDriveMode(DriveMode d) {
      this.drivemode = d;
    }
  */
  
    public void zeroOdometry() {
      resetEncoders();
      resetHeading();
  
      Pose2d origin = new Pose2d(0,0,new Rotation2d(0));
      odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), origin);
    }
  
    public void resetOdometry(Pose2d pose) {
      odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), pose);
    }
  
    // Resets the encoders
    public void resetEncoders() {
      
      leftLead.setSelectedSensorPosition(0);
      leftFollow1.setSelectedSensorPosition(0);
  
      rightLead.setSelectedSensorPosition(0);
      rightFollow1.setSelectedSensorPosition(0);
      
    }
}