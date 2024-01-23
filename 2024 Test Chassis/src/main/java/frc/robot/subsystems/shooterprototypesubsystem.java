


package frc.robot.subsystems;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
public class ShooterPrototypeSubsystem {
*/
    public class shooterprototypesubsystem extends SubsystemBase {
    
    public static WPI_TalonSRX shooterPrototypeController = new WPI_TalonSRX(34);

    public shooterprototypesubsystem() {
        
    }

    public static void setspeedofShooterPrototypeMotor(double speed) {
        shooterPrototypeController.set(speed);
    }
/*
    public static boolean extendedvelevatorlimitswitchstatus() {
        return extendedvelevatorlimitswitch.get();
    }

    public static boolean retractedvelevatorlimitswitchstatus() {
        return retractedvelevatorlimitswitch.get();
    }

    public static double velevatorencoderposition() {
        RelativeEncoder velevationencoder = leftvelevationmotor.getEncoder();
        return velevationencoder.getPosition();
    }

    public static void zerovelevatorencoder() {
        leftvelevationmotor.getEncoder().setPosition(0);
    }
*/ 
    }
/*
}
*/
