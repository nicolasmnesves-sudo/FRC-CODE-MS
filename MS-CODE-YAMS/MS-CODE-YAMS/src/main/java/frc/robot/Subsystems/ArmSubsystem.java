package frc.robot.Subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;




import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase{

    // Arm motor
    private final SparkMax armMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

    // config arm
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(2.660,0,0)

    .withSoftLimits(Degrees.of(-90), Degrees.of(190))

    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))

    .withIdleMode(MotorMode.BRAKE)

    .withTelemetry("armMotor", TelemetryVerbosity.HIGH)

    .withStatorCurrentLimit(Amps.of(40))

    .withMotorInverted(false)

    .withClosedLoopRampRate(Seconds.of(0.25))

    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))

    .withControlMode(ControlMode.CLOSED_LOOP)

    .withSimStartingPosition(Degrees.of(0));



    private final SmartMotorController motor = new SparkWrapper(
    armMotor, 
    DCMotor.getNEO(1), 
    motorConfig);

      private ArmConfig m_config = new ArmConfig()
      .withLength(Meters.of(0.5))
      .withHardLimits(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH);


      private final Arm arm = new Arm(m_config, motor);

      public ArmSubsystem()
      {
      }

      @Override
      public void periodic()
      {}

      @Override
      public void simulationPeriodic()
      {arm.simIterate();}

      public Command armCmd(double dutycycle)
      {return arm.set(dutycycle);}

      public Command setAngle(Angle angle)
      {return arm.setAngle(angle);}

      
}
