
package frc.robot.commands.swervedrive.drivebase;

// ============ IMPORTAÇÕES ============
// Importa as ferramentas de orientação, condução, limitação de velocidade e telemetria.
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.*;
import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;


// ============ CLASSE DE CONDUÇÃO ORIENTADA AO CAMPO ============
// Controla a translação do robô e recebe um valor de orientação desejada.
public class AbsoluteFieldDrive extends Command {

    // ============ ATRIBUTOS ============ 
    // Armazena o subsistema swerve e as entradas de translação e orientação.
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier heading;


    // ============ CONSTRUTOR ============ 
    // Recebe as entradas de condução e declara o requisito do subsistema.
    public AbsoluteFieldDrive(
            SwerveSubsystem swerve,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier heading) {

        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;

        addRequirements(swerve);
    }


    // ============ INICIALIZAÇÃO ============ 
    // Executado uma vez quando o comando é iniciado.
    @Override
    public void initialize() {
    }


    // ============ EXECUÇÃO DA CONDUÇÃO ============ 
    // Calcula as velocidades necessárias para a translação e a orientação desejadas.
    @Override
    public void execute() {

        // ============ CÁLCULO DAS VELOCIDADES ============ 
        // Converte o valor de orientação recebido em um ângulo em radianos.
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
                vX.getAsDouble(),
                vY.getAsDouble(),
                new Rotation2d(heading.getAsDouble() * Math.PI)
        );


        // ============ LIMITAÇÃO DE VELOCIDADE ============ 
        // Limita a translação considerando a velocidade, a pose e a configuração do robô.
        Translation2d translation =
                SwerveController.getTranslation2d(desiredSpeeds);

        translation = SwerveMath.limitVelocity(
                translation,
                swerve.getFieldVelocity(),
                swerve.getPose(),
                Constants.LOOP_TIME,
                Constants.ROBOT_MASS,
                List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration()
        );

        SmartDashboard.putNumber(
                "LimitedTranslation",
                translation.getX()
        );

        SmartDashboard.putString(
                "Translation",
                translation.toString()
        );


        // ============ MOVIMENTAÇÃO DO ROBÔ ============ 
        // Envia a translação limitada e a velocidade angular ao subsistema.
        swerve.drive(
                translation,
                desiredSpeeds.omegaRadiansPerSecond,
                true
        );
    }


    // ============ FINALIZAÇÃO ============ 
    // Executado quando o comando termina ou é interrompido.
    @Override
    public void end(boolean interrupted) {
    }


    // ============ VERIFICAÇÃO DE CONCLUSÃO ============ 
    // Mantém a condução ativa até que o comando seja interrompido.
    @Override
    public boolean isFinished() {
        return false;
    }
}
