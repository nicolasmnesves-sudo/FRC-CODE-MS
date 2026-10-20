
package frc.robot.commands.swervedrive.drivebase;

// ============ IMPORTAÇÕES ============
// Importa ferramentas de condução, limitação de velocidade e leitura dos controles.
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


// ============ CLASSE DE CONDUÇÃO ABSOLUTA ============
// Controla a translação do robô e define sua orientação por dois eixos do joystick.
public class AbsoluteDrive extends Command {

    // ============ ATRIBUTOS ============ 
    // Armazena os controles de translação, orientação e o subsistema swerve.
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier headingHorizontal;
    private final DoubleSupplier headingVertical;

    private boolean initRotation = false;


    // ============ CONSTRUTOR ============ 
    // Recebe os controles e declara o subsistema utilizado pelo comando.
    public AbsoluteDrive(
            SwerveSubsystem swerve,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier headingHorizontal,
            DoubleSupplier headingVertical) {

        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;

        addRequirements(swerve);
    }


    // ============ INICIALIZAÇÃO ============ 
    // Prepara o comando para preservar a orientação inicial quando necessário.
    @Override
    public void initialize() {
        initRotation = true;
    }


    // ============ EXECUÇÃO DA CONDUÇÃO ============ 
    // Calcula as velocidades desejadas usando os controles de translação e orientação.
    @Override
    public void execute() {

        // ============ CÁLCULO DAS VELOCIDADES ============ 
        // Converte as entradas dos controles em velocidades para o chassi.
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
                vX.getAsDouble(),
                vY.getAsDouble(),
                headingHorizontal.getAsDouble(),
                headingVertical.getAsDouble()
        );


        // ============ PRESERVAÇÃO DA ORIENTAÇÃO INICIAL ============ 
        // Evita uma mudança indesejada de orientação quando o comando começa.
        if (initRotation) {
            if (headingHorizontal.getAsDouble() == 0
                    && headingVertical.getAsDouble() == 0) {

                Rotation2d firstLoopHeading = swerve.getHeading();

                desiredSpeeds = swerve.getTargetSpeeds(
                        0,
                        0,
                        firstLoopHeading.getSin(),
                        firstLoopHeading.getCos()
                );
            }

            initRotation = false;
        }


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
        // Envia a translação limitada e a velocidade angular para o subsistema.
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
