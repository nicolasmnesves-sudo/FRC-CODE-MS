
package frc.robot.commands.swervedrive.drivebase;

// ============ IMPORTAÇÕES ============
// Importa os controles direcionais, cálculos de velocidade e ferramentas do swerve.
import java.util.List;
import java.util.function.BooleanSupplier;
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


// ============ CLASSE DE CONDUÇÃO AVANÇADA ============
// Permite escolher a orientação do robô por comandos direcionais e ajustar a rotação.
public class AbsoluteDriveAdv extends Command {

    // ============ ATRIBUTOS ============ 
    // Armazena os controles de movimento, rotação e direções predefinidas.
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier headingAdjust;

    private final BooleanSupplier lookAway;
    private final BooleanSupplier lookTowards;
    private final BooleanSupplier lookLeft;
    private final BooleanSupplier lookRight;

    private boolean resetHeading = false;


    // ============ CONSTRUTOR ============ 
    // Recebe os controles e declara o subsistema swerve como requisito.
    public AbsoluteDriveAdv(
            SwerveSubsystem swerve,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier headingAdjust,
            BooleanSupplier lookAway,
            BooleanSupplier lookTowards,
            BooleanSupplier lookLeft,
            BooleanSupplier lookRight) {

        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;

        this.lookAway = lookAway;
        this.lookTowards = lookTowards;
        this.lookLeft = lookLeft;
        this.lookRight = lookRight;

        addRequirements(swerve);
    }


    // ============ INICIALIZAÇÃO ============ 
    // Prepara o controle para preservar a orientação inicial quando necessário.
    @Override
    public void initialize() {
        resetHeading = true;
    }


    // ============ EXECUÇÃO DA CONDUÇÃO ============ 
    // Calcula a orientação desejada e transforma as entradas em velocidades do chassi.
    @Override
    public void execute() {

        double headingX = 0;
        double headingY = 0;


        // ============ SELEÇÃO DA ORIENTAÇÃO ============ 
        // Cada controle define um componente da direção desejada para o robô.
        if (lookAway.getAsBoolean()) {
            headingY = -1;
        }

        if (lookRight.getAsBoolean()) {
            headingX = 1;
        }

        if (lookLeft.getAsBoolean()) {
            headingX = -1;
        }

        if (lookTowards.getAsBoolean()) {
            headingY = 1;
        }


        // ============ PRESERVAÇÃO DA ORIENTAÇÃO INICIAL ============ 
        // Se nenhuma direção ou ajuste foi solicitado, conserva a orientação atual.
        if (resetHeading) {
            if (headingX == 0
                    && headingY == 0
                    && Math.abs(headingAdjust.getAsDouble()) == 0) {

                Rotation2d currentHeading = swerve.getHeading();

                headingX = currentHeading.getSin();
                headingY = currentHeading.getCos();
            }

            resetHeading = false;
        }


        // ============ CÁLCULO DAS VELOCIDADES ============ 
        // Converte a translação e a orientação desejadas em velocidades do chassi.
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
                vX.getAsDouble(),
                vY.getAsDouble(),
                headingX,
                headingY
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


        // ============ MOVIMENTAÇÃO E AJUSTE DE ROTAÇÃO ============ 
        // Usa o ajuste manual de rotação quando nenhuma direção predefinida está selecionada.
        if (headingX == 0
                && headingY == 0
                && Math.abs(headingAdjust.getAsDouble()) > 0) {

            resetHeading = true;

            swerve.drive(
                    translation,
                    Constants.OperatorConstants.TURN_CONSTANT
                            * -headingAdjust.getAsDouble(),
                    true
            );

        } else {
            swerve.drive(
                    translation,
                    desiredSpeeds.omegaRadiansPerSecond,
                    true
            );
        }
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
