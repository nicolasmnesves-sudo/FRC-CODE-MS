
package frc.robot.commands.swervedrive.drivebase;

// ============ IMPORTAÇÕES ============
// Importa os controladores PID com perfil de movimento e as ferramentas de navegação.
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.DriveToPoseConstants;
import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;

import swervelib.SwerveInputStream;


// ============ CLASSE DE MOVIMENTAÇÃO PARA UMA POSE ============
// Controla a posição e a orientação para conduzir o robô até uma pose específica do campo.
public class DriveToPose extends Command {

    // ============ ATRIBUTOS ============ 
    // Armazena o subsistema, a pose de destino e o fluxo de entrada da condução.
    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private final SwerveInputStream stream;


    // ============ CONTROLADORES PID COM PERFIL ============ 
    // Controla os eixos X, Y e a rotação respeitando os limites de velocidade e aceleração.
    private final ProfiledPIDController xPID;
    private final ProfiledPIDController yPID;
    private final ProfiledPIDController thetaPID;


    // ============ CONSTRUTOR ============ 
    // Recebe a pose de destino e configura os controladores com os parâmetros do projeto.
    public DriveToPose(
            SwerveSubsystem swerve,
            Pose2d targetPose,
            SwerveInputStream stream) {

        this.swerve = swerve;
        this.targetPose = targetPose;
        this.stream = stream;


        // ============ PID DE TRANSLAÇÃO NO EIXO X ============ 
        // Define a correção de posição e os limites de velocidade e aceleração para X.
        xPID = new ProfiledPIDController(
                DriveToPoseConstants.TranslationPID.KP,
                DriveToPoseConstants.TranslationPID.KI,
                DriveToPoseConstants.TranslationPID.KD,
                new Constraints(
                        DriveToPoseConstants.TranslationPID.MAX_VELOCITY,
                        DriveToPoseConstants.TranslationPID.MAX_ACELERATION
                )
        );


        // ============ PID DE TRANSLAÇÃO NO EIXO Y ============ 
        // Define a correção de posição e os limites de velocidade e aceleração para Y.
        yPID = new ProfiledPIDController(
                DriveToPoseConstants.TranslationPID.KP,
                DriveToPoseConstants.TranslationPID.KI,
                DriveToPoseConstants.TranslationPID.KD,
                new Constraints(
                        DriveToPoseConstants.TranslationPID.MAX_VELOCITY,
                        DriveToPoseConstants.TranslationPID.MAX_ACELERATION
                )
        );


        // ============ PID DE ROTAÇÃO ============ 
        // Controla a orientação do robô utilizando os limites angulares configurados.
        thetaPID = new ProfiledPIDController(
                DriveToPoseConstants.RotationPID.KP,
                DriveToPoseConstants.RotationPID.KI,
                DriveToPoseConstants.RotationPID.KD,
                new Constraints(
                        DriveToPoseConstants.RotationPID.MAX_VELOCITY,
                        DriveToPoseConstants.RotationPID.MAX_ACELERATION
                )
        );


        // ============ CONFIGURAÇÃO DAS TOLERÂNCIAS ============ 
        // Define os erros máximos aceitos para posição e orientação.
        xPID.setTolerance(
                DriveToPoseConstants.TranslationPID.TOLERANCE
        );

        yPID.setTolerance(
                DriveToPoseConstants.TranslationPID.TOLERANCE
        );

        thetaPID.setTolerance(
                DriveToPoseConstants.RotationPID.TOLERANCE
        );

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }


    // ============ INICIALIZAÇÃO DO DESLOCAMENTO ============ 
    // Desativa temporariamente o controle de orientação do fluxo e inicializa os PID pela pose atual.
    @Override
    public void initialize() {

        stream.headingWhile(false);

        Pose2d pose = swerve.getPose();

        xPID.reset(pose.getX());
        yPID.reset(pose.getY());
        thetaPID.reset(pose.getRotation().getRadians());


        // ============ DEFINIÇÃO DA POSE DE DESTINO ============ 
        // Configura as metas de posição e orientação que o robô deverá alcançar.
        xPID.setGoal(targetPose.getX());
        yPID.setGoal(targetPose.getY());
        thetaPID.setGoal(targetPose.getRotation().getRadians());
    }


    // ============ EXECUÇÃO DO DESLOCAMENTO ============ 
    // Calcula as velocidades necessárias para aproximar o robô da pose desejada.
    @Override
    public void execute() {

        Pose2d pose = swerve.getPose();


        // ============ CÁLCULO DAS VELOCIDADES PID ============ 
        // Calcula as correções de translação e rotação a partir da pose atual.
        double vx = xPID.calculate(pose.getX());
        double vy = yPID.calculate(pose.getY());

        double omega = thetaPID.calculate(
                pose.getRotation().getRadians()
        );


        // ============ CONVERSÃO PARA O REFERENCIAL DO ROBÔ ============ 
        // Converte as velocidades do campo para o referencial atual do chassi.
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx,
                        vy,
                        omega,
                        pose.getRotation()
                );


        // ============ EXECUÇÃO DA MOVIMENTAÇÃO ============ 
        // Envia as velocidades calculadas para o subsistema swerve.
        swerve.drive(speeds);
    }


    // ============ VERIFICAÇÃO DE CONCLUSÃO ============ 
    // Finaliza quando os três controladores alcançam suas respectivas metas.
    @Override
    public boolean isFinished() {
        return xPID.atGoal()
                && yPID.atGoal()
                && thetaPID.atGoal();
    }


    // ============ FINALIZAÇÃO DO DESLOCAMENTO ============ 
    // Restaura o controle de orientação do fluxo e interrompe o movimento do robô.
    @Override
    public void end(boolean interrupted) {

        stream.headingWhile(true);

        swerve.drive(new ChassisSpeeds());
    }
}
