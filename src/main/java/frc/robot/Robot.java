// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// ============ IMPORTS =================
// Classes do WPILib (ciclo do robô, timer, comandos) e as constantes do projeto.
// Constants fica em frc.robot.constants, por isso precisa de import próprio.
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;

// ============ CLASSE Robot =================
// Ciclo de vida do robô: a JVM chama cada método abaixo conforme o modo (disabled, auto, teleop, test).
// Se mudar o nome ou o pacote desta classe, atualize também o build.gradle e o Main.java.
public class Robot extends TimedRobot
{

    // ####################################################################
    // SEÇÃO 1 - ATRIBUTOS
    // ####################################################################

    // ============ INSTÂNCIA ÚNICA DO ROBÔ =================
    // Guarda a própria instância para outras classes acessarem via getInstance().
    // É preenchida no construtor.
    private static Robot instance;

    // ============ COMANDO AUTÔNOMO ATUAL =================
    // Guarda o comando de autônomo agendado, para poder cancelá-lo no início do teleop.
    // Fica null enquanto nenhum autônomo foi iniciado.
    private Command m_autonomousCommand;

    // ============ ROBOTCONTAINER =================
    // Contém os subsistemas, os botões e o seletor de autônomo.
    // É criado no robotInit().
    private RobotContainer m_robotContainer;

    // ============ TIMER DO FREIO AO DESABILITAR =================
    // Conta o tempo desde que o robô foi desabilitado.
    // Serve para soltar o freio dos motores depois de alguns segundos.
    private Timer disabledTimer;

    // ####################################################################
    // SEÇÃO 2 - CONSTRUTOR E ACESSO À INSTÂNCIA
    // ####################################################################

    // ============ CONSTRUTOR =================
    // Registra a instância para o getInstance().
    // Toda a inicialização de verdade fica no robotInit().
    public Robot()
    {
        instance = this;
    }

    // ============ OBTER A INSTÂNCIA DO ROBÔ =================
    // Retorna o objeto Robot em execução.
    // Útil quando uma classe precisa consultar o estado geral do robô.
    public static Robot getInstance()
    {
        return instance;
    }

    // ####################################################################
    // SEÇÃO 3 - INICIALIZAÇÃO E CICLO GERAL
    // ####################################################################

    // ============ robotInit() =================
    // Roda uma vez quando o robô liga: cria o RobotContainer e o timer do freio.
    // Em simulação, silencia o aviso de joystick desconectado.
    @Override
    public void robotInit()
    {
        // Cria o RobotContainer: monta os subsistemas, os botões e o seletor de autônomo.
        m_robotContainer = new RobotContainer();

        // Timer usado para soltar o freio alguns segundos depois de desabilitar:
        // o robô para na hora, mas depois pode ser empurrado.
        disabledTimer = new Timer();

        // O aviso de joystick só é silenciado na simulação.
        if (isSimulation())
        {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    // ============ robotPeriodic() =================
    // Roda a cada 20 ms em qualquer modo e executa o CommandScheduler.
    // Sem esta chamada, nada do Command-Based funciona (botões, comandos, subsistemas).
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    // ####################################################################
    // SEÇÃO 4 - MODO DESABILITADO
    // ####################################################################

    // ============ disabledInit() =================
    // Roda uma vez ao entrar em disabled: liga o freio dos motores e inicia o timer.
    // Assim o robô para imediatamente em vez de deslizar.
    @Override
    public void disabledInit()
    {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }

    // ============ disabledPeriodic() =================
    // Quando o tempo de WHEEL_LOCK_TIME passa, solta o freio (coast) e zera o timer.
    // Isso permite empurrar o robô com as mãos depois da partida.
    @Override
    public void disabledPeriodic()
    {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
        {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
            disabledTimer.reset();
        }
    }

    // ####################################################################
    // SEÇÃO 5 - MODO AUTÔNOMO
    // ####################################################################

    // ============ autonomousInit() =================
    // Liga o freio, pede ao RobotContainer o autônomo escolhido e o agenda.
    // Se não houver autônomo (null), nada é agendado.
    @Override
    public void autonomousInit()
    {
        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Agenda o comando autônomo, se existir.
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }

    // ============ autonomousPeriodic() =================
    // Roda a cada 20 ms durante o autônomo.
    // Está vazio: o autônomo é executado pelo CommandScheduler.
    @Override
    public void autonomousPeriodic()
    {
    }

    // ####################################################################
    // SEÇÃO 6 - MODO TELEOPERADO
    // ####################################################################

    // ============ teleopInit() =================
    // Ao começar o teleop, cancela o autônomo para ele não continuar rodando.
    // Se não havia autônomo, cancela todos os comandos agendados.
    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        } else
        {
            CommandScheduler.getInstance().cancelAll();
        }
    }

    // ============ teleopPeriodic() =================
    // Roda a cada 20 ms durante o teleop.
    // Está vazio: os botões e comandos são tratados pelo CommandScheduler.
    @Override
    public void teleopPeriodic()
    {
    }

    // ####################################################################
    // SEÇÃO 7 - MODO TESTE
    // ####################################################################

    // ============ testInit() =================
    // Ao entrar no modo teste, cancela todos os comandos em execução.
    // Garante que o teste comece sem nada rodando em segundo plano.
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    // ============ testPeriodic() =================
    // Roda a cada 20 ms durante o modo teste.
    // Está vazio por enquanto.
    @Override
    public void testPeriodic()
    {
    }

    // ####################################################################
    // SEÇÃO 8 - SIMULAÇÃO
    // ####################################################################

    // ============ simulationInit() =================
    // Roda uma vez ao iniciar a simulação.
    // Está vazio por enquanto.
    @Override
    public void simulationInit()
    {
    }

    // ============ simulationPeriodic() =================
    // Roda a cada 20 ms apenas na simulação.
    // Está vazio: o YAGSL cuida da simulação do swerve.
    @Override
    public void simulationPeriodic()
    {
    }
}