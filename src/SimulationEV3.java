import java.io.*;
import java.awt.geom.*;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

public class SimulationEV3 {
	public static void main(String[] args) {
		
		RobotMove rm = new RobotMove(); //���{�b�g�̃C���X�^���X����
		SensorMearsure sm = new SensorMearsure(); //�Z���T�̃C���X�^���X����
		
		rm.SetInstance(sm); //�Z���T�C���X�^���X�ւ̎Q�Ƃ����{�b�g�C���X�^���X���ɃZ�b�g
		sm.SetInstance(rm); //���{�b�g�C���X�^���X�ւ̎Q�Ƃ��Z���T�C���X�^���X���ɃZ�b�g
		
		final long delayRobo = 0;
		final long delaySensor = 50;
		final long periodRobo = 10;
		final long periodSensor = 50;
		
		Timer timerRobo = new Timer();
		Timer timerSensor = new Timer();
		timerRobo.scheduleAtFixedRate(rm, delayRobo, periodRobo); //���{�b�g�C���X�^���X�̃^�X�N�X�P�W���[�����O�J�n
		//timerSensor.scheduleAtFixedRate(sm,delaySensor,periodSensor); //�Z���T�C���X�^���X�̃^�X�N�X�P�W���[�����O�J�n
	}
}
