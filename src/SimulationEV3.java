import java.util.Timer;

public class SimulationEV3 {
	public static void main(String[] args) {
		long time0 = System.nanoTime();
		RobotMove rm = new RobotMove(time0); //���{�b�g�̃C���X�^���X����
		SensorMearsure sm = new SensorMearsure(time0); //�Z���T�̃C���X�^���X����
		
		rm.SetInstance(sm); //�Z���T�C���X�^���X�ւ̎Q�Ƃ����{�b�g�C���X�^���X���ɃZ�b�g
		sm.SetInstance(rm); //���{�b�g�C���X�^���X�ւ̎Q�Ƃ��Z���T�C���X�^���X���ɃZ�b�g
		
		final long delayRobo = 0;
		final long delaySensor = 50;
		final long periodRobo = 10;
		final long periodSensor = 50;
		
		Timer timerRobo = new Timer();
		Timer timerSensor = new Timer();
		timerRobo.scheduleAtFixedRate(rm, delayRobo, periodRobo); //���{�b�g�C���X�^���X�̃^�X�N�X�P�W���[�����O�J�n
		timerSensor.scheduleAtFixedRate(sm,delaySensor,periodSensor); //�Z���T�C���X�^���X�̃^�X�N�X�P�W���[�����O�J�n
	}
}
