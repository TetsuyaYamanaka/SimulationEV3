import java.io.*;
import java.awt.geom.*;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

public class SimulationEV3 {
	public static void main(String[] args) {
		
		MakeGridmap gm = new MakeGridmap(); //�O���b�h�}�b�v�̃C���X�^���X����
		RobotMove rm = new RobotMove(); //���{�b�g�̃C���X�^���X����
		SensorMearsure sm = new SensorMearsure(); //�Z���T�̃C���X�^���X����
		rm.TaskRun(); //���{�b�g�C���X�^���X�̃X�P�W���[�����O�J�n
		sm.TaskRun(rm); //�Z���T�C���X�^���X�̃X�P�W���[�����O�J�n
	}
}
