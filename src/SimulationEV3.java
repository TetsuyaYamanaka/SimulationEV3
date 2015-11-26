import java.io.*;
import java.awt.geom.*;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

public class SimulationEV3 {
	public static void main(String[] args) {
		
		RobotMove rm = new RobotMove(); //ロボットのインスタンス生成
		SensorMearsure sm = new SensorMearsure(); //センサのインスタンス生成
		
		rm.SetInstance(sm); //センサインスタンスへの参照をロボットインスタンス内にセット
		sm.SetInstance(rm); //ロボットインスタンスへの参照をセンサインスタンス内にセット
		
		final long delayRobo = 0;
		final long delaySensor = 50;
		final long periodRobo = 10;
		final long periodSensor = 50;
		
		Timer timerRobo = new Timer();
		Timer timerSensor = new Timer();
		timerRobo.scheduleAtFixedRate(rm, delayRobo, periodRobo); //ロボットインスタンスのタスクスケジューリング開始
		//timerSensor.scheduleAtFixedRate(sm,delaySensor,periodSensor); //センサインスタンスのタスクスケジューリング開始
	}
}
