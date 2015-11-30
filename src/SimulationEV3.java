import java.util.Timer;

public class SimulationEV3 {
	public static void main(String[] args) {
		long time0 = System.nanoTime();
		RobotMove rm = new RobotMove(time0); //ロボットのインスタンス生成
		SensorMearsure sm = new SensorMearsure(time0); //センサのインスタンス生成
		
		rm.SetInstance(sm); //センサインスタンスへの参照をロボットインスタンス内にセット
		sm.SetInstance(rm); //ロボットインスタンスへの参照をセンサインスタンス内にセット
		
		final long delayRobo = 0;
		final long delaySensor = 50;
		final long periodRobo = 10;
		final long periodSensor = 50;
		
		Timer timerRobo = new Timer();
		Timer timerSensor = new Timer();
		timerRobo.scheduleAtFixedRate(rm, delayRobo, periodRobo); //ロボットインスタンスのタスクスケジューリング開始
		timerSensor.scheduleAtFixedRate(sm,delaySensor,periodSensor); //センサインスタンスのタスクスケジューリング開始
	}
}
