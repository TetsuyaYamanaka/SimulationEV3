import java.io.*;
import java.awt.geom.*;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

public class SimulationEV3 {
	public static void main(String[] args) {
		
		MakeGridmap gm = new MakeGridmap(); //グリッドマップのインスタンス生成
		RobotMove rm = new RobotMove(); //ロボットのインスタンス生成
		SensorMearsure sm = new SensorMearsure(); //センサのインスタンス生成
		rm.TaskRun(); //ロボットインスタンスのスケジューリング開始
		sm.TaskRun(rm); //センサインスタンスのスケジューリング開始
	}
}
