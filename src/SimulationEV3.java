import java.io.*;
import java.awt.geom.*;

class RobotParameter{ //ロボットに関するパラメータ
	final double dtor = Math.PI/180; //deg→rad
	final int datamax = 1000;
	final double tread = 12.2;
	final double hankei = 1.5;
	
	int i;
	double v; //車体速度(m/s)
	double omega; //車体回転角度(rad)
	double[] sisei = new double[datamax]; //車体姿勢角
	double[] d_omega = new double[datamax]; //サンプリングタイムの車体回転角度
	double[] a = new double[datamax]; //サンプリングタイムの走行距離(cm)
	Point2D.Double[] location = new Point2D.Double[datamax]; //ロボットの位置
	
	RobotParameter(){
		v = 0.04;
		omega = 0*dtor;
		for(i=0;i<datamax;i++){
			location[i] = new Point2D.Double();
		}
	}
}

public class SimulationEV3 {
	public static void main(String[] args) {
		final int datamax = 1000;
		final double period = 0.01;
		final int floorFlag = 1,obstFlag = 2,estFlag = 3; //（1.床確定 2.障害物確定  3.推定中）
		
		int i,cnt = 0;
		double len[] = new double[datamax],len_p[] = new double[datamax]; //m
		double d_theta,theta;
		double theta2;
		double time = 0;
		int timemax = 5;
		double sin_theta;
		double cos_d_theta;
		Point2D.Double[] p_p = new Point2D.Double[datamax]; Point2D.Double[] p2_p = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] p = new Point2D.Double[datamax]; Point2D.Double[] p2 = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] obst = new Point2D.Double[datamax]; Point2D.Double[] obst2 = new Point2D.Double[datamax];//推定障害物座標(ワールド系)
		
		Point2D.Double obst_vir = new Point2D.Double(0.4,0.2); //障害物座標（ワールド系）を設定
		RobotParameter robo = new RobotParameter();
		MakeGridmap gm = new MakeGridmap();
		
		for(i=0;i<datamax;i++){
			p_p[i] = new Point2D.Double(); p2_p[i] = new Point2D.Double();
			p[i] = new Point2D.Double(); p2[i] = new Point2D.Double();
			obst[i] = new Point2D.Double(); obst2[i] = new Point2D.Double();
		}
		
		len[cnt] = obst_vir.distance(robo.location[cnt]);
		robo.a[cnt] = 0;
		robo.sisei[cnt] = (float)(0*(Math.PI/180));
		robo.d_omega[cnt] = 0;
		len_p[cnt+1] = len[cnt];
		cnt++;

		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("SimulationP.dat")));
			PrintWriter pw2 = new PrintWriter(new BufferedWriter(new FileWriter("Obstacle.dat")));
			PrintWriter pw3 = new PrintWriter(new BufferedWriter(new FileWriter("Robozahyou.dat")));
			
			while(time < timemax){
				/*走行及びロボットの位置推定*/
				robo.a[cnt] = robo.v*period;
				robo.sisei[cnt] = (float)((robo.omega/timemax)*period + robo.sisei[cnt-1]);
				robo.location[cnt].setLocation(robo.a[cnt]*Math.cos(robo.sisei[cnt]) + robo.location[cnt-1].x,robo.a[cnt]*Math.sin(robo.sisei[cnt]) + robo.location[cnt-1].y);
				len[cnt] = obst_vir.distance(robo.location[cnt]);
				robo.d_omega[cnt] = robo.sisei[cnt] - robo.sisei[cnt-1];
				
				/*距離データから障害物位置の推定*/
				cos_d_theta = (len[cnt]*len[cnt] + len_p[cnt]*len_p[cnt] - robo.a[cnt]*robo.a[cnt])/(2*len[cnt]*len_p[cnt]);
				d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める
				if(robo.a[cnt] != 0 && robo.d_omega[cnt] == 0){ //直進
					sin_theta = len_p[cnt]*Math.sin(d_theta)/robo.a[cnt];
					theta = Math.asin(sin_theta);//逆三角関数によりθを求める
					theta2 = -theta;
					
					p[cnt].x = len[cnt]*Math.sin(theta); p[cnt].y = len[cnt]*Math.cos(theta);
					p2[cnt].x = len[cnt]*Math.sin(theta2); p2[cnt].y = len[cnt]*Math.cos(theta2);
					
					obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]) + robo.location[cnt].x;
					obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]) + robo.location[cnt].y;
					obst2[cnt].x = p2[cnt].x*Math.sin(robo.sisei[cnt]) + p2[cnt].y*Math.cos(robo.sisei[cnt]) + robo.location[cnt].x;
					obst2[cnt].y = -p2[cnt].x*Math.cos(robo.sisei[cnt]) + p2[cnt].y*Math.sin(robo.sisei[cnt]) + robo.location[cnt].y;
				}
				else if(robo.a[cnt] == 0 && robo.d_omega[cnt] != 0){ //その場で回転
					p[cnt].x = len[cnt]*Math.cos(Math.PI/2 + robo.d_omega[cnt]);
					p[cnt].y = len[cnt]*Math.sin(Math.PI/2 + robo.d_omega[cnt]);
					
					obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]+robo.d_omega[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]+robo.d_omega[cnt]) + robo.location[cnt].x;
					obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]+robo.d_omega[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]+robo.d_omega[cnt]) + robo.location[cnt].y;
				}
				else if(robo.a[cnt] != 0 && robo.d_omega[cnt] != 0){ //旋回
					theta = d_theta + Math.abs(robo.d_omega[cnt]);
					theta2 = -theta;
					if(robo.d_omega[cnt] < 0){
						p[cnt].x = len[cnt]*Math.sin(theta); p[cnt].y = len[cnt]*Math.cos(theta);
						
						obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]-d_theta-robo.d_omega[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]-d_theta-robo.d_omega[cnt]) + robo.location[cnt].x;
						obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]-d_theta-robo.d_omega[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]-d_theta-robo.d_omega[cnt]) + robo.location[cnt].y;
					}
					else{
						p[cnt].x = -len[cnt]*Math.sin(theta); p[cnt].y = len[cnt]*Math.cos(theta);
						
						obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]+d_theta+robo.d_omega[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]+d_theta+robo.d_omega[cnt]) + robo.location[cnt].x;
						obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]+d_theta+robo.d_omega[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]+d_theta+robo.d_omega[cnt]) + robo.location[cnt].y;
					}
				}
				
				/*ファイル出力、グリッドマップ更新*/
				pw.println(p[cnt].x + "\t" + p[cnt].y + "\t" + p2[cnt].x + "\t" + p2[cnt].y);
				pw2.println(obst[cnt].x + "\t" + obst[cnt].y + "\t" + obst2[cnt].x + "\t" + obst2[cnt].y);
				pw3.println(robo.location[cnt].x + "\t" + robo.location[cnt].y);
				gm.UpdateGrid(robo.location[cnt],floorFlag); //ロボットの走行軌道を床確定に更新
				gm.UpdateGrid(obst[cnt],obstFlag); //推定障害物位置を障害物確定に更新
				gm.UpdateGrid(obst2[cnt],obstFlag); //同上
				//gm.SetFloorStatus(robo.location[cnt], obst[cnt]);
				//gm.GaussianUpdate();
				
				len_p[cnt+1] = len[cnt];
				time += period;
				cnt++;
			}
			
			pw.close();
			pw2.close();
			pw3.close();
		} catch(IOException e){
			System.out.println(e);
		}
		
		try{
			PrintWriter pw4 = new PrintWriter(new BufferedWriter(new FileWriter("Obstacle_virtual.dat")));
			pw4.println(obst_vir.x + "\t" + obst_vir.y);
			pw4.close();
		}catch(IOException e){
			System.out.println(e);
		}
		
		gm.PrintGrid(); //グリッドマップの情報をファイル出力（x,y,障害物確率）
	}
}
