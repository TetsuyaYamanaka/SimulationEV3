import java.io.*;
import java.awt.geom.*;

class RobotParameter{
	final double dtor = Math.PI/180; //deg→rad
	final int datamax = 1000;
	
	int i;
	double v = 4; //車体速度(cm/s)
	double omega = 0*dtor; //車体回転角度(rad)
	double[] sisei = new double[datamax]; //車体姿勢角
	double[] d_omega = new double[datamax]; //サンプリングタイムの車体回転角度
	double[] a = new double[datamax]; //サンプリングタイムの走行距離(cm)
	Point2D.Double[] location = new Point2D.Double[datamax]; //ロボットの位置
	
	RobotParameter(){
		v = 4;
		omega = 0*dtor;
		for(i=0;i<datamax;i++){
			location[i] = new Point2D.Double();
		}
	}
}

public class SimulationEV3 {
	public static void main(String[] args) {
		final int datamax = 1000;
		final double tread = 12.2;
		final double hankei = 1.5;
		final double period = 0.01;
		final int floorFlag = 1,obstFlag = 2,estFlag = 3; //（1.床確定 2.障害物確定  3.推定中）
		
		int i,cnt = 0;
		double len[] = new double[datamax],len_p[] = new double[datamax]; //cm
		double d_theta,theta;
		double theta2;
		double time = 0;
		int timemax = 5;
		double sin_theta;
		double cos_d_theta;
		Point2D.Double[] p_p = new Point2D.Double[datamax]; Point2D.Double[] p2_p = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] p = new Point2D.Double[datamax]; Point2D.Double[] p2 = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] obst = new Point2D.Double[datamax]; Point2D.Double[] obst2 = new Point2D.Double[datamax];//推定障害物座標(ワールド系)
		float[] d_omega = new float[datamax]; //1サンプリングタイムの車体回転角
		
		Point2D.Double obst_vir = new Point2D.Double(40,20); //障害物座標（ワールド系）を設定
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
		d_omega[cnt] = 0;
		len_p[cnt+1] = len[cnt];
		cnt++;

		while(time < timemax){
			robo.a[cnt] = robo.v*period;
			robo.sisei[cnt] = (float)((robo.omega/timemax)*period + robo.sisei[cnt-1]);
			robo.location[cnt].setLocation(robo.a[cnt]*Math.cos(robo.sisei[cnt]) + robo.location[cnt-1].x,robo.a[cnt]*Math.sin(robo.sisei[cnt]) + robo.location[cnt-1].y);
			len[cnt] = obst_vir.distance(robo.location[cnt]);
			robo.d_omega[cnt] = robo.sisei[cnt] - robo.sisei[cnt-1];
			len_p[cnt+1] = len[cnt];
			time += period;
			cnt++;
		}
		
		for(i=0;i<cnt;i++){
			cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - robo.a[i]*robo.a[i])/(2*len[i]*len_p[i]);
			d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める

			if(robo.d_omega[i] == 0){ //直進
				sin_theta = len_p[i]*Math.sin(d_theta)/robo.a[i];
				theta = Math.asin(sin_theta);//逆三角関数によりθを求める
				theta2 = -theta;
				
				p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
				p2[i].x = len[i]*Math.sin(theta2); p2[i].y = len[i]*Math.cos(theta2);
				
				obst[i].x = p[i].x*Math.sin(robo.sisei[i]) + p[i].y*Math.cos(robo.sisei[i]) + robo.location[i].x;
				obst[i].y = -p[i].x*Math.cos(robo.sisei[i]) + p[i].y*Math.sin(robo.sisei[i]) + robo.location[i].y;
				obst2[i].x = p2[i].x*Math.sin(robo.sisei[i]) + p2[i].y*Math.cos(robo.sisei[i]) + robo.location[i].x;
				obst2[i].y = -p2[i].x*Math.cos(robo.sisei[i]) + p2[i].y*Math.sin(robo.sisei[i]) + robo.location[i].y;
			}
			else if(robo.a[i] == 0){ //その場で回転
				p[i].x = len[i]*Math.cos(Math.PI/2 + robo.d_omega[i]);
				p[i].y = len[i]*Math.sin(Math.PI/2 + robo.d_omega[i]);
				
				obst[i].x = p[i].x*Math.sin(robo.sisei[i]+robo.d_omega[i]) + p[i].y*Math.cos(robo.sisei[i]+robo.d_omega[i]) + robo.location[i].x;
				obst[i].y = -p[i].x*Math.cos(robo.sisei[i]+robo.d_omega[i]) + p[i].y*Math.sin(robo.sisei[i]+robo.d_omega[i]) + robo.location[i].y;
			}
			else{ //旋回
				theta = d_theta + Math.abs(d_omega[i]);
				theta2 = -theta;
				if(d_omega[i] < 0){
					p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
				}
				else{
					p[i].x = -len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);	
				}
			}
			
			/*obst[i].x = p[i].x*Math.sin(sisei[i]) + p[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst[i].y = -p[i].x*Math.cos(sisei[i]) + p[i].y*Math.sin(sisei[i]) + robo[i].y;
			obst2[i].x = p2[i].x*Math.sin(sisei[i]) + p2[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst2[i].y = -p2[i].x*Math.cos(sisei[i]) + p2[i].y*Math.sin(sisei[i]) + robo[i].y;*/
		}
		
		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("SimulationP.dat")));
			PrintWriter pw2 = new PrintWriter(new BufferedWriter(new FileWriter("Obstacle.dat")));
			PrintWriter pw3 = new PrintWriter(new BufferedWriter(new FileWriter("Robozahyou.dat")));
			PrintWriter pw4 = new PrintWriter(new BufferedWriter(new FileWriter("Obstacle_virtual.dat")));
			PrintWriter pw5 = new PrintWriter(new BufferedWriter(new FileWriter("Simulationlen.dat")));
			
			pw4.println(obst_vir.x + "\t" + obst_vir.y);			
			for(i=0;i<cnt;i++){
				pw.println(p[i].x + "\t" + p[i].y + "\t" + p2[i].x + "\t" + p2[i].y);
				pw2.println(obst[i].x + "\t" + obst[i].y + "\t" + obst2[i].x + "\t" + obst2[i].y);
				pw3.println(robo.location[i].x + "\t" + robo.location[i].y);
				pw5.println(len[i]);
				gm.UpdateGrid(robo.location[i].x,robo.location[i].y,floorFlag); //ロボットの走行軌道を床確定に更新
				gm.UpdateGrid(obst[i].x,obst[i].y,obstFlag); //推定障害物位置を障害物確定に更新
				gm.UpdateGrid(obst2[i].x,obst2[i].y,obstFlag); //同上
			}
					
			pw.close();
			pw2.close();
			pw3.close();
			pw4.close();
			pw5.close();
		}catch(IOException e){
			System.out.println(e);
		}
		
		gm.PrintGrid(); //グリッドマップの情報をファイル出力（x,y,障害物確率）
	}
}
