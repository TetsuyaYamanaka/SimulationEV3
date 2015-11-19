import java.io.*;
import java.awt.geom.*;

class RobotParameter{ //���{�b�g�Ɋւ���p�����[�^
	final double dtor = Math.PI/180; //deg��rad
	final int datamax = 1000;
	final double tread = 12.2;
	final double hankei = 1.5;
	
	int i;
	double v; //�ԑ̑��x(m/s)
	double omega; //�ԑ̉�]�p�x(rad)
	double[] sisei = new double[datamax]; //�ԑ̎p���p
	double[] d_omega = new double[datamax]; //�T���v�����O�^�C���̎ԑ̉�]�p�x
	double[] a = new double[datamax]; //�T���v�����O�^�C���̑��s����(cm)
	Point2D.Double[] location = new Point2D.Double[datamax]; //���{�b�g�̈ʒu
	
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
		final int floorFlag = 1,obstFlag = 2,estFlag = 3; //�i1.���m�� 2.��Q���m��  3.���蒆�j
		
		int i,cnt = 0;
		double len[] = new double[datamax],len_p[] = new double[datamax]; //m
		double d_theta,theta;
		double theta2;
		double time = 0;
		int timemax = 5;
		double sin_theta;
		double cos_d_theta;
		Point2D.Double[] p_p = new Point2D.Double[datamax]; Point2D.Double[] p2_p = new Point2D.Double[datamax];//�O�����̐����Q�����W(���{�b�g�n)
		Point2D.Double[] p = new Point2D.Double[datamax]; Point2D.Double[] p2 = new Point2D.Double[datamax];//�O�����̐����Q�����W(���{�b�g�n)
		Point2D.Double[] obst = new Point2D.Double[datamax]; Point2D.Double[] obst2 = new Point2D.Double[datamax];//�����Q�����W(���[���h�n)
		
		Point2D.Double obst_vir = new Point2D.Double(0.4,0.2); //��Q�����W�i���[���h�n�j��ݒ�
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
				/*���s�y�у��{�b�g�̈ʒu����*/
				robo.a[cnt] = robo.v*period;
				robo.sisei[cnt] = (float)((robo.omega/timemax)*period + robo.sisei[cnt-1]);
				robo.location[cnt].setLocation(robo.a[cnt]*Math.cos(robo.sisei[cnt]) + robo.location[cnt-1].x,robo.a[cnt]*Math.sin(robo.sisei[cnt]) + robo.location[cnt-1].y);
				len[cnt] = obst_vir.distance(robo.location[cnt]);
				robo.d_omega[cnt] = robo.sisei[cnt] - robo.sisei[cnt-1];
				
				/*�����f�[�^�����Q���ʒu�̐���*/
				cos_d_theta = (len[cnt]*len[cnt] + len_p[cnt]*len_p[cnt] - robo.a[cnt]*robo.a[cnt])/(2*len[cnt]*len_p[cnt]);
				d_theta = Math.acos(cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
				if(robo.a[cnt] != 0 && robo.d_omega[cnt] == 0){ //���i
					sin_theta = len_p[cnt]*Math.sin(d_theta)/robo.a[cnt];
					theta = Math.asin(sin_theta);//�t�O�p�֐��ɂ��Ƃ����߂�
					theta2 = -theta;
					
					p[cnt].x = len[cnt]*Math.sin(theta); p[cnt].y = len[cnt]*Math.cos(theta);
					p2[cnt].x = len[cnt]*Math.sin(theta2); p2[cnt].y = len[cnt]*Math.cos(theta2);
					
					obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]) + robo.location[cnt].x;
					obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]) + robo.location[cnt].y;
					obst2[cnt].x = p2[cnt].x*Math.sin(robo.sisei[cnt]) + p2[cnt].y*Math.cos(robo.sisei[cnt]) + robo.location[cnt].x;
					obst2[cnt].y = -p2[cnt].x*Math.cos(robo.sisei[cnt]) + p2[cnt].y*Math.sin(robo.sisei[cnt]) + robo.location[cnt].y;
				}
				else if(robo.a[cnt] == 0 && robo.d_omega[cnt] != 0){ //���̏�ŉ�]
					p[cnt].x = len[cnt]*Math.cos(Math.PI/2 + robo.d_omega[cnt]);
					p[cnt].y = len[cnt]*Math.sin(Math.PI/2 + robo.d_omega[cnt]);
					
					obst[cnt].x = p[cnt].x*Math.sin(robo.sisei[cnt]+robo.d_omega[cnt]) + p[cnt].y*Math.cos(robo.sisei[cnt]+robo.d_omega[cnt]) + robo.location[cnt].x;
					obst[cnt].y = -p[cnt].x*Math.cos(robo.sisei[cnt]+robo.d_omega[cnt]) + p[cnt].y*Math.sin(robo.sisei[cnt]+robo.d_omega[cnt]) + robo.location[cnt].y;
				}
				else if(robo.a[cnt] != 0 && robo.d_omega[cnt] != 0){ //����
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
				
				/*�t�@�C���o�́A�O���b�h�}�b�v�X�V*/
				pw.println(p[cnt].x + "\t" + p[cnt].y + "\t" + p2[cnt].x + "\t" + p2[cnt].y);
				pw2.println(obst[cnt].x + "\t" + obst[cnt].y + "\t" + obst2[cnt].x + "\t" + obst2[cnt].y);
				pw3.println(robo.location[cnt].x + "\t" + robo.location[cnt].y);
				gm.UpdateGrid(robo.location[cnt],floorFlag); //���{�b�g�̑��s�O�������m��ɍX�V
				gm.UpdateGrid(obst[cnt],obstFlag); //�����Q���ʒu����Q���m��ɍX�V
				gm.UpdateGrid(obst2[cnt],obstFlag); //����
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
		
		gm.PrintGrid(); //�O���b�h�}�b�v�̏����t�@�C���o�́ix,y,��Q���m���j
	}
}
