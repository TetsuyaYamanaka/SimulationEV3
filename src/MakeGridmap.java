import java.io.*;
import java.awt.geom.*;

public class MakeGridmap { //�P�ʌn��m
	double x,y;
	int i,xcnt,ycnt;
	int index_x,index_y;
	final double resolution = 0.01; //�O���b�h�Ԋu
	final int width = 101; //x�����̃O���b�h��
	final int height = 101; //y�����̃O���b�h��
	final int gridMax = width*height;
	final double centerX = 0.5; //���S��x���W
	final double centerY = 0; //���S��y���W
	final double startX = 0;
	final double startY = -0.5;
	Grid[] gm = new Grid[gridMax]; //�O���b�h�}�b�v
	
	MakeGridmap(){ //�R���X�g���N�^
		for(ycnt=0;ycnt<height;ycnt++){
			y = ycnt*resolution+startY;
			for(xcnt=0;xcnt<width;xcnt++){
				x = xcnt*resolution+startX;
				i = height*ycnt + xcnt;
				gm[i] = new Grid(x,y); //�O���b�h�}�b�v�I�u�W�F�N�g�̏�����
			}
		}
	}
	
	public void UpdateGrid(Point2D.Double world,int flag){ //�O���b�h�̏�Ԃ��X�V���郁�\�b�h
		index_x = (int)((world.x - centerX)/resolution - width/2);
		index_y = (int)((world.y - centerY)/resolution - height/2);
		i = (height*index_y + index_x) + (gridMax - 1);
		gm[i].UpdateGrid(flag);
	}
	
	public void SetFloorStatus(Point2D.Double robo,Point2D.Double obst){
		int i;
		Point2D.Double b,c,grid_i;
		double otoa;
		double slope,slope2,slope3; //�X��
		double ic,ic2,ic3; //�ؕ�
		double f,f2,f3; //�̈������Ă���֐�
		b = new Point2D.Double(); c = new Point2D.Double();
		grid_i = new Point2D.Double();
		if(robo.x == obst.x){
			index_x = (int)((obst.x - centerX)/resolution - width/2);
			index_y = (int)((obst.y - centerY)/resolution - height/2);
			i = (height*index_y + index_x) + (gridMax - 1);
			
			b.setLocation(robo.x,robo.y+1);
			c.setLocation(robo.x,robo.y-1);
			slope = (b.y-robo.y)/(b.x-robo.x);
			slope2 = (c.y-robo.y)/(c.x-robo.x);
			ic = robo.y - slope*robo.x;
			ic2 = robo.y - slope2*robo.x;
			
			for(i=0;i<gridMax;i++){
				grid_i = gm[i].ReturnZahyou();
				f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = obst.x;
				if(f2<grid_i.y && grid_i.y<f && grid_i.x<f3){
					gm[i].UpdateGrid(1); //�̈�Ɉ͂܂�Ă���Ƃ�������m��ɂ���
				}
			}
		}
		else if(robo.y == obst.y){
			b.setLocation(robo.x-1,robo.y);
			c.setLocation(robo.x+1,robo.y);
			slope = (b.y-robo.y)/(b.x-robo.x);
			slope2 = (c.y-robo.y)/(c.x-robo.x);
			ic = robo.y - slope*robo.x;
			ic2 = robo.y - slope2*robo.x;
			
			for(i=0;i<gridMax;i++){
				grid_i = gm[i].ReturnZahyou();
				f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = obst.y;
				if(f<grid_i.y && f2<grid_i.y && grid_i.y<f3){
					gm[i].UpdateGrid(1); //�̈�Ɉ͂܂�Ă���Ƃ�������m��ɂ���
				}
			}
		}
		else{
			otoa = (robo.y - obst.y)/(robo.x - obst.y);
			if(otoa < 0){
				b.setLocation(robo.x-1,robo.y+1);
				c.setLocation(robo.x+1,robo.y-1);
				slope = (b.y-robo.y)/(b.x-robo.x);
				slope2 = (c.y-robo.y)/(c.x-robo.x);
				slope3 = (c.y-b.y)/(c.x-b.x);
				ic = robo.y - slope*robo.x;
				ic2 = robo.y - slope2*robo.x;
				ic3 = b.y - slope3*b.x;
				
				for(i=0;i<gridMax;i++){
					grid_i = gm[i].ReturnZahyou();
					f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = slope3*grid_i.x + ic3;
					if(f2<grid_i.y && grid_i.y<f && grid_i.y<f3){
						gm[i].UpdateGrid(1); //�̈�Ɉ͂܂�Ă���Ƃ�������m��ɂ���
					}
				}
			}
			else{
				b.setLocation(robo.x-1,robo.y-1);
				c.setLocation(robo.x+1,robo.y+1);
				slope = (b.y-robo.y)/(b.x-robo.x);
				slope2 = (c.y-robo.y)/(c.x-robo.x);
				slope3 = (c.y-b.y)/(c.x-b.x);
				ic = robo.y - slope*robo.x;
				ic2 = robo.y - slope2*robo.x;
				ic3 = b.y - slope3*b.x;
				
				for(i=0;i<gridMax;i++){
					grid_i = gm[i].ReturnZahyou();
					f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = slope3*grid_i.x + ic3;
					if(f<grid_i.y && grid_i.y>f2 && grid_i.y<f3){
						gm[i].UpdateGrid(1); //�̈�Ɉ͂܂�Ă���Ƃ�������m��ɂ���
					}
				}
			}
		}
	}
	
	public void GaussianUpdate(){ //�K�E�X�֐��ɂ��m���̏d�݂Â�
		Point2D.Double obst; //�ϑ��_�̍��W
		double sigma = ((1+Math.sqrt(2))*resolution)/2; //���ϒl�A�΍�
		double f;
		int i,j,cnt = 0;
		double lentoObst = 999; //�ϑ��_�Ƃ̋���
		int[] obstGrid = new int[100]; //�ϑ��_�O���b�h�C���f�b�N�X�̔z��
		int mostNear; //�ŋߖT�̊ϑ��_�O���b�h�l
		
		for(i=0;i<gridMax;i++){
			if(gm[i].ReturnExp() == 1){
				obstGrid[cnt] = i; //�ϑ��_�O���b�h�C���f�b�N�X��ۑ�
				cnt++;
			}
		}
		
		for(i=0;i<gridMax;i++){
			for(j=0;j<cnt;j++){
				obst = gm[obstGrid[j]].ReturnZahyou();
				if(obst.distance(gm[i].ReturnZahyou()) < lentoObst){
					lentoObst = obst.distance(gm[i].ReturnZahyou());
				}
			}
			f = Math.exp(-Math.pow(lentoObst,2)/(2*Math.pow(sigma,2)))/(Math.sqrt(2*Math.PI)*sigma);
			gm[i].UpdateGrid(f);
		}
	}
	
	public void PrintGrid(){ //�O���b�h�}�b�v�̏��ix,y,��Q���̊m���j���t�@�C���o�͂��郁�\�b�h
		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("Grid.dat")));
			for(i=0;i<gridMax;i++){
				gm[i].PrintGrid(pw);
			}
			pw.close();
		} catch(IOException e){
			System.out.println(e);
		}		
	}
}
