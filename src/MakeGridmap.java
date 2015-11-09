import java.io.*;

public class MakeGridmap {
	double x,y;
	int i;
	int index_x,index_y,index;
	final double resolution = 1; //�O���b�h�Ԋu
	final int width = 101; //x�����̃O���b�h��
	final int height = 101; //y�����̃O���b�h��
	final int gridMax = width*height;
	final int centerX = 50; //���S��x���W
	final int centerY = 0; //���S��y���W
	Grid[] gm = new Grid[gridMax]; //�O���b�h�}�b�v
	
	public MakeGridmap(){ //�R���X�g���N�^
		for(y=-50;y<=50;y++){ //�O���b�h�}�b�v�̊e�v�f��������
			index_y = (int)((y-centerY)/resolution - height/2);
			for(x=0;x<=100;x++){
				index_x = (int)((x-centerX)/resolution - width/2);
				index = (width*index_x + index_y) + (gridMax - 1);
				gm[index] = new Grid(x,y);
			}
		}
	}
	
	public void UpdateGrid(double worldX,double worldY,int flag){ //�O���b�h�̏�Ԃ��X�V���郁�\�b�h
		index_x = (int)((worldX - centerX)/resolution - width/2);
		index_y = (int)((worldY - centerY)/resolution - height/2);
		index = (index_x*width+index_y) + (gridMax - 1);
		gm[index].UpdateGrid(flag);
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
