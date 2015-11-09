import java.io.*;
import java.awt.geom.*;

public class Grid {
	private Point2D.Double zahyou = new Point2D.Double(); //�O���b�h�ʒu
	private int status; //�O���b�h���(0:������, 1:��Ԋm��, 2:��Q���m��, 3:���蒆)
	private double exp; //��Q�����ǂ����̊m��(0:��ԁ`1:��Q��) �����l��0.5
	
	public Grid(double x,double y){
		zahyou.x = x; zahyou.y = y;
		status = 0;
		exp = 0.5;
	}
	
	public void UpdateGrid(int flag){
		switch(flag){
		case 1:
			status = 1;
			exp = 0;
			break;
		case 2:
			status = 2;
			exp = 1;
			break;
		case 3:
			if(exp == 1){
				break;
			}
			else{
				status = 3;
				exp += 0.1;
				if(exp == 1){
					status = 2;
				}
				break;
			}
		}
	}
	
	public void PrintGrid(PrintWriter pw){
		pw.println(zahyou.x + "\t" + zahyou.y + "\t" + exp);
	}
}
