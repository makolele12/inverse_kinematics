#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <iostream>
#define ER 0.001
#define PI acos(-1.0)
using namespace std;

//Formato
void gotoxy(int x, int y);



void posiciones(double P[9][3],double th[9],double d[5]);
void lectura(double Q[3],double th[9]);
void posicionest(double P[9][3],double th[9],double d[5],int i);
void matrizmultiplicacion(double m1[9][4][4], double m2[9][4][4],double m3[9][4][4],int i);
double angulo(double vector1[9][3],double vector2[9][3], int i, double P[9][3],double Q[3],	double vectorfinal[9][3]);


void main()
{
	double th[9];	// ángulos de los eslabones 8
	double d[5]={0,126,98,105,113};	//medidas de los eslabones  4
	
	double P[9][3];		//Punto inicial del robot en cada eslabón (la posición cero del vector no se está contando).
	double Pt[9][3];	//Punto de prueba para generar plano
	double Q[3];		//Punto a alcanzar
	double vector1[9][3];	//Del ultimo eslabón al efector final
	double vector2[9][3];   //Del ultimo eslabón al efector final temporal
	double vectorfinal[9][3];	//Del ultimo eslabón a la proyeccion del punto a alcanzar sobre el plano
	double unitario1[3],unitario2[3]; //Creados para comprobar la dirección de giro
	double correcto[3];					//Almacena el error en xyz

	int i,c,y,ct; //ct cuenta las veces que se iteró.

	ct=0; 
	for(c=0;c<3;c++)
	{
		correcto[c]=1;
	}

	double magnitud1,magnitud2,angulotemporal;
	
	gotoxy(3,0);
	printf("Programa para obtener la cinematica inversa para el brazo de un robot NAO \n");
	gotoxy(23,10);
	printf("Autor: Marco Antonio Peredo Tiburcio \n");
	gotoxy(0,23);
	printf("Version: 1.0 (15/10/2013)\n");
	system("pause");
	system("cls");

	lectura(Q,th);
	posiciones(P,th,d);		//Obtiene las posiciones de cada eslabón.
	
	printf("\n Posicion xyz inicial del efector final:");
	for(c=0;c<3;c++)
	{
		printf("\n%lf",P[8][c]);
	}
	printf("\n\n");

	while(correcto[0]>ER || correcto[1]>ER || correcto[2]>ER) //mientras al menos 1 sea > 0.001 el ciclo sigue
	{
		ct++;
		for(i=7;i>0;i--)
		{
	
			posiciones(P,th,d);		//Obtiene las posiciones de cada eslabón.
			posicionest(Pt,th,d,i);  //Calcula un nuevo punto temporal para generar el plano mediante un cambio de angulos por pi radianes.
			for(c=0;c<3;c++)		//Crea los dos vectores necesarios para generar el plano.
			{
				vector1[i][c]=P[8][c]-P[i][c]; //Vector al efector final
				vector2[i][c]=Pt[8][c]-P[i][c];//Vector al efector final de prueba
			}

			angulotemporal = angulo(vector1,vector2,i,P,Q,vectorfinal);
			th[i] = th[i] + angulotemporal;
			
			posiciones(P,th,d);					//Obtiene las posiciones de cada eslabón nuevamente.
			for(y=0;y<3;y++)					
			{
				vector1[i][y]=P[8][y]-P[i][y];  //Vector a nueva posición del efector final
			}
			
			//Mediante vectores unitarios comprueba que la suma del ángulo haya sido correcta
			magnitud1 = pow(pow(vector1[i][0],2) + pow(vector1[i][1],2) + pow(vector1[i][2],2),0.5);
			magnitud2 = pow(pow(vectorfinal[i][0],2) + pow(vectorfinal[i][1],2) + pow(vectorfinal[i][2],2),0.5);
			for(c=0;c<3;c++)
			{
				unitario1[c] = vector1[i][c] / magnitud1;
				unitario2[c] = vectorfinal[i][c] / magnitud2;
			}
			if(unitario1[0]==unitario2[0] && unitario1[1]==unitario2[1] && unitario1[2]==unitario2[2]){}
			
			else th[i] = th[i]- (2*angulotemporal); //Era suma no resta, por lo tanto se resta 2 veces ya que se había sumado

			posiciones(P,th,d);					//Obtiene las posiciones de cada eslabón nuevamente.
					
		}

		for(c=0;c<3;c++)
		{
			correcto[c] = P[8][c] - Q[c];
			if(correcto[c]<0) correcto[c] = correcto[c] * -1;
		}
		
	}

	for(c=1;c<8;c++) //Ajustar ángulos a una escala de entre -360 y 360 y después ajustarlos entre 0 y 360
	{
		while(th[c] > (2*PI))
		{
			th[c] = th[c]- (2*PI);
		}

		while(th[c] < -(2*PI))
		{
			th[c] = th[c]+ (2*PI);
		}

		if(th[c]<0) th[c] = th[c] + 2*PI; 
	}


	printf("\n Angulo de cada eslabon, calculado en %i iteracion(es):",ct);
	for(c=1;c<9;c++)
	{
		printf("\nth%i: %lf",c,(th[c]*180)/PI);
	}
	printf("\n\n\n");
	printf("\n Posicion xyz final del efector final:");
	for(c=0;c<3;c++)
	{
		printf("\n%lf",P[8][c]);
	}
	printf("\n");
	system("pause");
	
}

void posiciones(double P[9][3],double th[9],double d[5])
{
	int c,y;

	double T[9][4][4];

	double a[8]={0,0,0,d[1],0,0,0,0};
	double al[8]={0,-PI/2,-PI/2,-PI/2,PI/2,-PI/2,PI/2,-PI/2};
	double D[8]={0,0,0,d[2],0,d[3],0,d[4]};
	double TH[8]={th[1],th[2]-(PI/2),th[3],th[4],th[5],th[6],th[7],th[8]};

	for(c=0;c<8;c++) //Crear matrices de Rotación y traslación.
	{
		T[c+1][0][0]= cos(TH[c]);
		T[c+1][0][1]= -sin(TH[c]);
		T[c+1][0][2]= 0;
		T[c+1][0][3]= a[c];

		T[c+1][1][0]= (sin(TH[c]))*cos(al[c]);
		T[c+1][1][1]= (cos(TH[c]))*cos(al[c]);
		T[c+1][1][2]=  -sin(al[c]);
		T[c+1][1][3]= (-sin(al[c]))*D[c];

		T[c+1][2][0]= (sin(TH[c]))*sin(al[c]);
		T[c+1][2][1]= (cos(TH[c]))*sin(al[c]);
		T[c+1][2][2]= cos(al[c]);
		T[c+1][2][3]= (cos(al[c]))*D[c];

		T[c+1][3][0]= 0;
		T[c+1][3][1]= 0;
		T[c+1][3][2]= 0;
		T[c+1][3][3]= 1;
	}

	
	double multi[9][4][4];

	for(c=0;c<4;c++)
	{
		for(y=0;y<4;y++)
		{
			multi[1][c][y]=T[1][c][y];
		}
	}
		
	
	for(c=2;c<9;c++)
	{
		matrizmultiplicacion(multi,T,multi,c); //Multiplica las matrices, para tener todas las transformaciones.

	}

	for(c=1;c<9;c++)
	{
		for(y=0;y<3;y++)
		{
			P[c][y]=multi[c][y][3]; //Almacena únicamente el vector posición de la última matriz.
		}
	}
}

void posicionest(double P[9][3],double th[9],double d[5],int i)
{
	int c,y;

	double T[9][4][4];
	double th1[9]={th[0],th[1],th[2],th[3],th[4],th[5],th[6],th[7],th[8]};	
	
	//Decide cual ángulo será el que se modificará temporalmente para así obtener otro punto y crear el plano
	if(i==1) th1[1] = th[1]+PI/2;
	if(i==2) th1[2] = th[2]+PI/2;
	if(i==3) th1[3] = th[3]+PI/2;
	if(i==4) th1[4] = th[4]+PI/2;
	if(i==5) th1[5] = th[5]+PI/2;
	if(i==6) th1[6] = th[6]+PI/2;
	if(i==7) th1[7] = th[7]+PI/2;


	double a[8]={0,0,0,d[1],0,0,0,0};
	double al[8]={0,-PI/2,-PI/2,-PI/2,PI/2,-PI/2,PI/2,-PI/2};
	double D[8]={0,0,0,d[2],0,d[3],0,d[4]};
	double TH[8]={th1[1],th1[2]-(PI/2),th1[3],th1[4],th1[5],th1[6],th1[7],th1[8]};

	for(c=0;c<8;c++)
	{
		T[c+1][0][0]= cos(TH[c]);
		T[c+1][0][1]= -sin(TH[c]);
		T[c+1][0][2]= 0;
		T[c+1][0][3]= a[c];

		T[c+1][1][0]= (sin(TH[c]))*cos(al[c]);
		T[c+1][1][1]= (cos(TH[c]))*cos(al[c]);
		T[c+1][1][2]=  -sin(al[c]);
		T[c+1][1][3]= (-sin(al[c]))*D[c];

		T[c+1][2][0]= (sin(TH[c]))*sin(al[c]);
		T[c+1][2][1]= (cos(TH[c]))*sin(al[c]);
		T[c+1][2][2]= cos(al[c]);
		T[c+1][2][3]= (cos(al[c]))*D[c];

		T[c+1][3][0]= 0;
		T[c+1][3][1]= 0;
		T[c+1][3][2]= 0;
		T[c+1][3][3]= 1;

	}

	
	double multi[9][4][4];

	for(c=0;c<4;c++)
	{
		for(y=0;y<4;y++)
		{
			multi[1][c][y]=T[1][c][y];
		}
	}

	for(c=2;c<9;c++)
	{
		matrizmultiplicacion(multi,T,multi,c);

	}
	
	for(c=1;c<9;c++)
	{
		for(y=0;y<3;y++)
		{
			P[c][y]=multi[c][y][3];
		}
	}

}

void matrizmultiplicacion(double m1[9][4][4], double m2[9][4][4],double m3[9][4][4],int i)
{
	//Multiplica dos matrices de 4x4
	int c,y;
	for(c=0;c<4;c++)
	{
		for(y=0;y<4;y++)
		{
			m3[i][c][y]= m1[i-1][c][0]*m2[i][0][y] + m1[i-1][c][1]*m2[i][1][y] + m1[i-1][c][2]*m2[i][2][y] + m1[i-1][c][3]*m2[i][3][y];
		}
	}
}

double angulo(double vector1[9][3],double vector2[9][3], int i,double P[9][3],double Q[3],double vectorfinal[9][3])
{
	double vectornormal[3];
	double d;
	double proyeccion[3];
	double prodpunto;
	double magnitud1,magnitud2;
	double angulo;
	int x=0;

	//Producto cruz para obtener el vector normal al plano
	vectornormal[0]=    (vector1[i][1]*vector2[i][2]) - (vector1[i][2]*vector2[i][1]); 
	vectornormal[1]= -( (vector1[i][0]*vector2[i][2]) - (vector1[i][2]*vector2[i][0]) );
	vectornormal[2]=    (vector1[i][0]*vector2[i][1]) - (vector1[i][1]*vector2[i][0]);

	//Obtener el valor de d para la ecuación del plano
	d= -(vectornormal[0]*P[8][0] + vectornormal[1]*P[8][1] + vectornormal[2]*P[8][2]);
	
	double a=vectornormal[0];
	double b=vectornormal[1];
	double c=vectornormal[2];

	if(vectornormal[0]==0 && vectornormal[1]==0 && vectornormal[2]==0) //Comprobar que el efector final no se encuentre ya alineado con proyeccion de Q
	{
		double p=Q[0];
		double q=Q[1];
		double r=Q[2];
		
		double t = (-d-(a*p)-(b*q)-(c*r)) / (pow(a,2)+pow(b,2)+pow(c,2));
	
		proyeccion[0]= p + a*t;
		proyeccion[1]= q + b*t;
		proyeccion[2]= r + c*t;
	
		for(x=0;x<3;x++)
		{
			vectorfinal[i][x] = proyeccion[x] - P[i][x];
		}
		
		angulo=0; //Inmediatamente se puede afirmar.
	}	
	else
	{
		double p=Q[0];
		double q=Q[1];
		double r=Q[2];
		
		double t = (-d-(a*p)-(b*q)-(c*r)) / (pow(a,2)+pow(b,2)+pow(c,2));
	
		proyeccion[0]= p + a*t;
		proyeccion[1]= q + b*t;
		proyeccion[2]= r + c*t;
	
		for(x=0;x<3;x++)
		{
			vectorfinal[i][x] = proyeccion[x] - P[i][x];
		}
		
		//Calcular el angulo entre el vector1 y el vectorfinal
		prodpunto = (vector1[i][0]*vectorfinal[i][0] + vector1[i][1]*vectorfinal[i][1] + vector1[i][2]*vectorfinal[i][2]);
		magnitud1 = pow(pow(vector1[i][0],2) + pow(vector1[i][1],2) + pow(vector1[i][2],2),0.5);
		magnitud2 = pow(pow(vectorfinal[i][0],2) + pow(vectorfinal[i][1],2) + pow(vectorfinal[i][2],2),0.5);
		angulo = acos(prodpunto/(magnitud1*magnitud2));
	}

	return angulo;

}

void lectura(double Q[3],double th[9])
{
	int c;
	printf("\n Escribe las coordenadas xyz a alcanzar por el efector final: \n");
	for(c=0;c<3;c++)
	{
		scanf("%lf",&Q[c]);
	}
	
	printf("\n Escribe los angulos iniciales de cada eslabon: \n");
	for(c=1;c<9;c++)
	{
		printf(" Th%i: ",c);
		scanf("%lf",&th[c]);
		th[c] = (PI*th[c])/180;
	}

}

void gotoxy(int x, int y)
{
	COORD coord;
	coord.X = x; coord.Y = y;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleCursorPosition(hStdOut, coord);
}