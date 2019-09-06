#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#include"ndt.h"
#include"algebra.h"

#define E_THETA 0.00000001

extern int point_num;
double qdd[3][3][2];
double qd[3][2];
double qd3[6][3];
double qde3[3][6][3];
double qdd3[6][6][3];
void set_sincos(double a,double b,double g,double sc_d[3][3][3]);
void set_sincos2(double a,double b,double g,double sc[3][3]);


/*ヘッセ行列の計算もう少し楽できるかも。
一次微分は点の数だけ計算して、ヘッセ行列はそれの差分から求める。*/
double calc_summand3d( PointPtr p,NDPtr nd,PosturePtr pose, 
		       double *g,double Jac[6][3]){ 
  double a[3];
  double e;
  double q[3];

  int i;

  /*qの計算*/ 
  q[0] = p->x - nd->mean.x;
  q[1] = p->y - nd->mean.y;
  q[2] = p->z - nd->mean.z; 
 
  /*expの計算*/
  e = probability_on_ND(nd, p->x, p->y, p->z); 
     
  /*  */
  a[0] =q[0] * nd->inv_covariance[0][0] 
    +   q[1] * nd->inv_covariance[1][0]  
    +   q[2] * nd->inv_covariance[2][0];
  a[1] =q[0] * nd->inv_covariance[0][1] 
    +   q[1] * nd->inv_covariance[1][1]  
    +   q[2] * nd->inv_covariance[2][1];
  a[2] =q[0] * nd->inv_covariance[0][2] 
    +   q[1] * nd->inv_covariance[1][2] 
    +   q[2] * nd->inv_covariance[2][2]; 

  g[0] = (a[0]*Jac[0][0] + a[1]*Jac[0][1] + a[2]*Jac[0][2])*e;
  g[1] = (a[0]*Jac[1][0] + a[1]*Jac[1][1] + a[2]*Jac[1][2])*e;
  g[2] = (a[0]*Jac[2][0] + a[1]*Jac[2][1] + a[2]*Jac[2][2])*e;
  g[3] = (a[0]*Jac[3][0] + a[1]*Jac[3][1] + a[2]*Jac[3][2])*e;
  g[4] = (a[0]*Jac[4][0] + a[1]*Jac[4][1] + a[2]*Jac[4][2])*e;
  g[5] = (a[0]*Jac[5][0] + a[1]*Jac[5][1] + a[2]*Jac[5][2])*e;
 
  
  //  for(i = 0;i < 6;i++)g[i] = g[i]*e;
  
  return e;
}

int check_Hessian(double H[3][3]){
  int i,j;
  for(i = 0;i < 3;i++){
    for(j = 0;j < 3;j++){
      if(H[i][j] < -0.0001)return 0;
    }
  }
  return 1;
}

void save_data(PointPtr scan, int num, PosturePtr pose){
 double sc[3][3],x,y,z;
 int i;
 FILE *scanfile;
 Point p;
 scanfile = fopen("scan","w");

 set_sincos2(pose->theta, pose->theta2, pose->theta3,sc);
 for(i = 0;i < num; i++){
      
    x   = scan[i].x;
    y   = scan[i].y;
    z   = scan[i].z;  
   
    p.x = x*sc[0][0] + y*sc[0][1]+ z*sc[0][2] + pose->x;
    p.y = x*sc[1][0] + y*sc[1][1]+ z*sc[1][2] + pose->y;
    p.z = x*sc[2][0] + y*sc[2][1]+ z*sc[2][2] + pose->z;

    fprintf(scanfile,"%f %f %f \n",p.x,p.y,p.z);
 }
 fclose(scanfile);
}

void depth(PointPtr scan, int num, PosturePtr pose){
 double sc[3][3],x,y,z;
 int i;

 Point p;


 set_sincos2(pose->theta, pose->theta2, pose->theta3,sc);
 for(i = 0;i < num; i++){
      
    x   = scan[i].x;
    y   = scan[i].y;
    z   = scan[i].z;  
   
    p.x = x*sc[0][0] + y*sc[0][1]+ z*sc[0][2] + pose->x;
    p.y = x*sc[1][0] + y*sc[1][1]+ z*sc[1][2] + pose->y;
    p.z = x*sc[2][0] + y*sc[2][1]+ z*sc[2][2] + pose->z;
 }
}


double adjust3d(PointPtr scan, int num, PosturePtr initial,int target){
  double gsum[6], gesum[6][6],Hsum[6][6],Hsumh[6][6],Hinv[6][6],g[6],ge[6][6];
  double sc[3][3],sc_d[3][3][3],sc_dd[3][3][3][3],sce[3][3][3];
  double *work;
  double esum = 0,gnum = 0;
  NDPtr nd[8];
  int i,j,n,m,k;
  double x,y,z;//,sa,ca,sb,cb,sg,cg;
  PosturePtr pose;
  Point p,pe[6];
  PointPtr scanptr;
  int inc;

  /*initialize*/
  gsum[0] = 0; gsum[1] = 0; gsum[2] = 0;
  gsum[3] = 0; gsum[4] = 0; gsum[5] = 0;

  for(n = 0;n < 6;n++){
    gesum[n][0] = 0; gesum[n][1] = 0; gesum[n][2] = 0;
    gesum[n][3] = 0; gesum[n][4] = 0; gesum[n][5] = 0;
  }

  zero_matrix6d(Hsum);
  zero_matrix6d(Hsumh);
  pose = initial;
  
  /*calcurate*/
  set_sincos( pose->theta, pose->theta2, pose->theta3,sc_d);
  set_sincos( pose->theta +E_THETA, pose->theta2, pose->theta3, sc_dd[0]);
  set_sincos( pose->theta, pose->theta2 +E_THETA, pose->theta3, sc_dd[1]);
  set_sincos( pose->theta, pose->theta2, pose->theta3 +E_THETA, sc_dd[2]);
  
  set_sincos2(pose->theta, pose->theta2, pose->theta3,sc);
  set_sincos2(pose->theta +E_THETA, pose->theta2, pose->theta3, sce[0]); 
  set_sincos2(pose->theta, pose->theta2 +E_THETA, pose->theta3, sce[1]);
  set_sincos2(pose->theta, pose->theta2, pose->theta3 +E_THETA, sce[2]);
  
  qd3[0][0] = 1;qd3[0][1] = 0;qd3[0][2] = 0;
  qd3[1][0] = 0;qd3[1][1] = 1;qd3[1][2] = 0;
  qd3[2][0] = 0;qd3[2][1] = 0;qd3[2][2] = 1;

  for(n = 0;n < 3;n++){
    qde3[n][0][0] = 1;qde3[n][0][1] = 0;qde3[n][0][2] = 0;
    qde3[n][1][0] = 0;qde3[n][1][1] = 1;qde3[n][1][2] = 0;
    qde3[n][2][0] = 0;qde3[n][2][1] = 0;qde3[n][2][2] = 1;
  }

  for(n = 0;n < 6;n++){for(m = 0;m < 6;m++){ for(k = 0;k < 3;k++){
    qdd3[n][m][k] = 0;
  }}}
  
  if(target > 1){
    inc = 1;
    //    printf("high rezolution\n");
  }else{ 
    inc = 10;
  }

  scanptr = scan;

  for(i = 0;i < num; i+=inc){      
    x   = scanptr->x;
    y   = scanptr->y;
    z   = scanptr->z;  
    scanptr+=inc;
    p.x = x*sc[0][0] +y*sc[0][1]+ z*sc[0][2] + pose->x;
    p.y = x*sc[1][0] +y*sc[1][1]+ z*sc[1][2] + pose->y;
    p.z = x*sc[2][0] +y*sc[2][1]+ z*sc[2][2] + pose->z;
    
    pe[0].x = p.x + E_THETA; pe[0].y = p.y;           pe[0].z = p.z;
    pe[1].x = p.x;           pe[1].y = p.y + E_THETA; pe[1].z = p.z;
    pe[2].x = p.x;           pe[2].y = p.y;           pe[2].z = p.z + E_THETA;
    for(j = 0;j < 3;j++){
      pe[j+3].x = x*sce[j][0][0] +y*sce[j][0][1]+ z*sce[j][0][2] + pose->x;
      pe[j+3].y = x*sce[j][1][0] +y*sce[j][1][1]+ z*sce[j][1][2] + pose->y;
      pe[j+3].z = x*sce[j][2][0] +y*sce[j][2][1]+ z*sce[j][2][2] + pose->z;
    }

    if(target){
      if(!get_ND(&p,nd))continue;     
    }else{ 
      if(!get_ND2(&p,nd))continue;
    }

    /*qの一次微分(変化する場所のみ)*/
    work = sc_d;
    for(m = 0;m < 3;m++){
      for(k = 0;k < 3;k++){
	qd3[m+3][k] =   x* (*work)+y* (*(work+1))+z*(*(work+2));
	work+=3;
      }
    }
    
    for(n = 0;n < 3;n++){
      work = sc_dd[n];
      for(m = 0;m < 3;m++){
	for(k = 0;k < 3;k++){
	  qde3[n][m+3][k] =   x* (*work)+y* (*(work+1))+z*(*(work+2));
	  work+=3;
	}
      }
    }

    for(j = 0;j < 8;j++){
      if(nd[j]->num > 5){
       
	esum += calc_summand3d(&p,nd[j],pose,g,qd3);	
	
	gsum[0] += g[0];
	gsum[1] += g[1];
	gsum[2] += g[2];
	gsum[3] += g[3];
	gsum[4] += g[4];
	gsum[5] += g[5];
	
	for(n = 0;n < 6;n++){	
	  if(n < 3)
	    calc_summand3d(&pe[n], nd[j], pose, ge[n], qd3); 
	  else 
	    calc_summand3d(&pe[n], nd[j], pose, ge[n], qde3[n-3]); 
	  gesum[n][0] += ge[n][0];
	  gesum[n][1] += ge[n][1];
	  gesum[n][2] += ge[n][2];
	  gesum[n][3] += ge[n][3];
	  gesum[n][4] += ge[n][4];
	  gesum[n][5] += ge[n][5];
	}
	
	gnum++;	
      }
    }
  }

  for(n = 0; n < 6; n++){
    for(m = 0; m < 6; m++){  
      Hsumh[m][n] =  (gesum[n][m]-gsum[m])/E_THETA;
    }
  }
    
  ginverse_matrix6d(Hsumh,Hinv);
  
  pose->x     -= ( Hinv[0][0]*gsum[0]+ Hinv[0][1]*gsum[1]+ Hinv[0][2]*gsum[2]+
	           Hinv[0][3]*gsum[3]+ Hinv[0][4]*gsum[4]+ Hinv[0][5]*gsum[5]);
  pose->y     -= ( Hinv[1][0]*gsum[0]+ Hinv[1][1]*gsum[1]+ Hinv[1][2]*gsum[2]+
	           Hinv[1][3]*gsum[3]+ Hinv[1][4]*gsum[4]+ Hinv[1][5]*gsum[5]);
  pose->z     -= ( Hinv[2][0]*gsum[0]+ Hinv[2][1]*gsum[1]+ Hinv[2][2]*gsum[2]+
	           Hinv[2][3]*gsum[3]+ Hinv[2][4]*gsum[4]+ Hinv[2][5]*gsum[5]);
  pose->theta -= ( Hinv[3][0]*gsum[0]+ Hinv[3][1]*gsum[1]+ Hinv[3][2]*gsum[2]+
	           Hinv[3][3]*gsum[3]+ Hinv[3][4]*gsum[4]+ Hinv[3][5]*gsum[5]);
  pose->theta2-= ( Hinv[4][0]*gsum[0]+ Hinv[4][1]*gsum[1]+ Hinv[4][2]*gsum[2]+
	           Hinv[4][3]*gsum[3]+ Hinv[4][4]*gsum[4]+ Hinv[4][5]*gsum[5]);
  pose->theta3-= ( Hinv[5][0]*gsum[0]+ Hinv[5][1]*gsum[1]+ Hinv[5][2]*gsum[2]+
		   Hinv[5][3]*gsum[3]+ Hinv[5][4]*gsum[4]+ Hinv[5][5]*gsum[5]);
  

  return esum;
}


void set_sincos2(double a,double b,double g,double sc[3][3]){
  double sa,ca,sb,cb,sg,cg;
  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);
  
  sc[0][0] =  ca*cb*cg-sa*sg;
  sc[0][1] = -ca*cb*sg -sa*cg;
  sc[0][2] =  ca*sb;
  sc[1][0] =  sa*cb*cg+ca*sg;
  sc[1][1] = -sa*cb*sg+ca*cg;
  sc[1][2] =  sa*sb;
  sc[2][0] = -sb*cg;
  sc[2][1] =  sb*sg;
  sc[2][2] =  cb;

}

void set_sincos (double a,double b,double g,double sc[3][3][3]){
  double sa,ca,sb,cb,sg,cg;
  
  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  //sc[tx ty tz a b g][x y z][x+y+z ]
  sc[0][0][0] = -sa*cb*cg-ca*sg;
  sc[0][0][1] =  sa*cb*sg-ca*cg;
  sc[0][0][2] = -sa*sb;
  sc[0][1][0] =  ca*cb*cg-sa*sg;
  sc[0][1][1] = -ca*cb*sg-sa*cg;
  sc[0][1][2] =  ca*sb;
  sc[0][2][0] =  0;
  sc[0][2][1] =  0;
  sc[0][2][2] =  0;

  sc[1][0][0] = -ca*sb*cg;
  sc[1][0][1] =  ca*sb*sg;
  sc[1][0][2] =  ca*cb;
  sc[1][1][0] = -sa*sb*cg;
  sc[1][1][1] =  sa*sb*sg;
  sc[1][1][2] =  sa*cb;
  sc[1][2][0] = -cb*cg;
  sc[1][2][1] =  cb*sg;
  sc[1][2][2] = -sb;
  

  sc[2][0][0] = -ca*cb*sg-sa*cg;
  sc[2][0][1] = -ca*cb*cg+sa*sg;
  sc[2][0][2] =  0;
  sc[2][1][0] = -sa*cb*sg+ca*cg;
  sc[2][1][1] = -sa*cb*cg-ca*sg;
  sc[2][1][2] =  0;
  sc[2][2][0] =  sb*sg;
  sc[2][2][1] =  sb*cg;
  sc[2][2][2] =  0;
}
/*
void set_sincos(double a,double b,double g,double sc_d[3][3][3]){
  double sa,ca,sb,cb,sg,cg;

  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  sc_d[0][0][0] = -sa*cb*cg - ca*sg;
  sc_d[0][0][1] =  ca*sb*sg - sa*cg;
  sc_d[0][0][2] = -sa*sb;

  sc_d[1][0][0] =  ca*sb*cg;
  sc_d[1][0][1] =  ca*sb*sg;
  sc_d[1][0][2] =  ca*cb;
  
  sc_d[2][0][0] =  -ca*cb*sg-sa*cg;
  sc_d[2][0][1] =  -ca*cb*cg+sa*sg;
  sc_d[2][0][2] =  0;


  sc_d[0][1][0] =  ca*cb*cg - sa*sg;
  sc_d[0][1][1] = -ca*cb*sg - sa*cg;
  sc_d[0][1][2] =  ca*sb;

  sc_d[1][1][0] =  -sa*sb*cg;
  sc_d[1][1][1] =  sa*sb*sg;
  sc_d[1][1][2] =  sa*cb;
  
  sc_d[2][1][0] =  -sa*cb*sg+ca*cg;
  sc_d[2][1][1] =  -sa*cb*cg-ca*sg;
  sc_d[2][1][2] =  0;

  sc_d[0][2][0] =  0;
  sc_d[0][2][1] =  0;
  sc_d[0][2][2] =  0;

  sc_d[1][2][0] =  -cb*cg;
  sc_d[1][2][1] =  cb*sg;
  sc_d[1][2][2] =  -sb;
  
  sc_d[2][2][0] =  sb*sg;
  sc_d[2][2][1] =  sb*cg;
  sc_d[2][2][2] =  0;

}
 
*/
