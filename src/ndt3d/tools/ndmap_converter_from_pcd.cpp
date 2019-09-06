#include <stdio.h>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>

#include "ndt.h"
#include "algebra.h"

NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

char g_ndmap_name[500];
double g_map_center_x, g_map_center_y, g_map_center_z, g_map_rotation;
int g_map_x, g_map_y, g_map_z;
double g_map_cellsize;
double g_ini_x, g_ini_y, g_ini_z, g_ini_roll, g_ini_pitch, g_ini_yaw;
int g_use_gnss;
double minx, maxx, miny, maxy, minz, maxz;

char *root_dir;

int load_ndt_ini(char *name);
NDMapPtr initialize_NDmap(void);
int add_point_map(NDMapPtr ndmap, PointPtr point);
void save_nd_map(char *name);

// read PCD file and add points to ndmap
void read_file(const char *root_dir_name, const char *dir_name, const char *file_name)
{
	char target_file_name[1024];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	sprintf(target_file_name, "%s%s/%s", root_dir_name, dir_name, file_name);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(target_file_name, *cloud) == -1)
	{
		printf("cannot read pcd file.\n");
	}
	else
	{
		printf("%s\n", target_file_name);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			Point p;
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			if (strcmp(dir_name, "Manually_Sence") != 0)
			{
				p.x -= g_map_center_x;
				p.y -= g_map_center_y;
				p.z -= g_map_center_z;
			}
			if (p.x < minx)	minx = p.x;
			if (p.x > maxx)	maxx = p.x;
			if (p.y < miny)	miny = p.y;
			if (p.y > maxy)	maxy = p.y;
			if (p.z < minz)	minz = p.z;
			if (p.z > maxz)	maxz = p.z;
			add_point_map(NDmap, &p);
		}
	}	
}

// into directory which contains PCD files
void into_dir(const char *root_dir_name, const char *dir_name)
{
	char target_dir_name[1024];
	DIR *dp;

	sprintf(target_dir_name, "%s%s/", root_dir_name, dir_name);
	dp = opendir(target_dir_name);
	if (dp != NULL)
	{
		struct dirent *dent;
		do
		{
			dent = readdir(dp);
			if (dent == NULL)
			{
//				printf("cannot find directories which contain PCD files.\n");
			}
 			else if (strcmp(dent->d_name, ".") != 0 && strcmp(dent->d_name, "..") != 0)
			{
//				std::cout << dent->d_name << std::endl;
				read_file(root_dir_name, dir_name, dent->d_name);
			}
		} while (dent != NULL);
		closedir(dp);
	}
}

// search root directory provided from AISAN
void build_ndmap(const char *root_dir_name)
{
	int	i, r;
	struct dirent **namelist;

	r = scandir(root_dir_name, &namelist, NULL, NULL);
	if (r == -1)
	{
//		perror("cannot find a root directory of PCD files.");
		exit(1);
	}
	for (i = 0; i < r; ++i)
	{
		if (strcmp(namelist[i]->d_name, ".") != 0 && strcmp(namelist[i]->d_name, "..") != 0)
		{
//			std::cout << namelist[i]->d_name << std::endl;
			into_dir(root_dir_name, namelist[i]->d_name);
		}
		free(namelist[i]);
	}
	free(namelist);
}

int main(int argc, char **argv)
{
	// argv[1] = hoge.ndt
	// argv[2] = /PCD/
	if (argc > 2)
	{
		if (!load_ndt_ini(argv[1]))
		{
			printf("\nsetting file error. please confirm a setting file configure\n\n");
			return 1;
		}
	}
	else
	{
		printf("\nplease set a setting file and root dir path of PCD files as --> %s hoge.ndt [fuga]/PCD/\n\n", argv[0]);
		return 1;
	}
	minx = miny = minz = 10000000.0;
	maxx = maxy = maxz = -10000000.0;
	root_dir = argv[2];
	NDmap = initialize_NDmap();
	build_ndmap(root_dir);
	save_nd_map(g_ndmap_name);
	double avex = (maxx + minx + 2.0 * g_map_center_x) / 2.0;
	double avey = (maxy + miny + 2.0 * g_map_center_y) / 2.0;
	double avez = (maxz + minz + 2.0 * g_map_center_z) / 2.0;
	printf("\n");
	printf("min: x = %lf, y = %lf, z = %lf\n", minx, miny, minz);
	printf("max: x = %lf, y = %lf, z = %lf\n", maxx, maxy, maxz);
	printf("diff: x = %lf, y = %lf, z = %lf\n", maxx - minx, maxy - miny, maxz - minz);
	printf("middle: x = %lf, y = %lf, z = %lf\n", avex, avey, avez);
	printf("\n");
	printf("An example of configuration file\n");
	printf("ndmap\n");
	printf("diff_x/nd_cellsize diff_y/nd_cellsize diff_z/nd_cellsize nd_cellsize\n");
	printf("%lf %lf %lf 0.0\n", avex, avey, avez);
	printf("1 (use gps), 0 (do not use gps)\n");
	printf("ini_x ini_y ini_z ini_roll ini_pitch ini_yaw\n");
	printf("\n");

	return 0;
}

//load ndt setting file
int load_ndt_ini(char* name){
  FILE *ifp;
  char ndmap_name[200];
  char ndmap_dir[200];
  int i;
  ifp=fopen(name,"r");
  if(!ifp)return 0;

  //map path
  if(fscanf(ifp,"%s",ndmap_name)==EOF)return 0;
  
  for(i=strlen(name)-1;i>0;i--){
    if(name[i]=='/'){
      ndmap_dir[i+1]=0;
      break;
    }
  }
  for(;i>=0;i--)ndmap_dir[i]=name[i];
  sprintf(g_ndmap_name,"%s%s",ndmap_dir,ndmap_name);
  printf("%s\n",g_ndmap_name);
  //map size
  if(fscanf(ifp,"%d %d %d %lf",
	    &g_map_x,&g_map_y,&g_map_z,&g_map_cellsize)==EOF)return 0;
  //map center
  if(fscanf(ifp,"%lf %lf %lf %lf",
	    &g_map_center_x,&g_map_center_y,&g_map_center_z,
	    &g_map_rotation)==EOF)return 0;
  //use gnss
  if(fscanf(ifp,"%d", &g_use_gnss)==EOF)return 0;
  if(!g_use_gnss){
    if(fscanf(ifp,"%lf %lf %lf %lf %lf %lf",
	      &g_ini_x,&g_ini_y,&g_ini_z,
	      &g_ini_roll,&g_ini_pitch,&g_ini_yaw)==EOF)return 0;
  }

  //
  return 1;
}

NDPtr add_ND(void){
  NDPtr ndp;
  //int m;

  if(NDs_num>=MAX_ND_NUM){
    printf("over flow\n");
    return 0;
  }
  
  ndp= NDs+NDs_num;
  NDs_num++;
  
  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num  = 0;
  ndp->m_x  = 0;
  ndp->m_y  = 0;
  ndp->m_z  = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;
  
  return ndp;
}

NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr child){
  int i,j,k,i2,i3,m;
  int x,y,z;
  NDPtr    *nd,*ndp;
  NDMapPtr ndmap;
  
  i2 = i3 = 0;
  printf("Initializing...layer %d\n",layer);
  
  
  x = (g_map_x >> layer)+1;
  y = (g_map_y >> layer)+1; 
  z = (g_map_z >> layer)+1;
  
  /*鐃緒申鐃緒申鐃緒申鐃塾鰹申鐃楯￥申*/
  nd = (NDPtr*)malloc(x*y*z*sizeof(NDPtr));
  ndmap= (NDMapPtr)malloc(sizeof(NDMap));
  
  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y*z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1<<layer);
  printf("size %f\n",ndmap->size);
  
  ndp = nd;
  /*鐃曙イ鐃巡ー鐃塾緒申鐃緒申鐃緒申*/
  for(i = 0; i < x; i++){
    for(j = 0; j < y; j++){
      for(k = 0; k < z; k++){
	*ndp=0;
	ndp++;
      }
    }
  }
  /*鐃曙イ鐃巡ー鐃瞬わ申連鐃暑？*/
  return ndmap;
}

NDMapPtr initialize_NDmap(void){
  int i;
  NDMapPtr ndmap;
  NDPtr null_nd;

  printf("Initializing");
  ndmap = 0;

  //init NDs
  NDs=(NDPtr)malloc(sizeof(NormalDistribution)*MAX_ND_NUM);
  NDs_num=0;
  
  null_nd= add_ND();

  for(i = LAYER_NUM-1;i >= 0;i--){
    ndmap = initialize_NDmap_layer(i,ndmap);

    /*progress dots*/
    printf("layer %d\n",i);
  }


  printf("done\n");

  return ndmap;/*鐃緒申鐃瞬駕申鐃緒申鐃舜のポワ申鐃藷タわ申鐃瞬わ申*/
}

int add_point_covariance(NDPtr nd,PointPtr p){

  /*add data num*/
  nd->num ++;
  nd->flag = 0; /*need to update*/
  //printf("%d \n",nd->num);
  
  /*calcurate means*/
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x*p->x;
  nd->c_yy += p->y*p->y;
  nd->c_zz += p->z*p->z;

  nd->c_xy += p->x*p->y;
  nd->c_yz += p->y*p->z;
  nd->c_zx += p->z*p->x;	

  return 1;
}

int add_point_map(NDMapPtr ndmap, PointPtr point){
  int x,y,z,i;
  NDPtr  *ndp[8];  

  /*
    
  +---+---+
  |   |   |
  +---+---+
  |   |###|
  +---+---+
  
  */
  
  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x/2;
  y = (point->y / ndmap->size) + ndmap->y/2;
  z = (point->z / ndmap->size) + ndmap->z/2;
    
  /*clipping*/
  if(x < 1 || x >=  ndmap->x )return 0;
  if(y < 1 || y >=  ndmap->y )return 0;
  if(z < 1 || z >=  ndmap->z )return 0;
  
  /*select root ND*/
  ndp[0] = ndmap->nd + x*ndmap->to_x + y*ndmap->to_y +z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
 
  /*add  point to map */
  for(i = 0;i < 8;i++){
    if((*ndp[i])==0)*ndp[i]=add_ND();
    if((*ndp[i])!=0)add_point_covariance(*ndp[i], point);
  }
  
  if(ndmap->next){
    add_point_map(ndmap->next , point);  
  }
    
  return 0;		
}

int round_covariance(NDPtr nd){
  double v[3][3],a;

  eigenvecter_matrix3d(nd->covariance,v,nd->l);
  //  print_matrix3d(v);
  if(fabs(v[0][0]*v[0][0]+v[1][0]*v[1][0]+v[2][0]*v[2][0]-1) >0.1)printf("!1");
  if(fabs(v[0][0]*v[0][1]+v[1][0]*v[1][1]+v[2][0]*v[2][1]) >0.01)printf("!01");
  if(fabs(v[0][1]*v[0][2]+v[1][1]*v[1][2]+v[2][1]*v[2][2]) >0.01)printf("!02");
  if(fabs(v[0][2]*v[0][0]+v[1][2]*v[1][0]+v[2][2]*v[2][0]) >0.01)printf("!03");

  a = fabs(nd->l[1]/nd->l[0]);
  if(a <0.001){
    return 0;
    if(nd->l[1] > 0)
      nd->l[1] = fabs(nd->l[0])/10.0;
    else 
      nd->l[1] = -fabs(nd->l[0])/10.0;

  
    a = fabs(nd->l[2]/nd->l[0]);
    if(a <0.01){
      if(nd->l[2] > 0)
	nd->l[2] = fabs(nd->l[0])/10.0;
      else 
	nd->l[2] =-fabs(nd->l[0])/10.0;
     
     
    }
    //    printf("r");
    matrix3d_eigen(v,nd->l[0],nd->l[1],nd->l[2],nd->covariance);
  }
  return 1;
}

int update_covariance(NDPtr nd){
  double a,b,c;/*for calcurate*/
  if(!nd->flag){/*need calcurate?*/    
		/*means*/
    nd->mean.x =a= nd->m_x / nd->num; 
    nd->mean.y =b= nd->m_y / nd->num; 
    nd->mean.z =c= nd->m_z / nd->num; 
    
    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx -2*a*nd->m_x )/ nd->num + a*a;
    nd->covariance[1][1] = (nd->c_yy -2*b*nd->m_y )/ nd->num + b*b;
    nd->covariance[2][2] = (nd->c_zz -2*c*nd->m_z )/ nd->num + c*c;
    nd->covariance[0][1]=nd->covariance[1][0] = (nd->c_xy - nd->m_x*b - nd->m_y*a)/ nd->num+ a*b;
    nd->covariance[1][2]=nd->covariance[2][1] = (nd->c_yz - nd->m_y*c - nd->m_z*b)/ nd->num+ b*c;
    nd->covariance[2][0]=nd->covariance[0][2] = (nd->c_zx - nd->m_z*a - nd->m_x*c)/ nd->num+ c*a;
    nd->sign=0;	
    nd->flag = 1;  /*this ND updated*/
    if(nd->num >= 5){    
      if(round_covariance(nd)==1){
	if(ginverse_matrix3d(nd->covariance, nd->inv_covariance))
	  nd->sign=1;
      }
    }
  }

  return 1;
}

void load (char *name){
  FILE *fp;
  double x,y,z,q;
  Point p;

  fp=fopen(name,"r");

  while(fscanf(fp,"%lf %lf %lf %lf",&y,&x,&z,&q)!=EOF){//x,y swaped
    p.x=(x-g_map_center_x)*cos(-g_map_rotation)-(y-g_map_center_y)*sin(-g_map_rotation);
    p.y=(x-g_map_center_x)*sin(-g_map_rotation)+(y-g_map_center_y)*cos(-g_map_rotation);
    p.z=z-g_map_center_z;
    
    add_point_map(NDmap, &p);
  }
  fclose(fp);
}

void save_nd_map(char* name){
  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;
  FILE *ofp;
  
  //for pcd 
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;
  
  //cloud.is_dense = false;
  //cloud.points.resize (cloud.width * cloud.height);
  //ros::Time stamp;
  //stamp = msg->header.stamp;
  //double now = ros::Time::now().toSec();
  
  ndmap=NDmap;
  ofp=fopen(name,"w");
  
  for(layer=0;layer<2;layer++){
    ndp = ndmap->nd;
    /*鐃曙イ鐃巡ー鐃塾緒申鐃緒申鐃緒申*/
    for(i = 0; i < ndmap->x; i++){
      for(j = 0; j < ndmap->y; j++){
	for(k = 0; k < ndmap->z; k++){
	  if(*ndp){
	    update_covariance(*ndp); 
	    nddat.nd = **ndp;
	    nddat.x =i;
	    nddat.y =j;
	    nddat.z =k;
	    nddat.layer= layer;
	    
	    fwrite(&nddat,sizeof(NDData),1,ofp);

	    //regist the point to pcd data;
	    p.x = (*ndp)->mean.x;
	    p.y = (*ndp)->mean.y;
	    p.z = (*ndp)->mean.z;
	    cloud.points.push_back(p);
    
	  }
	  ndp++;
	}
      }
//      printf("a\n");
    }
    ndmap=ndmap->next;
    
  }
  printf("done\n");
  fclose(ofp);


  //save pcd

  cloud.width=cloud.points.size();
  cloud.height=1;
  pcl::io::savePCDFileASCII ("/tmp/means.pcd", cloud);
  printf("%d\n",(int)cloud.points.size());  
}

