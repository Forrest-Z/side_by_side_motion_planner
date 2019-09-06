#include <stdio.h>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>

char *progName = NULL;

void
usage(void)
{
	printf("\n");
	printf("\tThis program was written by Naoki AKAI.\n");
	printf("\n");
	printf("\tdefault ---> %s points.pts points.pcd\n", progName);
	printf("\n");
	exit(0);
}

int main(int argc, char *argv[])
{
	progName = argv[0];
	if (argv[1] == NULL)	usage();
	if (argv[2] == NULL)	usage();

	FILE *fp;
	fp = fopen(argv[1], "r");
	if (fp == NULL)
	{
		printf("cannot open pts file (%s)", argv[1]);
		exit(1);
	}

	double ny, z, x;
	pcl::PointXYZ p;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	printf("start conversion\n");
	while ((fscanf(fp, "%lf %lf %lf", &ny, &z, &x)) != EOF)
	{
		// pts [cm], pcd [m]
		p.x = x / 100.0;
		p.y = -1.0 * ny / 100.0;
		p.z = z / 100.0;
		cloud.points.push_back(p);
	}
	printf("\nconversion is done\n");
	fclose(fp);

	if (cloud.points.size() != 0)
	{
		printf("save point data in pcd file\n");
		cloud.width = cloud.points.size();
		cloud.height = 1;
		pcl::io::savePCDFileASCII(argv[2], cloud);
	}
	else
	{
		printf("no point data\n");
	}

	return 0;
}
