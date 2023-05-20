#ifndef _LIDAR_H
#define _LIDAR_H


struct Params
{
	/* Pointer to shared struct */
	struct Lidar_data* shared;
	/* Params to set the viewing distance */
	float max_distance;
	/* Values of the most recent valid sensor reading */
	float theta;
	float distance;
	int age;
	bool* p_terminate;
};

struct Lidar_data {
	float theta;
	float distance;
	int quality;
};

#endif  /* _LIDAR_H */
