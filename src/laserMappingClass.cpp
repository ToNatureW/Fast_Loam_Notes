// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingClass.h"

/**
 * 初始化局部子图5*5*5的三维vector,对应为map，每个格子对应50米
 * @param map_resolution
 */
void LaserMappingClass::init(double map_resolution){
	//init map
	//init can have real object, but future added block does not need
	for(int i=0;i<LASER_CELL_RANGE_HORIZONTAL*2+1;i++){
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
		for(int j=0;j<LASER_CELL_RANGE_HORIZONTAL*2+1;j++){
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
			for(int k=0;k<LASER_CELL_RANGE_VERTICAL*2+1;k++){
				pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
				map_depth_temp.push_back(point_cloud_temp);	
			}
			map_height_temp.push_back(map_depth_temp);
		}
		map.push_back(map_height_temp);
	}

	origin_in_map_x = LASER_CELL_RANGE_HORIZONTAL;  // 2
	origin_in_map_y = LASER_CELL_RANGE_HORIZONTAL;  // 2
	origin_in_map_z = LASER_CELL_RANGE_VERTICAL;    // 2
	map_width       = LASER_CELL_RANGE_HORIZONTAL*2+1; // 5
	map_height      = LASER_CELL_RANGE_HORIZONTAL*2+1; // 5
	map_depth       = LASER_CELL_RANGE_HORIZONTAL*2+1; // 5

	//downsampling size
	downSizeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);  // 设置降采样大小
}
/**
 * x轴负方向上多一个yoz面
 */
void LaserMappingClass::addWidthCellNegative(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.insert(map.begin(), map_height_temp);

	origin_in_map_x++;
	map_width++;
}
/**
 * x轴正方向多一个yoz面
 */
void LaserMappingClass::addWidthCellPositive(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height; j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0; k< map_depth; k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.push_back(map_height_temp);
	map_width++;
}

/**
 * y轴负方向多一个xoz面
 */
void LaserMappingClass::addHeightCellNegative(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].insert(map[i].begin(), map_depth_temp);
	}
	origin_in_map_y++;
	map_height++;
}

/**
 * y轴正方向多一个xoz面
 */
void LaserMappingClass::addHeightCellPositive(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].push_back(map_depth_temp);
	}
	map_height++;
}

/**
 * z轴负方向上多一个xoy平面
 */
void LaserMappingClass::addDepthCellNegative(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].insert(map[i][j].begin(), point_cloud_temp);
		}
	}
	origin_in_map_z++;
	map_depth++;
}

/**
 * z轴正方向上多一个xoy平面
 */
void LaserMappingClass::addDepthCellPositive(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].push_back(point_cloud_temp);
		}
	}
	map_depth++;
}

//extend map is points exceed size
/**
 * 距离原点超过25米时，扩展map
 * @param x
 * @param y
 * @param z
 */
void LaserMappingClass::checkPoints(int& x, int& y, int& z){

	while(x + LASER_CELL_RANGE_HORIZONTAL> map_width-1){
		addWidthCellPositive();
	}
	while(x-LASER_CELL_RANGE_HORIZONTAL<0){
		addWidthCellNegative();
		x++;
	}
	while(y + LASER_CELL_RANGE_HORIZONTAL> map_height-1){
		addHeightCellPositive();
	}
	while(y-LASER_CELL_RANGE_HORIZONTAL<0){
		addHeightCellNegative();
		y++;
	}
	while(z + LASER_CELL_RANGE_VERTICAL> map_depth-1){
		addDepthCellPositive();
	}
	while(z-LASER_CELL_RANGE_VERTICAL<0){
		addDepthCellNegative();
		z++;
	}

	// initialize
	// create object if area is null
    // 初始化5*5*5的map中的Ptr
	for(int i=x-LASER_CELL_RANGE_HORIZONTAL;i<x+LASER_CELL_RANGE_HORIZONTAL+1;i++){
		for(int j=y-LASER_CELL_RANGE_HORIZONTAL;j<y+LASER_CELL_RANGE_HORIZONTAL+1;j++){
			for(int k=z-LASER_CELL_RANGE_VERTICAL;k<z+LASER_CELL_RANGE_VERTICAL+1;k++){
				if(map[i][j][k]==NULL){
					pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
					map[i][j][k] = point_cloud_temp;
				}

			}
				
		}

	}
}

//update points to map 
void LaserMappingClass::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current){
	
	int currentPosIdX = int(std::floor(pose_current.translation().x() / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
	int currentPosIdY = int(std::floor(pose_current.translation().y() / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
	int currentPosIdZ = int(std::floor(pose_current.translation().z() / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;


    // 检查是否在xyz方向上走25米了
	checkPoints(currentPosIdX,currentPosIdY,currentPosIdZ);

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());  // 将当前帧转换到map坐标系下
	

    // 将当前帧的所有特征点加入到map中对应的位置
	for (int i = 0; i < (int)transformed_pc->points.size(); i++)
	{
		pcl::PointXYZI point_temp = transformed_pc->points[i];
		//for visualization only
		point_temp.intensity = std::min(1.0 , std::max(pc_in->points[i].z+2.0, 0.0) / 5);
		int currentPointIdX = int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
		int currentPointIdY = int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
		int currentPointIdZ = int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

		map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
		
	}
	

    // 对map中的每一个格子进行降采样
	for(int i = currentPosIdX-LASER_CELL_RANGE_HORIZONTAL; i < currentPosIdX+LASER_CELL_RANGE_HORIZONTAL+1 ; i++){
		for(int j = currentPosIdY-LASER_CELL_RANGE_HORIZONTAL;j < currentPosIdY+LASER_CELL_RANGE_HORIZONTAL+1 ; j++){
			for(int k = currentPosIdZ-LASER_CELL_RANGE_VERTICAL;k < currentPosIdZ+LASER_CELL_RANGE_VERTICAL+1 ; k++){
				downSizeFilter.setInputCloud(map[i][j][k]);
				downSizeFilter.filter(*(map[i][j][k]));
			}
				
		}

	}

}


pcl::PointCloud<pcl::PointXYZI>::Ptr LaserMappingClass::getMap(void){
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new  pcl::PointCloud<pcl::PointXYZI>());
	for (int i = 0; i < map_width; i++){
		for (int j = 0; j < map_height; j++){
			for (int k = 0; k < map_depth; k++){
				if(map[i][j][k]!=NULL){
					*laserCloudMap += *(map[i][j][k]);
				}
			}
		}
	}
	return laserCloudMap;
}

LaserMappingClass::LaserMappingClass(){

}

