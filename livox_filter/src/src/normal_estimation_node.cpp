#include"normal_estimation.h"



int main(int argc,char **argv){
    ros::init(argc,argv,"normal_estimation");
    Normal_estimation livox_filter_node;
    ros::spin();
    
}