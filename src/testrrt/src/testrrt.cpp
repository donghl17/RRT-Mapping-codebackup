#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <thread>
#include <map_merge/map_merge.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#define foreach BOOST_FOREACH
	using namespace std;

	nav_msgs::OccupancyGrid msg1;
	nav_msgs::OccupancyGrid msg2;
  nav_msgs::OccupancyGrid msg3;
	nav_msgs::OccupancyGrid msg_merge;
// #include <thread>

// #include <map_merge/map_merge.h>
// #include <ros/assert.h>
// #include <ros/console.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace map_merge
{
MapMerge::MapMerge() : subscriptions_size_(0)//构造函数主要创建节点、获取参数、发布topic
{
  ros::NodeHandle private_nh;
  std::string frame_id;
  std::string merged_map_topic;

  private_nh.param("merging_rate", merging_rate_, 4.0);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("estimation_rate", estimation_rate_, 0.5);
  private_nh.param("known_init_poses", have_initial_poses_, true);
  private_nh.param("estimation_confidence", confidence_threshold_, 1.0);
  private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
  private_nh.param<std::string>("robot_map_updates_topic",
                                robot_map_updates_topic_, "map_updates");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
  private_nh.param<std::string>("world_frame", world_frame_, "world");


  /* publishing */
  merged_map_publisher_ =
      node_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
}

/*
 * Subcribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  ROS_DEBUG("Robot discovery started.");

  ros::master::V_TopicInfo topic_infos;
  geometry_msgs::Transform init_pose;
  std::string robot_name;
  std::string map_topic;
  std::string map_updates_topic;

  ros::master::getTopics(topic_infos);
  // default msg constructor does no properly initialize quaternion
  init_pose.rotation.w = 1;  // create identity quaternion

  for (const auto& topic : topic_infos) {
    // we check only map topic
    if (!isRobotMapTopic(topic)) {
      continue;
    }

    robot_name = robotNameFromTopic(topic.name);
    if (robots_.count(robot_name)) {
      // we already know this robot
      continue;
    }

    if (have_initial_poses_ && !getInitPose(robot_name, init_pose)) {
      ROS_WARN("Couldn't get initial position for robot [%s]\n"
               "did you defined parameters map_merge/init_pose_[xyz]? in robot "
               "namespace? If you want to run merging without known initial "
               "positions of robots please set `known_init_poses` parameter "
               "to false. See relavant documentation for details.",
               robot_name.c_str());
      continue;
    }

    ROS_INFO("adding robot [%s] to system", robot_name.c_str());
    {
      std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
      subscriptions_.emplace_front();
      ++subscriptions_size_;
    }

    // no locking here. robots_ are used only in this procedure
    MapSubscription& subscription = subscriptions_.front();
    robots_.insert({robot_name, &subscription});
    subscription.initial_pose = init_pose;

    /* subscribe callbacks */
    map_topic = ros::names::append(robot_name, robot_map_topic_);
    map_updates_topic =
        ros::names::append(robot_name, robot_map_updates_topic_);
    ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
    subscription.map_sub = node_.subscribe<nav_msgs::OccupancyGrid>(
        map_topic, 50,
        [this, &subscription](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
          fullMapUpdate(msg, subscription);
        });
    ROS_INFO("Subscribing to MAP updates topic: %s.",
             map_updates_topic.c_str());
    subscription.map_updates_sub =
        node_.subscribe<map_msgs::OccupancyGridUpdate>(
            map_updates_topic, 50,
            [this, &subscription](
                const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
              partialMapUpdate(msg, subscription);
            });
  }
}

/*
 * mapMerging()
 */
void MapMerge::mapMerging()
{
  ROS_DEBUG("Map merging started.");

  if (have_initial_poses_) {
    std::vector<nav_msgs::OccupancyGridConstPtr> grids;
    std::vector<geometry_msgs::Transform> transforms;
    grids.reserve(subscriptions_size_);
    {
      //boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);//!!!!!!!!!这里改成两张图!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
      //int count=1;
      geometry_msgs::Transform init_pose1;
      init_pose1.rotation.w = 1;
      init_pose1.translation.x = 0.0;
      init_pose1.translation.y = 0.4;
      init_pose1.translation.z = 0.0;
      double yaw = 0.0;
      tf2::Quaternion q;
      q.setEuler(0., 0., yaw);
      init_pose1.rotation = toMsg(q);

      geometry_msgs::Transform init_pose2;
      init_pose2.rotation.w = 1;
      init_pose2.translation.x = 0.0;
      init_pose2.translation.y = -0.4;
      init_pose2.translation.z = 0.0;
      //double yaw = 0.0;
      //tf2::Quaternion q;
      //q.setEuler(0., 0., yaw);
      init_pose2.rotation = toMsg(q);
      subscriptions_.emplace_front();
      MapSubscription& subscription = subscriptions_.front();
      subscription.readonly_map = nav_msgs::OccupancyGrid::ConstPtr(&msg1);
      subscription.initial_pose=init_pose1;
      grids.push_back(subscription.readonly_map);
      transforms.push_back(subscription.initial_pose);
      subscriptions_.emplace_front();
      MapSubscription& subscription1= subscriptions_.front();
      subscription1.readonly_map = nav_msgs::OccupancyGrid::ConstPtr(&msg2);
      //subscription1.readonly_map=(nav_msgs::OccupancyGrid::ConstPtr)&msg2;
      subscription1.initial_pose=init_pose2;
      grids.push_back(subscription1.readonly_map);
      transforms.push_back(subscription1.initial_pose);

      //for (auto& subscription : subscriptions_) {
      //   //std::lock_guard<std::mutex> s_lock(subscription.mutex);
      //   if(count==1)
      //   {
       //   subscription.readonly_map=(nav_msgs::OccupancyGrid::ConstPtr)&msg1;
      //     subscription.initial_pose=init_pose1;
      //   }
      //   if(count==2)
      //   {
      //     subscription.readonly_map=(nav_msgs::OccupancyGrid::ConstPtr)&msg2;
      //     subscription.initial_pose=init_pose2;
      //   }
      //   grids.push_back(subscription.readonly_map);
      //   transforms.push_back(subscription.initial_pose);
      //   count++;
      //}
    }
    // we don't need to lock here, because when have_initial_poses_ is true we
    // will not run concurrently on the pipeline
    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(transforms.begin(), transforms.end());//把地图和坐标变换全部变成openvcv形式了
  }

  nav_msgs::OccupancyGridPtr merged_map;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    merged_map = pipeline_.composeGrids();//！！！！！！！！！！！！！！！！！！合并操作在这里！，函数声明在merging——pipline。cpp----->gridcompositor
  }
  if (!merged_map) {
    return;
  }

  ROS_DEBUG("all maps merged, publishing");
  ros::Time now = ros::Time::now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = world_frame_;

  ROS_ASSERT(merged_map->info.resolution > 0.f);
  merged_map_publisher_.publish(merged_map);////！！！！！！！！！！！发布合并后的地图在这里
}

// void MapMerge::poseEstimation()
// {
//   ROS_DEBUG("Grid pose estimation started.");
//   std::vector<nav_msgs::OccupancyGridConstPtr> grids;
//   grids.reserve(subscriptions_size_);
//   {
//     boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
//     for (auto& subscription : subscriptions_) {
//       std::lock_guard<std::mutex> s_lock(subscription.mutex);
//       grids.push_back(subscription.readonly_map);
//     }
//   }

//   std::lock_guard<std::mutex> lock(pipeline_mutex_);
//   pipeline_.feed(grids.begin(), grids.end());
//   // TODO allow user to change feature type
//   pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
//                                confidence_threshold_);
// }

void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                             MapSubscription& subscription)
{
  ROS_DEBUG("received full map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);
  if (subscription.readonly_map &&
      subscription.readonly_map->header.stamp > msg->header.stamp) {
    // we have been overrunned by faster update. our work was useless.
    return;
  }

  subscription.readonly_map = msg;
  subscription.writable_map = nullptr;
}

void MapMerge::partialMapUpdate(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
    MapSubscription& subscription)
{
  ROS_DEBUG("received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  nav_msgs::OccupancyGridPtr map;
  nav_msgs::OccupancyGridConstPtr readonly_map;  // local copy
  {
    // load maps
    std::lock_guard<std::mutex> lock(subscription.mutex);
    map = subscription.writable_map;
    readonly_map = subscription.readonly_map;
  }

  if (!readonly_map) {
    ROS_WARN("received partial map update, but don't have any full map to "
             "update. skipping.");
    return;
  }

  // we don't have partial map to take update, we must copy readonly map and
  // update new writable map
  if (!map) {
    map.reset(new nav_msgs::OccupancyGrid(*readonly_map));
  }

  size_t grid_xn = map->info.width;
  size_t grid_yn = map->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map &&
        subscription.readonly_map->header.stamp > map->header.stamp) {
      // we have been overrunned by faster update. our work was useless.
      return;
    }
    subscription.writable_map = map;
    subscription.readonly_map = map;
  }
}

std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only occupancy grids as maps */
  bool is_occupancy_grid = topic.datatype == "nav_msgs/OccupancyGrid";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!获得初始位置参数！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
 */
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::Transform& pose)
{
  std::string merging_namespace = ros::names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      ros::param::get(ros::names::append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}

/*
 * execute()
 */
void MapMerge::executemapMerging()
{
  ros::Rate r(merging_rate_);
  while (node_.ok()) {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mapMerging();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << " time cost= " << (time_used.count() * 1000) << " ms." << std::endl;
    r.sleep();
  }
}

// void MapMerge::executetopicSubscribing()
// {
//   ros::Rate r(discovery_rate_);
//   while (node_.ok()) {
//     topicSubscribing();
//     r.sleep();
//   }
// }

// void MapMerge::executeposeEstimation()
// {
//   if (have_initial_poses_)
//     return;

//   ros::Rate r(estimation_rate_);
//   while (node_.ok()) {
//     poseEstimation();
//     r.sleep();
//   }
// }

/*
 * spin()
 */
void MapMerge::spin()
{
  ros::spinOnce();
  std::thread merging_thr([this]() { executemapMerging(); });
  //std::thread subscribing_thr([this]() { executetopicSubscribing(); });
  //std::thread estimation_thr([this]() { executeposeEstimation(); });
  ros::spin();
  //estimation_thr.join();
  merging_thr.join();
  //subscribing_thr.join();
}

}  // namespace map_merge

void show_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	std::cout << "get!!!\n";
}


































nav_msgs::OccupancyGrid mapData1,mapData2,mapData3;

void mapCallBack1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData1=*msg;
}
void mapCallBack2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData2=*msg;
}
void mapCallBack3(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData3=*msg;
}



signed char pixel_merge(signed char merged_pixel,signed char local_pixel)
{
  if(merged_pixel==-1)
  {merged_pixel=local_pixel;
  }
  else if (merged_pixel==0)
  {
    if(local_pixel==100){merged_pixel=local_pixel;}
  }
  else if(merged_pixel==100){}
  return merged_pixel;
}

//nav_msgs::OccupancyGrid
void merge(nav_msgs::OccupancyGrid &result,std::vector<nav_msgs::OccupancyGrid> temp1,std::vector<float> transform)
{
 int size=temp1.size();
 float resolution=temp1.front().info.resolution;
 std::vector<nav_msgs::OccupancyGrid>::iterator it;
 int count=0;
 for(it=temp1.begin();it!=temp1.end();it++)
 {
  //int x=transform[2*count]/resolution;
  //int y=transform[2*count+1]/resolution;
  nav_msgs::OccupancyGrid it_map=*it;
  int x_leftdown=(transform[2*count]+it_map.info.origin.position.x)/resolution;//像素坐标
  int y_leftdown=(transform[2*count+1]+it_map.info.origin.position.y)/resolution;//像素坐标
  int row_start=(result.info.height/2+y_leftdown)*result.info.width+(result.info.width/2+x_leftdown);//每行起始坐标
  int row_end=row_start+it_map.info.width-1;//每行停止坐标
  for(int j=0;j<it_map.info.height;j++)//对小地图的第j列（0～height-1）
  {
   for (int i=row_start;i<=row_end;i++)//对小地图的每一行对应的大地图中的位置
   {
    int k=j*it_map.info.width+i-row_start;
    
    result.data[i]=pixel_merge(result.data[i],it_map.data[k]);
    
   }
   row_start=row_start+result.info.width;
   row_end=row_start+it_map.info.width-1;
  }
  count++;
 }
 result.header.stamp = ros::Time::now(); 
 //return result;
}



int main(int argc, char **argv){
	// std::cout << "hello world" << std::endl;
	rosbag::Bag bag;
  bag.open("/home/donghl/robot1_largemap_test.bag",rosbag::bagmode::Read);
	//bag.open("/home/donghl/robot1_map_test.bag",rosbag::bagmode::Read);
	rosbag::View view(bag);
	nav_msgs::OccupancyGrid s1;
  //nav_msgs::OccupancyGrid::ConstPtr s1_ptr;

	foreach(rosbag::MessageInstance const m1, view)
	{	
		s1=*m1.instantiate<nav_msgs::OccupancyGrid>();
	}
	bag.close();
  //s1_ptr = nav_msgs::OccupancyGrid::ConstPtr(&s1);
  rosbag::Bag bag2;
	bag2.open("/home/donghl/robot2_largemap_test.bag",rosbag::bagmode::Read);
	//bag2.open("/home/donghl/robot2_map_test.bag",rosbag::bagmode::Read);
  rosbag::View view2(bag2);
	nav_msgs::OccupancyGrid s2;
	foreach(rosbag::MessageInstance const m2, view2)
	{		
		s2=*m2.instantiate<nav_msgs::OccupancyGrid>();
	}
	bag2.close();
  rosbag::Bag bag3;
  bag3.open("/home/donghl/robot3_largemap_test.bag",rosbag::bagmode::Read);
  //bag3.open("/home/donghl/map_merge_test.bag",rosbag::bagmode::Read);
	rosbag::View view_m(bag3);
	
	nav_msgs::OccupancyGrid s3;
	foreach(rosbag::MessageInstance const m3, view_m)
	{	
		s3=*m3.instantiate<nav_msgs::OccupancyGrid>();
	}
	bag3.close();

	//rviz 显示map1,map2,map_merge
	ros::init(argc,argv,"show_node");
	ros::NodeHandle n1;
	ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::NodeHandle n4;
  ros::Publisher pub2=n2.advertise<nav_msgs::OccupancyGrid>("show_topic_robot2",1000);
	ros::Publisher pub=n1.advertise<nav_msgs::OccupancyGrid>("show_topic_robot1",1000);
  ros::Publisher pub3=n3.advertise<nav_msgs::OccupancyGrid>("show_topic_robot3",1000);
  ros::Publisher pub4=n4.advertise<nav_msgs::OccupancyGrid>("merged_map",1000);
  ros::Subscriber sub1= n1.subscribe("show_topic_robot1", 100 ,mapCallBack1);	
  ros::Subscriber sub2= n2.subscribe("show_topic_robot2", 100 ,mapCallBack2);
  ros::Subscriber sub3= n3.subscribe("show_topic_robot3", 100 ,mapCallBack3);
	ros::Rate loop_rate(1);

	// ros::Subscriber sub=n.subscribe("show_topic",1000,show_callback);

  //我把消息全部变成全局变量了，方便在函数声明中调用
	//int count=0;
  nav_msgs::OccupancyGrid merged_map;
  nav_msgs::OccupancyGrid result;
  result.header.frame_id="merge";
  result.header.stamp = ros::Time::now(); 
  result.info.resolution = 0.1;         // float32
  result.info.width      = 1000;           // uint32
  result.info.height     = 1000; 
  int p[result.info.width*result.info.height];
  for(int i=0;i<result.info.width*result.info.height;i++)
  {p[i]=-1;}
  std::vector<signed char> a(p, p+result.info.width*result.info.height);
  result.data = a;
	while(ros::ok())
	{

		msg1=s1;
		msg1.header.frame_id = "show_topic_robot1";
		msg2=s2;
    msg2.header.frame_id = "show_topic_robot2";
		std::cout << "test1\n";
		msg3 = s3;
    msg3.header.frame_id = "show_topic_robot3";
		std::cout << "test2\n";
    pub2.publish(msg2);
		ros::spinOnce();
		std::cout << "test3\n";
    //loop_rate.sleep();
		pub.publish(msg1);
    ros::spinOnce();
		std::cout << "test4\n";
    //loop_rate.sleep();
		pub3.publish(msg3);
		std::cout << "test5\n";
		//std::cout << count << " t\n";
		ros::spinOnce();
		//loop_rate.sleep();
		//count++;
		//if(count>=10) break;
    std::vector<float> transform;
    transform.push_back(0.0);
    transform.push_back(6.0);
    transform.push_back(-9.0);
    transform.push_back(-0.5);
    transform.push_back(6.0);
    transform.push_back(0.0);
    //result.data = a;
    std::vector<nav_msgs::OccupancyGrid> temp1;
    temp1.push_back(mapData1);
    temp1.push_back(mapData2); 
    temp1.push_back(mapData3);     
      //merged_map=
    merge(result,temp1,transform);
    pub4.publish(result);
    
  


	}


	
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // ros::init(argc, argv, "map_merge");
  // // this package is still in development -- start wil debugging enabled
  // 	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // 	}
  // 	map_merge::MapMerge map_merging;
	//设置参数
	//map_merging.subscriptions_size_=2;

  
	  
	//map_merging.spin();
	return 0;
}
