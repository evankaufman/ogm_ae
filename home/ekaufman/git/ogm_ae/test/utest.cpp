#include <chrono>
#include <typeinfo>
#include <gtest/gtest.h>
#include "ogm_ae/voxel_map.hpp"
#include "ogm_ae/ProbabilisticOccupancyGridMapping.hpp"

using namespace std;
using namespace std::chrono;
// using namespace probabilistic_occupancy_grid_mapping;
auto map3D = new VoxelMap(1,1,2,Eigen::Vector3d(10,10,10),0.1);

TEST(Mapping, voxelCelltest)
{
    //auto map3D = new GridMap(1,1,1,0.1);
    auto dim_vector = map3D->getCellDim();
    auto map  = map3D->getMap();
    ASSERT_EQ(11, dim_vector[0]);
    ASSERT_EQ(11, dim_vector[1]);
    ASSERT_EQ(21, dim_vector[2]);
}
TEST(Mapping, mapTotalVoxel)
{
    ASSERT_EQ(11*11*21, map3D->getN_grid());
}
TEST(Mapping, mapInRange)
{
    map3D->setMinMaxDim(-2,1,-1,2,3,1);
    vector<double> point{0.5,1.5,0.5};
    ASSERT_TRUE(map3D->inMapRange(point));
    point[0] = 2.1;
    ASSERT_FALSE(map3D->inMapRange(point));
    point[0] = -4;
    ASSERT_FALSE(map3D->inMapRange(point));
    point[0] = 0.5;
    ASSERT_TRUE(map3D->inMapRange(point));
    point[1] = -12;
    ASSERT_FALSE(map3D->inMapRange(point));
    point[1] = 3.1;
    ASSERT_FALSE(map3D->inMapRange(point));
}
// TEST(Mapping, point2sub)
// {
//     vector<double> point{0.5,1.5,0.5};
//     auto index =  map3D->point2sub(point);
// }
TEST(Mapping, point2index)
{
    vector<double> point{0.5,1.5,0.5};
    ASSERT_TRUE(map3D->point2ind(point)> -1);
    point[0] = 2;
    ASSERT_TRUE(map3D->point2ind(point) > -1);
}
TEST(Mapping, index2point)
{
    //vector<double> point{0.5,1.5,0.5};
    //ASSERT_EQ(point,map3D->index2point(map3D->point2index(point)));
    vector<double> point2{-2,2,-1};
    ASSERT_EQ(point2,map3D->ind2point(map3D->point2ind(point2)));
    vector<double> point3{0.5,1.5,0.5};
    ASSERT_EQ(point3,map3D->ind2point(map3D->point2ind(point3)));
    //ASSERT_EQ( MapLocationFromInd(20)
}
TEST(Mapping, ind2sub)
{
    // vector<double> point{0.5,1.5,0.5};
    int test = 2;
    auto index =  map3D->ind2sub(test);
}
TEST(Mapping, rayTracing)
{
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();
    auto diff = end - start;
    cout << chrono::duration <double, nano> (diff).count() << " ns" << endl;
}
TEST(Mapping, forwardSensorModel)
{

}
TEST(Mapping, inverseSensorModel)
{

}
TEST(Mapping, msgCallback)
{

}
TEST(Mapping, functionProfile)
{

}
TEST(Mapping, multiThreding)
{

}
TEST(Mapping, visualization)
{
     // initiate window
     // show map
     // check results

}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  //ros::init(argc, argv, "utester");
  return RUN_ALL_TESTS();
}
