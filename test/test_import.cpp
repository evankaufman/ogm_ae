#include "ros/ros.h"
#include "gtest/gtest.h"
// #include "gmock/gmock.h"
#include <vector>
#include "voxel/voxel.h"

// use the testing for importing all the voxel functionalities
// all the package functions can be shared between exploration and mapping
using namespace std;

// Voxel voxel_map;
// voxel_map.init();

TEST(Voxel, package_import)
{
    ASSERT_TRUE(import_test());
}
TEST(Voxel, constructor)
{
    Voxel test;
    test.init();
    double z = 5;
    double z_exp = 4;
    uint sensor = 1;
    // auto temp = test.ForwardSensorModel(z,z_exp, sensor);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "controller_tester");
    return RUN_ALL_TESTS();
}
