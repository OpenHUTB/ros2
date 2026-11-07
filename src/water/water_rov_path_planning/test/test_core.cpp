#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "water_rov_path_planning/heatmap.hpp"
#include "water_rov_path_planning/ocean_current.hpp"
#include "water_rov_path_planning/path_planner.hpp"

using namespace water_rov_path_planning;

TEST(HeatmapRisk, GaussianDecayAndRadius)
{
  Heatmap::Params hp;
  hp.grid_size = 100;
  hp.resolution = 0.5;
  hp.obstacle_radius = 2.0;
  hp.risk_decay = 0.15;
  Heatmap heatmap(hp);
  heatmap.addObstacle(25.0, 25.0);  // 落在格子 (50, 50)

  EXPECT_NEAR(heatmap.getRisk(50, 50), 1.0, 1e-9);
  // 3 格 = 1.5 m
  EXPECT_NEAR(heatmap.getRisk(53, 50), std::exp(-0.15 * 1.5 * 1.5), 1e-9);
  // 4 格 = 2.0 m，恰好等于影响半径
  EXPECT_NEAR(heatmap.getRisk(54, 50), std::exp(-0.15 * 2.0 * 2.0), 1e-9);
  // 5 格 = 2.5 m，超出影响半径
  EXPECT_DOUBLE_EQ(heatmap.getRisk(55, 50), 0.0);
  // 出界按高风险处理
  EXPECT_DOUBLE_EQ(heatmap.getRisk(-1, 0), 1.0);
}

TEST(HeatmapGridMapping, NegativeAndBoundaryCoordinatesRejected)
{
  Heatmap::Params hp;
  hp.grid_size = 100;
  hp.resolution = 0.5;
  Heatmap heatmap(hp);

  int gx = -1, gy = -1;
  // 负坐标必须判越界，不能被截断成 0 号格
  EXPECT_FALSE(heatmap.worldToGrid(-0.2, 5.0, gx, gy));
  EXPECT_FALSE(heatmap.worldToGrid(5.0, -0.2, gx, gy));
  // 恰好在地图边上的坐标映射到 grid_size 号格，同样越界
  EXPECT_FALSE(heatmap.worldToGrid(50.0, 5.0, gx, gy));
  EXPECT_FALSE(heatmap.worldToGrid(5.0, 50.0, gx, gy));
  // 边界内的点正常映射
  ASSERT_TRUE(heatmap.worldToGrid(49.9, 0.0, gx, gy));
  EXPECT_EQ(gx, 99);
  EXPECT_EQ(gy, 0);
}

TEST(OceanCurrent, TidalPhaseDiffersFromStatic)
{
  OceanCurrent::Params cp;
  cp.pattern = OceanCurrent::Pattern::TIDAL;
  cp.magnitude = 0.4;
  cp.direction = 0.0;
  cp.tidal_period = 60.0;
  OceanCurrent current(cp);

  // t = 0 时潮汐流速为 0
  EXPECT_NEAR(current.getVelocity(5.0, 5.0, 0.0).norm(), 0.0, 1e-12);
  // t = T/4 时达到峰值
  Eigen::Vector2d quarter = current.getVelocity(5.0, 5.0, 15.0);
  EXPECT_NEAR(quarter.x(), 0.4, 1e-9);
  EXPECT_NEAR(quarter.y(), 0.0, 1e-9);
}

TEST(PathPlannerCost, FavourableCrossAdverse)
{
  Heatmap::Params hp;
  hp.grid_size = 20;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);

  OceanCurrent::Params cp;
  cp.pattern = OceanCurrent::Pattern::UNIFORM;
  cp.magnitude = 0.4;
  cp.direction = 0.0;  // 沿 +x
  cp.world_size = 10.0;
  auto current = std::make_shared<OceanCurrent>(cp);

  PathPlanner planner(PathPlanner::Params{});
  planner.setEnvironment(heatmap, current);

  // 单步 1 格 = 0.5 m，长度项 0.5；洋流项按 (û·v)/v_ref 归一化：
  // 顺流 0.5 - 0.5*3.0*0.4*0.5 = 0.20，横向 0.50，逆流 0.5 + 3.0*0.4*0.5 = 1.10
  double favourable = planner.stepCost(10, 10, 11, 10);  // 顺流
  double cross = planner.stepCost(10, 10, 10, 11);       // 横向
  double adverse = planner.stepCost(10, 10, 9, 10);      // 逆流

  EXPECT_NEAR(favourable, 0.20, 1e-9);
  EXPECT_NEAR(cross, 0.50, 1e-9);
  EXPECT_NEAR(adverse, 1.10, 1e-9);
  EXPECT_LT(favourable, cross);
  EXPECT_LT(cross, adverse);
}

TEST(PathPlannerCost, FavourableRewardCappedAtLengthTerm)
{
  Heatmap::Params hp;
  hp.grid_size = 20;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);

  OceanCurrent::Params cp;
  cp.pattern = OceanCurrent::Pattern::UNIFORM;
  cp.magnitude = 1.0;  // 等于参考流速，归一化对齐度饱和在 1
  cp.direction = 0.0;
  cp.world_size = 10.0;
  auto current = std::make_shared<OceanCurrent>(cp);

  PathPlanner planner(PathPlanner::Params{});
  planner.setEnvironment(heatmap, current);

  // 顺流奖励按公式为 0.5*3.0*1.0 = 1.5 倍长度项，被截到长度项本身：
  // 代价最低为 0 而不是负数，A* 不会把强顺流边当免费移动
  EXPECT_DOUBLE_EQ(planner.stepCost(10, 10, 11, 10), 0.0);
  // 逆流不受截断影响：0.5 + 3.0*1.0*0.5 = 2.0
  EXPECT_NEAR(planner.stepCost(10, 10, 9, 10), 2.0, 1e-9);
}

TEST(PathPlannerCost, ReferenceSpeedNormalizesMagnitude)
{
  Heatmap::Params hp;
  hp.grid_size = 20;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);

  OceanCurrent::Params cp;
  cp.pattern = OceanCurrent::Pattern::UNIFORM;
  cp.magnitude = 0.4;
  cp.direction = 0.0;
  cp.world_size = 10.0;
  auto current = std::make_shared<OceanCurrent>(cp);

  // 参考流速翻倍时，洋流项贡献减半，长度项不变
  PathPlanner::Params pp;
  pp.current_ref_speed = 2.0;
  PathPlanner planner(pp);
  planner.setEnvironment(heatmap, current);

  // 参考流速翻倍后对齐度减半：顺流 0.5-0.5*3.0*0.2*0.5 = 0.35，逆流 0.5+3.0*0.2*0.5 = 0.80
  EXPECT_NEAR(planner.stepCost(10, 10, 11, 10), 0.35, 1e-9);  // 顺流
  EXPECT_NEAR(planner.stepCost(10, 10, 10, 11), 0.50, 1e-9);  // 横向
  EXPECT_NEAR(planner.stepCost(10, 10, 9, 10), 0.80, 1e-9);   // 逆流
}

TEST(PathPlannerSmooth, SegmentSamplingCatchesInteriorRisk)
{
  Heatmap::Params hp;
  hp.grid_size = 100;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);
  // 障碍物位于长段内部，离两端点和段中点都超过影响半径 2 m
  heatmap->addObstacle(8.0, 10.0);

  PathPlanner planner(PathPlanner::Params{});
  planner.setEnvironment(heatmap, nullptr);

  std::vector<PathPlanner::Point> path{{0.0, 10.0}, {10.0, 10.0}, {20.0, 10.0}};
  // 端点与段中点均安全，只有按半格采样的段内检查能发现 8.0 m 处的风险
  EXPECT_FALSE(planner.smoothPath(path, 1));
  EXPECT_DOUBLE_EQ(path[1].x, 10.0);
  EXPECT_DOUBLE_EQ(path[1].y, 10.0);
}

TEST(PathPlannerSmooth, JacobiAverageAndRiskFallback)
{
  Heatmap::Params hp;
  hp.grid_size = 100;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);

  PathPlanner planner(PathPlanner::Params{});
  planner.setEnvironment(heatmap, nullptr);

  std::vector<PathPlanner::Point> path{{5.0, 5.0}, {6.0, 6.0}, {7.0, 5.0}};
  ASSERT_TRUE(planner.smoothPath(path, 1));
  // 单个点等于 0.5*自身 + 0.25*前后点
  EXPECT_DOUBLE_EQ(path[1].x, 6.0);
  EXPECT_DOUBLE_EQ(path[1].y, 5.5);

  // 在平滑后的位置放置障碍物，风险超阈值时必须整条回退到原始路径
  heatmap->addObstacle(6.0, 5.5);
  std::vector<PathPlanner::Point> risky{{5.0, 5.0}, {6.0, 6.0}, {7.0, 5.0}};
  EXPECT_FALSE(planner.smoothPath(risky, 1));
  EXPECT_DOUBLE_EQ(risky[1].x, 6.0);
  EXPECT_DOUBLE_EQ(risky[1].y, 6.0);
}
