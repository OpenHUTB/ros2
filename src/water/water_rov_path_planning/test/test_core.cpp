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

  PathPlanner planner(PathPlanner::Params());
  planner.setEnvironment(heatmap, current);

  // 单步 1 格 = 0.5 m，长度项 0.5
  double favourable = planner.stepCost(10, 10, 11, 10);  // 顺流
  double cross = planner.stepCost(10, 10, 10, 11);       // 横向
  double adverse = planner.stepCost(10, 10, 9, 10);      // 逆流

  EXPECT_NEAR(favourable, 0.38, 1e-9);
  EXPECT_NEAR(cross, 0.50, 1e-9);
  EXPECT_NEAR(adverse, 0.74, 1e-9);
  EXPECT_LT(favourable, cross);
  EXPECT_LT(cross, adverse);
}

TEST(PathPlannerSmooth, JacobiAverageAndRiskFallback)
{
  Heatmap::Params hp;
  hp.grid_size = 100;
  hp.resolution = 0.5;
  auto heatmap = std::make_shared<Heatmap>(hp);

  PathPlanner planner(PathPlanner::Params());
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
