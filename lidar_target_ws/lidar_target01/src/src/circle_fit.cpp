#include <cmath>
#include <random>
#include "tracker/circle_fit.h"

namespace tracker
{

bool computeCircleFrom3Pts(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    double& cx, double& cy, double& r)
{
  // 提取坐标
  const double x1 = a.x(), y1 = a.y();
  const double x2 = b.x(), y2 = b.y();
  const double x3 = c.x(), y3 = c.y();

  // 计算行列式参数
  const double A = x2 - x1;
  const double B = y2 - y1;
  const double C = x3 - x1;
  const double D = y3 - y1;

  const double E = A*(x1 + x2) + B*(y1 + y2);
  const double F = C*(x1 + x3) + D*(y1 + y3);
  const double G = 2.0*(A*(y3 - y2) - B*(x3 - x2));

  // 检查三点是否共线
  if (std::fabs(G) < 1e-12) return false;

  // 计算圆心和半径
  cx = (D*E - B*F) / G;
  cy = (A*F - C*E) / G;
  r = std::hypot(cx - x1, cy - y1);

  // 验证数值有效性
  return std::isfinite(cx) && std::isfinite(cy) && std::isfinite(r);
}

CircleFitResult ransacCircle2D(
    const std::vector<Eigen::Vector2d>& pts,
    int iters,
    double r_min, double r_max,
    double inlier_th)
{
  CircleFitResult best;
  if (pts.size() < 3) return best;

  // 固定种子确保可重复性
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> uni(0, static_cast<int>(pts.size()) - 1);

  // RANSAC迭代
  for (int k = 0; k < iters; ++k)
  {
    // 随机采样3个点
    int i1 = uni(rng), i2 = uni(rng), i3 = uni(rng);
    if (i1 == i2 || i1 == i3 || i2 == i3) continue;

    // 拟合圆
    double cx, cy, r;
    if (!computeCircleFrom3Pts(pts[i1], pts[i2], pts[i3], cx, cy, r)) continue;

    // 检查半径约束
    if (r < r_min || r > r_max) continue;

    // 计算内点
    int inliers = 0;
    double err_sum = 0.0;

    for (const auto& p : pts)
    {
      double d = std::hypot(p.x() - cx, p.y() - cy);
      double e = std::fabs(d - r);
      if (e < inlier_th)
      {
        inliers++;
        err_sum += e;
      }
    }

    if (inliers <= 0) continue;
    double mean_err = err_sum / static_cast<double>(inliers);

    // 更新最佳结果: 优先选择内点多的，内点数相同时选择误差小的
    if (!best.ok || inliers > best.inliers ||
        (inliers == best.inliers && mean_err < best.mean_abs_err))
    {
      best.ok = true;
      best.cx = cx;
      best.cy = cy;
      best.r = r;
      best.inliers = inliers;
      best.mean_abs_err = mean_err;
    }
  }
  return best;
}

} // namespace tracker
