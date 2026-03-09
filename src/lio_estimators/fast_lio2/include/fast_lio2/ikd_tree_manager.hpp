// About: ikd-Tree Manager for FAST-LIO2
// Incremental KD-Tree for fast nearest neighbor search

#ifndef FAST_LIO2__IKD_TREE_MANAGER_HPP_
#define FAST_LIO2__IKD_TREE_MANAGER_HPP_

#include <vector>
#include <Eigen/Dense>

namespace juppiter
{
namespace lio
{

/**
 * @brief Wrapper for ikd-Tree (incremental KD-Tree)
 * 
 * ikd-Tree supports incremental updates (add/delete points) with
 * logarithmic time complexity, enabling real-time mapping in FAST-LIO2.
 */
class IkdTreeManager
{
public:
  IkdTreeManager();
  ~IkdTreeManager();

  /**
   * @brief Add points to the tree
   */
  void addPoints(const std::vector<Eigen::Vector3d> & points);

  /**
   * @brief Delete points from the tree
   */
  void deletePoints(const std::vector<Eigen::Vector3d> & points);

  /**
   * @brief Perform k-nearest neighbor search
   * @return Vector of nearest points
   */
  std::vector<Eigen::Vector3d> nearestNeighborSearch(
    const Eigen::Vector3d & query, int k);

  /**
   * @brief Delete points within a box
   */
  void boxDelete(const Eigen::Vector3d & box_min, 
                 const Eigen::Vector3d & box_max);

  /**
   * @brief Get number of points in tree
   */
  size_t size() const;

  /**
   * @brief Clear the tree
   */
  void clear();

private:
  // TODO: Integrate actual ikd-Tree implementation
  // For now, this is a stub
  size_t point_count_{0};
};

}  // namespace lio
}  // namespace juppiter

#endif  // FAST_LIO2__IKD_TREE_MANAGER_HPP_
