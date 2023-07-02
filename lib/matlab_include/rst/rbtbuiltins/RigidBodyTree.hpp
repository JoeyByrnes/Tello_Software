/* Copyright 2022 The MathWorks, Inc. */
#ifndef RIGIDBODYTREE_HPP
#define RIGIDBODYTREE_HPP

#include "RigidBody.hpp"

namespace rbtcodegen {

/** A rigid body tree
 *
 * References:
 * [1] Featherstone, Roy. Rigid body dynamics algorithms. Springer, 2014.
 */
class RBTCODEGEN_API RigidBodyTree {
private:
  /// A vector of rigid bodies
  std::vector<RigidBody> m_RigidBodyVec;

  /** Map degree of freedom of a body to a configuration vector
   *
   * Given a joint configuration vector, each body in the rigid body tree maps
   * to values in the configuration.
   */
  std::vector<std::array<int32_T, 2>> m_DOFMap;

  /** Identify bodies in the serial chain from a body to the root of the tree
   *
   * Mark indices of the series of bodies which trace back to the root from the
   * body at input index.
   * @param bidx Index of the body
   * @return chainmask A vector indicating 1 for a body index which is a part of
   * the serial chain from the body at bidx.
   */
  std::vector<int32_T> identifyEESerialChainBodies(size_t bidx) const;

public:
  /// Default constructor
  RigidBodyTree() : m_RigidBodyVec{}, m_DOFMap{} {}

  /// Construct a rigid body tree from a vector of rigid bodies and DOF map
  RigidBodyTree(const std::vector<RigidBody> &rbvec,
                const std::vector<std::array<int32_T, 2>> &dofmap)
      : m_RigidBodyVec(rbvec), m_DOFMap(dofmap) {}

  /** Forward kinematics routine of a rigid body tree.
   *
   * For a given joint configuration, compute the homogeneous transformations
   * of the body frames with respect to the base frame of the rigid body tree.
   *
   * @param q A joint configuration of the rigid body tree
   * @return tforms A vector of homogeneous transformations where each
   * transformation corresponds to a body in the tree
   */
  std::vector<HomogeneousTransform>
  forwardKinematics(const std::vector<real64_T> &q) const;

  /** Geometric Jacobian routine of a rigid body tree.
   *
   * For a given joint configuration, compute the geometric Jacobian
   * of the body frames with respect to the base frame of the rigid body tree.
   *
   * @param q A joint configuration of the rigid body tree
   * @return jac A vector of geometric jacobians, where each jacobian
   * corresponds to a body in the tree
   */
  std::vector<std::vector<std::array<real64_T, 6>>>
  geometricJacobians(const std::vector<real64_T> &q) const;

  /// Size is the number of bodies on the tree
  inline size_t size() const { return m_RigidBodyVec.size(); }

  /** Geometric Jacobian routine of a rigid body tree.
   *
   * For a given set of joint configurations, compute the geometric Jacobian
   * of the body frames with respect to the base frame of the rigid body tree.
   *
   * @param[in] configarr An array of joint configurations.
   * @param[in] numconfig Number of configurations in configarr
   * @param[in] configsize The size of a configuration in configarr which is
   * also the DOF of the rigid body tree.
   * @param[out] jacout An array of geometric jacobians for every configuration
   * in configarr, where each jacobian corresponds to a body in the tree.
   */
  void geometricJacobians(const real64_T *configarr, size_t numconfig,
                          size_t configsize, real64_T *jacout) const;

  /** Forward kinematics routine of a rigid body tree.
   *
   * For a given set of joint configurations, compute the homogeneous
   * transformations of the body frames with respect to the base frame of the
   * rigid body tree.
   *
   * @param[in] configarr An array of joint configurations.
   * @param[in] numconfig Number of configurations in configarr
   * @param[in] configsize The size of a configuration in configarr which is
   * also the DOF of the rigid body tree.
   * @param[out] tforms An array of homogeneous transformations for every
   * configuration in configarr, where each transformation corresponds to a
   * body in the tree.
   */
  void forwardKinematics(const real64_T *configarr, size_t numconfig,
                         size_t configsize, real64_T *tforms) const;
};
} // namespace rbtcodegen
#endif
