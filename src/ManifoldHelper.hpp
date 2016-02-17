#ifndef _POSE_ESTIMATION_MANIFOLD_HELPER_HPP_
#define _POSE_ESTIMATION_MANIFOLD_HELPER_HPP_

namespace pose_estimation
{

/**
 * Undefines helper function that should be specialized by manifold types
 */
template<typename StateType, typename MatrixType>
void getStateVector( const StateType& state, MatrixType& state_vector );


}

#endif