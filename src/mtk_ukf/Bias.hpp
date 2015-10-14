#ifndef _BIAS_STATE_HPP_
#define _BIAS_STATE_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <Eigen/Core>

namespace pose_estimation
{

template<int dof>
struct Bias
{
    enum {BiasDOF = MTK::vect<dof>::DOF};
    enum {BiasIdx = 0};
public:
    typedef MTK::vect<dof> bias_type;
    typedef typename bias_type::scalar scalar;
    typedef Bias self;
    enum {DOF = BiasDOF};
    typedef Eigen::Matrix<unsigned, DOF, 1> map_type;

    // State types
    MTK::SubManifold<MTK::vect<dof>, BiasIdx> bias;

    // Helper
    map_type bias_map;

    // Construct from pose and velocities
    Bias(const bias_type& bias = bias_type()) : bias(bias), bias_map(map_type::Zero()) {}

    // Copy constructor
    template<int dof2>
    Bias(const Bias<dof2> &state) : bias(state.bias), bias_map(map_type::Zero()) {}

    void boxplus(MTK::vectview<const scalar, DOF> state_vec, scalar _scale = 1)
    {
        bias.boxplus(MTK::subvector(state_vec, &Bias::bias), _scale);
    }
    void boxminus(MTK::vectview<scalar, DOF> state_ret, const Bias &other) const
    {
        bias.boxminus(MTK::subvector(state_ret, &Bias::bias), other.bias);
    }

    friend std::ostream& operator<<(std::ostream &os, const Bias<dof> &state)
    {
        return os << state.bias;
    }
    friend std::istream& operator>>(std::istream &is, Bias<dof> &state)
    {
        return is >> state.bias;
    }

    void setBiasMap(const map_type& map)
    {
        bias_map = map;

        // make sure the indeces in the map are ordered
        std::sort(bias_map.data(), bias_map.data()+bias_map.size());
    }

    map_type getBiasMap()
    {
        return bias_map;
    }

    template<int mask_dim>
    Eigen::Matrix<scalar, -1, 1> getBiasForSubState(const Eigen::Matrix<unsigned, mask_dim, 1>& mask) const
    {
        Eigen::Matrix<scalar, -1, 1> bias_state;
        bias_state.resize(mask.count());
        bias_state.setZero();

        // populate bias sub state
        for(unsigned i = 0; i < DOF; i++)
        {
            if(bias_map[i] < mask.rows() && mask[bias_map[i]] > 0)
            {
                bias_state[i] = bias[i];
            }
        }

        return bias_state;
    }
};

}

#endif