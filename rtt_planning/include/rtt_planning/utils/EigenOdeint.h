/*
 * rtt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rtt_planning.
 *
 * rtt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rtt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rtt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_UTILS_EIGENODEINT_H_
#define INCLUDE_UTILS_EIGENODEINT_H_

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

// define arma::vec as resizeable

namespace boost
{
namespace numeric
{
namespace odeint
{

template<>
struct is_resizeable<Eigen::VectorXd>
{
    typedef boost::true_type type;
    static const bool value = type::value;
};

template<>
struct same_size_impl<Eigen::VectorXd, Eigen::VectorXd>
{
    // define how to check size
    static bool same_size(const Eigen::VectorXd& v1,
                          const Eigen::VectorXd& v2)
    {
        return v1.size() == v2.size();
    }
};

template<>
struct resize_impl<Eigen::VectorXd, Eigen::VectorXd>
{
    // define how to resize
    static void resize(Eigen::VectorXd& v1,
                       const Eigen::VectorXd& v2)
    {
        v1.resize(v2.size());
    }
};

}//end namespace odeint
}//end namespace numeric
}//end namespace boost


#endif /* INCLUDE_UTILS_EIGENODEINT_H_ */
