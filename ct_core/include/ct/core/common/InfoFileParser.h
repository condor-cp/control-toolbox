/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>


namespace ct {
namespace core {

/*!
 * \brief Loads a scalar file from an .info file
 *
 * \warning This function will throw an exception if the scalar is not found. For a non-throwing
 * version, see loadScalarOptional()
 *
 * @param filename the full or relative path of the file to load
 * @param scalarName the name of the scalar in the info file
 * @param scalar value to store
 * @param ns the namespace that the scalar lives in
 *
 * @tparam SCALAR the scalar type, i.e. double, int, size_t etc.
 */
template <typename SCALAR>
void loadScalar(const std::string& filename, const std::string& scalarName, SCALAR& scalar, const std::string& ns = "")
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(ns + scalarName);
}

/*!
 * \brief Tries to load a scalar from an .info file. Falls back to default value if not found
 *
 * @param filename the full or relative path of the file to load
 * @param scalarName the name of the scalar in the info file
 * @param scalar value to store
 * @param defaultValue the default value to fall back to. Can be the same as scalar.
 * @param ns the namespace that the scalar lives in
 *
 * @tparam SCALAR the scalar type, i.e. double, int, size_t etc.
 */
template <typename SCALAR>
void loadScalarOptional(const std::string& filename,
    const std::string& scalarName,
    SCALAR& scalar,
    const SCALAR& defaultValue,
    const std::string& ns = "")
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    scalar = pt.get<SCALAR>(ns + scalarName, defaultValue);
}

/*!
 * \brief Loads a matrix/vector file from an .info file
 *
 * This function supports sparse storage of vectors matrices, i.e. all entries not found in the
 * .info file are assumed to be zero. Also, it looks for parameters called "scaling" (defaults
 * to 1), "fill" (defaults to 0) and "diagonal". The matrix is filled with the following logic : 
 * all fields are filled with the "fill" value, or the "diagonal" value (if specified) for diagonal elements. 
 * All the sparse elements specified will override. All the elements of the matrix are then scaled 
 * (multiplied by the "scaling" value).
 * The example file:
 *
 * * \code{.txt}
 * M
 * {
 * 		scaling 2.5
 *      fill -1.0
 *      diagonal 3.0
 * 
 * 		(0,0) 10.0
 * 		(1,1) 20.0
 * 		(2,2) 0.0
 * 		(2,0) 1.0
 * }
 * \endcode
 *
 * would generate the following matrix
 *
 * \f[
 * 		M = 2.5 \cdot
 * 		\begin{pmatrix}
 *         10 & -1.0 & -1.0 \\
 *         -1.0 & 2.0 & -1.0 \\
 *         1.0 & -1.0 & 3.0
 *     \end{pmatrix}
 *     =
 *     \begin{pmatrix}
 *         25 & -2.5 & -2.5 \\
 *         -2.5 & 50.0 & -2.5 \\
 *         -2.5 & -2.5 & 7.5
 *     \end{pmatrix}
 * \f]
 *
 * \warning If the matrix is not found, it defaults all entries to zero.
 *
 * @param filename the full or relative path of the file to load
 * @param matrixName the name of the matrix in the info file
 * @param matrix value to store
 * @param ns the namespace that the matrix lives in
 *
 * @tparam SCALAR the scalar type, i.e. double, int, size_t etc.
 * @tparam ROW number of rows (needs to be positive!)
 * @tparam COL numer of columns (needs to be positive!)
 */
template <typename SCALAR, int ROW, int COL>
void loadMatrix(const std::string& filename,
    const std::string& matrixName,
    Eigen::Matrix<SCALAR, ROW, COL>& matrix,
    const std::string& ns = "")
{
    size_t rows = matrix.rows();
    size_t cols = matrix.cols();

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    double scaling = pt.get<double>(ns + matrixName + ".scaling", 1);
    double fill = pt.get<double>(ns + matrixName + ".fill", 0);
    boost::optional<double> specified_diagonal = pt.get_optional<double>(ns + matrixName + ".diagonal");

    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            if (boost::optional<double> specified_value = pt.get_optional<double>(
                    ns + matrixName + "." + "(" + std::to_string(i) + "," + std::to_string(j) + ")"))
                matrix(i, j) = scaling * *specified_value;
            else
            {
                matrix(i, j) = scaling * fill;
                if (i == j && specified_diagonal)
                    matrix(i, j) = scaling * *specified_diagonal;
            }
        }
    }
}
}  // namespace core
}  // namespace ct
