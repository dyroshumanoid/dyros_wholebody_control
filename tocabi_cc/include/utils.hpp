#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <casadi/casadi.hpp>

#include <vector>
#include <cstring>

// ================================
// Eigen -> CasADi DM
// ================================
template <typename Derived>
inline casadi::DM EigenToCasadiDM(const Eigen::MatrixBase<Derived> &eigen_mat)
{
    casadi::DM dm = casadi::DM::zeros(eigen_mat.rows(), eigen_mat.cols());
    std::memcpy(dm.ptr(),
                eigen_mat.derived().data(),
                sizeof(double) * eigen_mat.size());
    return dm;
}

// ================================
// CasADi DM -> Eigen Matrix
// ================================
inline Eigen::MatrixXd CasadiDMToEigenMatrix(const casadi::DM &dm)
{
    casadi::Sparsity sp = dm.get_sparsity();

    std::vector<casadi_int> rows, cols;
    sp.get_triplet(rows, cols);

    std::vector<double> values = dm.get_nonzeros();

    using T = Eigen::Triplet<double>;
    std::vector<T> triplets(values.size());

    for (size_t k = 0; k < values.size(); ++k)
        triplets[k] = T(rows[k], cols[k], values[k]);

    Eigen::SparseMatrix<double> spM(dm.size1(), dm.size2());
    spM.setFromTriplets(triplets.begin(), triplets.end());

    return Eigen::MatrixXd(spM);
}

// ================================
// CasADi DM -> Eigen Vector
// ================================
inline Eigen::VectorXd CasadiDMToEigenVector(const casadi::DM &dm)
{
    Eigen::MatrixXd M = CasadiDMToEigenMatrix(dm);
    Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(M.data(), M.size()));
    return v;
}
