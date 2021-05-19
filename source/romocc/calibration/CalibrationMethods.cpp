#include "CalibrationMethods.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>

namespace romocc
{

CalibrationMatrices CalibrationMethods::Shah(std::vector<Transform3d> A, std::vector<Transform3d> B)
{
    // Inspired by Shah method
    // M. Shah 2013, Solving the Robot-World/Hand-Eye Calibration Problem Using the Kronecker Product

    CalibrationMatrices cMatrices;

    size_t nMatrices = A.size();

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(9,9);

    for(int i = 0; i<nMatrices; i++)
    {
        Eigen::MatrixXd Ra = B.at(i).rotation().inverse();
        Eigen::MatrixXd Rb = A.at(i).rotation().inverse();
        K = K+Eigen::kroneckerProduct(Rb,Ra);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(K, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd x = ((svd.matrixV()).block<9,1>(0,0));
    x.resize(3,3);

    Eigen::MatrixXd y = ((svd.matrixU()).block<9,1>(0,0));
    y.resize(3,3);

    x = (sgn(x.determinant())/std::pow(std::fabs(x.determinant()),(double)1.0/3.0))*x;
    y = (sgn(y.determinant())/std::pow(std::fabs(y.determinant()),(double)1.0/3.0))*y;

    Eigen::JacobiSVD<Eigen::MatrixXd> svdX(x , Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdY(y , Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd Rx = svdX.matrixU()*svdX.matrixV().transpose();
    Eigen::MatrixXd Ry = svdY.matrixU()*svdY.matrixV().transpose();

    Eigen::MatrixXd AA = Eigen::MatrixXd::Zero(3*nMatrices, 6);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3*nMatrices, 1);

    for(int i = 0; i<nMatrices; i++)
    {
        AA.block<3,3>(3*i,0) = (B.at(i).rotation()).inverse();
        AA.block<3,3>(3*i,3) = -Eigen::MatrixXd::Identity(3,3);
        b.block<3,1>(3*i,0) = Ry*(A.at(i).inverse().translation())-(B.at(i).inverse()).translation();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svdA(AA,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd t = svdA.solve(b).eval();

    Eigen::Matrix4d bMpr = Eigen::Matrix4d::Identity();
    bMpr.block<3,3>(0,0) = Rx;
    bMpr.block<3,1>(0,3) = t.block<3,1>(0,0);

    Ry.resize(3,3);
    Eigen::Matrix4d eMt = Eigen::Matrix4d::Identity();
    eMt.block<3,3>(0,0) = Ry;
    eMt.block<3,1>(0,3) = t.block<3,1>(3,0);

    cMatrices.X = Transform3d(eMt).inverse();
    cMatrices.Y = Transform3d(bMpr).inverse();

    return cMatrices;
}

CalibrationMatrices CalibrationMethods::Li(std::vector<Transform3d> A, std::vector<Transform3d> B)
{
    // Inspired by Li method
    // A. Li, L.Wang and D. Wu, 2010
    // Simultaneous robot-world and hand-eye calibration using dual-quaternions and Kronecker product

    CalibrationMatrices cMatrices;

    size_t nMatrices = A.size();

    Eigen::MatrixXd AA = Eigen::MatrixXd::Zero(12*nMatrices, 24);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(12*nMatrices,1);

    for(int i = 0; i<nMatrices; i++)
    {
        Eigen::MatrixXd Ra = B.at(i).rotation().inverse();
        Eigen::Vector3d ta = B.at(i).inverse().translation();

        Eigen::MatrixXd Rb = A.at(i).rotation().inverse();
        Eigen::Vector3d tb = A.at(i).inverse().translation();

        AA.block<9,9>(12*i,0) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(3,3),Ra);
        AA.block<9,9>(12*i,9) = -Eigen::kroneckerProduct(Rb.transpose(),Eigen::MatrixXd::Identity(3,3));
        AA.block<3,9>(12*i+9,9) = Eigen::kroneckerProduct(tb.transpose(),Eigen::MatrixXd::Identity(3,3));
        AA.block<3,3>(12*i+9,18) = -Ra;
        AA.block<3,3>(12*i+9,21) = Eigen::MatrixXd::Identity(3,3);

        b.block<3,1>(12*i+9,0) = ta;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(AA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd x = svd.solve(b);

    Eigen::MatrixXd X = x.block<9,1>(0,0);
    Eigen::MatrixXd Y = x.block<9,1>(9,0);

    X.resize(3,3);
    Y.resize(3,3);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdX(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdY(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd Rx = svdX.matrixU()*svdX.matrixV().transpose();
    Eigen::MatrixXd Ry = svdY.matrixU()*svdY.matrixV().transpose();

    if(Rx.determinant()<0)
        Rx = svdX.matrixU()*(Eigen::Vector3d(1,1,-1)).asDiagonal()*svdX.matrixV().transpose();

    if(Ry.determinant()<0)
        Ry = svdY.matrixU()*(Eigen::Vector3d(1,1,-1)).asDiagonal()*svdY.matrixV().transpose();

    Eigen::Matrix4d bMpr = Eigen::Matrix4d::Identity();
    bMpr.block<3,3>(0,0) = Rx;
    bMpr.block<3,1>(0,3) = x.block<3,1>(18,0);

    Eigen::Matrix4d eMt = Eigen::Matrix4d::Identity();
    eMt.block<3,3>(0,0) = Ry;
    eMt.block<3,1>(0,3) = x.block<3,1>(21,0);

    cMatrices.X = Transform3d(eMt).inverse();
    cMatrices.Y = Transform3d(bMpr).inverse();

    return cMatrices;
}

CalibrationMatrices CalibrationMethods::Park(std::vector<Transform3d> A, std::vector<Transform3d> B)
{
    // Inspired by Park method
    // Park and Martin, 1994, Robot sensor calibration: solving AX=XB on the Euclidean group

    int nMatrices = int(A.size());

    std::vector<Transform3d> A_pairs, B_pairs, A_inv_pairs, B_inv_pairs;

    std::vector<Transform3d> A_inv = invert_matrices(A);
    std::vector<Transform3d> B_inv = invert_matrices(B);

    for(int i = 0; i<nMatrices; i++)
    {
        for(int j=i+1; j<nMatrices; j++)
        {
            A_pairs.push_back(A.at(j)*A.at(i).inverse());
            B_pairs.push_back(B.at(j)*B.at(i).inverse());
            A_inv_pairs.push_back(A_inv.at(j)*A_inv.at(i).inverse());
            B_inv_pairs.push_back(B_inv.at(j)*B_inv.at(i).inverse());
        }
    }

    Eigen::Matrix3d M, MM;
    M.setZero();
    MM.setZero();

    Eigen::Matrix4d iMk, lMj;
    iMk.setIdentity();
    lMj.setIdentity();

    Transform3d I = Transform3d::Identity();

    Eigen::Vector3d a, b, aa, bb;

    for(int i = 0; i< A_pairs.size(); i++)
    {

        if(TransformUtils::Affine::toAxisAngle(A_pairs.at(i)).norm()!=0 &&
        TransformUtils::Affine::toAxisAngle(B_pairs.at(i)).norm()!=0)
        {
            a = TransformUtils::Affine::toAxisAngle(A_pairs.at(i));
            b = TransformUtils::Affine::toAxisAngle(B_pairs.at(i));
            aa = TransformUtils::Affine::toAxisAngle(B_inv_pairs.at(i));
            bb = TransformUtils::Affine::toAxisAngle(A_inv_pairs.at(i));
        }
        else
        {
            a << 0, 0, 0;
            b << 0, 0, 0;
            aa << 0, 0, 0;
            bb << 0, 0, 0;
        }
        M = M + Eigen::Matrix3d(b*a.transpose());
        MM = MM + Eigen::Matrix3d(bb*aa.transpose());
    }

    iMk.block<3,3>(0,0) = ((M.transpose()*M).sqrt()).inverse()*M.transpose();
    lMj.block<3,3>(0,0) = ((MM.transpose()*MM).sqrt()).inverse()*MM.transpose();

    Eigen::MatrixXd C(A_pairs.size()*3,3);
    Eigen::MatrixXd d(A_pairs.size()*3,1);
    Eigen::MatrixXd CC(A_pairs.size()*3,3);
    Eigen::MatrixXd dd(A_pairs.size()*3,1);

    int j = 0;
    for(int i = 0; i < (A_pairs.size()*3); i=i+3)
    {
        C.block<3,3>(i,0) = I.rotation()-A_pairs.at(j).rotation();
        d.block<3,1>(i,0) = A_pairs.at(j).translation()-iMk.block<3,3>(0,0)*B_pairs.at(j).translation();
        CC.block<3,3>(i,0) = I.rotation()-B_inv_pairs.at(j).rotation();
        dd.block<3,1>(i,0) = B_inv_pairs.at(j).translation()-lMj.block<3,3>(0,0)*A_inv_pairs.at(j).translation();
        j++;
    }

    iMk.block<3,1>(0,3) = (C.transpose()*C).inverse()*C.transpose()*d;
    lMj.block<3,1>(0,3) = (CC.transpose()*CC).inverse()*CC.transpose()*dd;
    Transform3d jMl = Transform3d(lMj).inverse();

    CalibrationMatrices calibMatrices;
    calibMatrices.X = Transform3d(jMl);
    calibMatrices.Y = Transform3d(iMk);

    return calibMatrices;
}

CalibrationError CalibrationMethods::estimateCalibrationError(Transform3d X, Transform3d Y, std::vector<Transform3d> A, std::vector<Transform3d> B)
{
    CalibrationError calibrationError;

    int nMatrices = int(A.size());

    std::vector<double> rotError, transError;

    for(int i = 0; i < nMatrices; i++)
    {
        Transform3d errorMatrix = A.at(i)*X*(Y*B.at(i)).inverse();
        rotError.push_back(TransformUtils::Affine::toAxisAngle(errorMatrix).norm());

        Vector3d translError = A.at(i).linear()*X.translation()+A.at(i).translation()-Y.linear()*B.at(i).translation()-Y.translation();
        transError.push_back(translError.norm());
    }

    calibrationError.rotationError = compute_average(rotError);
    calibrationError.rotStd = sqrt(compute_variance(rotError, calibrationError.rotationError ));
    calibrationError.translationError = compute_average(transError);
    calibrationError.transStd = sqrt(compute_variance(transError, calibrationError.translationError));

    return calibrationError;
}

}
