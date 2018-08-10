//
// Created by androst on 10.08.18.
//

#include "CalibrationMethods.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>

CalibrationMatrices CalibrationMethods::Shah(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee)
{
    // Inspired by Shah method
    CalibrationMatrices cMatrices;

    size_t nMatrices = prMt.size();

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(9,9);

    for(int i = 0; i<nMatrices; i++)
    {
        Eigen::MatrixXd Ra = bMee.at(i).rotation().inverse();
        Eigen::MatrixXd Rb = prMt.at(i).rotation().inverse();
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

    //Ry.resize(9,1);
    for(int i = 0; i<nMatrices; i++)
    {
//        AA.block<3,3>(3*i,0) = -(bMee.at(i).rotation()).inverse();
//        AA.block<3,3>(3*i,3) = Eigen::MatrixXd::Identity(3,3);
//        b.block<3,1>(3*i,0) = (bMee.at(i).inverse()).translation()-
//                                (Eigen::kroneckerProduct((prMt.at(i).inverse()).translation().transpose(), Eigen::MatrixXd::Identity(3,3)))*(Ry);
        AA.block<3,3>(3*i,0) = (bMee.at(i).rotation()).inverse();
        AA.block<3,3>(3*i,3) = -Eigen::MatrixXd::Identity(3,3);
        b.block<3,1>(3*i,0) = Ry*(prMt.at(i).inverse().translation())-(bMee.at(i).inverse()).translation();
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

    cMatrices.prMb = Transform3d(bMpr).inverse();
    cMatrices.eeMt = Transform3d(eMt);

    return cMatrices;
}

CalibrationMatrices CalibrationMethods::Li(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee)
{
    // Inspired by Li method
    CalibrationMatrices cMatrices;

    size_t nMatrices = prMt.size();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12*nMatrices, 24);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(12*nMatrices,1);

    for(int i = 0; i<nMatrices; i++)
    {
        Eigen::MatrixXd Ra = bMee.at(i).rotation().inverse();
        Eigen::Vector3d ta = bMee.at(i).inverse().translation();

        Eigen::MatrixXd Rb = prMt.at(i).rotation().inverse();
        Eigen::Vector3d tb = prMt.at(i).inverse().translation();

        A.block<9,9>(12*i,0) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(3,3),Ra);
        A.block<9,9>(12*i,9) = -Eigen::kroneckerProduct(Rb.transpose(),Eigen::MatrixXd::Identity(3,3));
        A.block<3,9>(12*i+9,9) = Eigen::kroneckerProduct(tb.transpose(),Eigen::MatrixXd::Identity(3,3));
        A.block<3,3>(12*i+9,18) = -Ra;
        A.block<3,3>(12*i+9,21) = Eigen::MatrixXd::Identity(3,3);

        b.block<3,1>(12*i+9,0) = ta;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
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

    cMatrices.prMb = Transform3d(bMpr).inverse();
    cMatrices.eeMt = Transform3d(eMt);

    return cMatrices;
}

CalibrationMatrices CalibrationMethods::Park(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee)
{
    // Inspired by Park method
    int nMatrices = int(prMt.size());

    std::vector<Transform3d> A, B, AA, BB;

    std::vector<Transform3d> eMb = invert_matrices(bMee);
    std::vector<Transform3d> tMpr = invert_matrices(prMt);

    for(int i = 0; i<nMatrices; i++)
    {
        for(int j=i+1; j<nMatrices; j++)
        {
            A.push_back(prMt.at(j)*prMt.at(i).inverse());
            B.push_back(bMee.at(j)*bMee.at(i).inverse());
            AA.push_back(eMb.at(j)*eMb.at(i).inverse());
            BB.push_back(tMpr.at(j)*tMpr.at(i).inverse());
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

    for(int i = 0; i<A.size(); i++)
    {
        if(AffineToAxisAngle(A.at(i)).norm()!=0 && AffineToAxisAngle(B.at(i)).norm()!=0)
        {
            a = AffineToAxisAngle(A.at(i));
            b = AffineToAxisAngle(B.at(i));
            aa = AffineToAxisAngle(AA.at(i));
            bb = AffineToAxisAngle(BB.at(i));
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

    Eigen::MatrixXd C(A.size()*3,3);
    Eigen::MatrixXd d(A.size()*3,1);
    Eigen::MatrixXd CC(A.size()*3,3);
    Eigen::MatrixXd dd(A.size()*3,1);

    int j = 0;
    for(int i = 0; i < (A.size()*3); i=i+3)
    {
        C.block<3,3>(i,0) = I.rotation()-A.at(j).rotation();
        d.block<3,1>(i,0) = A.at(j).translation()-iMk.block<3,3>(0,0)*B.at(j).translation();
        CC.block<3,3>(i,0) = I.rotation()-AA.at(j).rotation();
        dd.block<3,1>(i,0) = AA.at(j).translation()-lMj.block<3,3>(0,0)*BB.at(j).translation();
        j++;
    }

    iMk.block<3,1>(0,3) = (C.transpose()*C).inverse()*C.transpose()*d;
    lMj.block<3,1>(0,3) = (CC.transpose()*CC).inverse()*CC.transpose()*dd;
    Transform3d jMl = Transform3d(lMj).inverse();

    CalibrationMatrices calibMatrices;
    calibMatrices.prMb = Transform3d(iMk);
    calibMatrices.eeMt = Transform3d(lMj);

    return calibMatrices;
}

CalibrationError CalibrationMethods::estimateCalibrationError(Transform3d prMb, Transform3d eeMt, std::vector<Transform3d> bMee, std::vector<Transform3d> prMt)
{
    CalibrationError calibrationError;

    int nMatrices = int(bMee.size());

    std::vector<double> rotError, transError;

    for(int i = 0; i < nMatrices; i++)
    {
        Transform3d errorMatrix = prMb*bMee.at(i)*eeMt*prMt.at(i).inverse();
        rotError.push_back(AffineToAxisAngle(errorMatrix).norm());

        Vector3d translError = bMee.at(i).linear()*eeMt.translation()+bMee.at(i).translation()-prMb.linear().inverse()*prMt.at(i).translation()-prMb.inverse().translation();
        transError.push_back(translError.norm());
    }

    calibrationError.rotationError = compute_average(rotError);
    calibrationError.rotStd = sqrt(compute_variance(rotError, calibrationError.rotationError ));
    calibrationError.translationError = compute_average(transError);
    calibrationError.transStd = sqrt(compute_variance(transError, calibrationError.translationError));

    return calibrationError;
}

