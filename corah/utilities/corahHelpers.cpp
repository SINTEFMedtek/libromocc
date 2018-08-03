//
// Created by androst on 06.07.18.
//

#include "corahHelpers.h"

Eigen::Affine3d KDLFrameToEigenAffine(KDL::Frame frame)
{
    Eigen::Affine3d affine;
    affine.translation() << frame.p.data[0], frame.p.data[1], frame.p.data[2];

    affine.linear() <<  frame.M.data[0], frame.M.data[1], frame.M.data[2],
                        frame.M.data[3], frame.M.data[4], frame.M.data[5],
                        frame.M.data[6], frame.M.data[7], frame.M.data[8];

    return affine;
}

Eigen::Affine3d ScaleTranslationAffine(Eigen::Affine3d affine, double scale)
{
    affine.translation() = affine.translation()*1000;
    return affine;
}

Eigen::RowVectorXd AffineToRowVector(Eigen::Affine3d affine)
{
    Eigen::RowVectorXd vector(6);

    Eigen::Vector3d pos = affine.translation();
    Eigen::Vector3d rxryrz = (Eigen::AngleAxisd(affine.linear())).angle()*(Eigen::AngleAxisd(affine.linear())).axis();

    vector << pos(0), pos(1), pos(2), rxryrz(0), rxryrz(1), rxryrz(2);

    return vector;
}