//
// Created by androst on 02.07.18.
//

#ifndef CORAH_MESSAGEENCODER_H
#define CORAH_MESSAGEENCODER_H

#include <QObject>
#include <Eigen/Dense>

class MessageEncoder {
public:
    virtual QString moveJoints(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t, double rad) = 0;
    virtual QString movePose(Eigen::Affine3d pose, double acc, double vel, double t, double rad) = 0;
};


#endif //CORAH_MESSAGEENCODER_H
