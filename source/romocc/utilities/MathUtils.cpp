#include "MathUtils.h"
#include "iostream"

namespace romocc
{

Eigen::Affine3d TransformUtils::Affine::scaleTranslation(Eigen::Affine3d affine, double scale)
{
    affine.translation() = affine.translation()*scale;
    return affine;
}

Eigen::Vector3d TransformUtils::Affine::toAxisAngle(Eigen::Affine3d affine)
{
    return (Eigen::AngleAxisd(affine.linear())).angle()*(Eigen::AngleAxisd(affine.linear())).axis();
}

Eigen::Matrix<double,6,1> TransformUtils::Affine::toVector6D(Eigen::Affine3d affine)
{
    Vector6d vector;
    Eigen::Vector3d pos = affine.translation();
    auto angle = Eigen::AngleAxisd(affine.linear()).angle();
    Eigen::Vector3d rxryrz = angle*Eigen::AngleAxisd(affine.linear()).axis();
    vector << pos(0), pos(1), pos(2), rxryrz(0), rxryrz(1), rxryrz(2);
    return vector;
}

Eigen::Affine3d TransformUtils::Affine::toAffine3DFromVector6D(Eigen::Matrix<double, 6, 1> vec) {
    Eigen::Vector3d aa_vec{vec(3), vec(4), vec(5)};

    auto angle = aa_vec.norm();
    auto rot_mat = Eigen::AngleAxisd::Identity().toRotationMatrix();

    if(angle != 0){
        auto norm_aa_vec = aa_vec/aa_vec.norm();
        auto aa_obj = Eigen::AngleAxisd(angle, norm_aa_vec);
        rot_mat = aa_obj.toRotationMatrix();
    }

    auto matrix = Eigen::Affine3d::Identity();
    matrix.translate(Eigen::Vector3d(vec(0), vec(1), vec(2)));
    matrix.linear() = rot_mat;

    return matrix;
}

double TransformUtils::norm(Vector6d a, Vector6d b){
    return (a-b).norm();
}

double TransformUtils::norm(Transform3d a, Transform3d b){
    return (a.matrix()-b.matrix()).norm();
}

double TransformUtils::norm(Transform3d a, Vector6d b){
        return (a.matrix()-TransformUtils::Affine::toAffine3DFromVector6D(b).matrix()).norm();
}

double TransformUtils::norm(Transform3d a, double b[6]){
    std::cout << a.matrix() << std::endl;
    Eigen::RowVectorXd vector(6, 1);
    vector << b[0], b[1], b[2], b[3], b[4], b[5];
    std::cout << vector.matrix() << std::endl;
    return (a.matrix()-TransformUtils::Affine::toAffine3DFromVector6D(vector).matrix()).norm();
}

Eigen::Affine3d TransformUtils::kdl::toAffine(KDL::Frame frame)
{
    Eigen::Affine3d affine;
    affine.translation() << frame.p.data[0], frame.p.data[1], frame.p.data[2];

    affine.linear() <<  frame.M.data[0], frame.M.data[1], frame.M.data[2],
            frame.M.data[3], frame.M.data[4], frame.M.data[5],
            frame.M.data[6], frame.M.data[7], frame.M.data[8];

    return affine;
}

template<typename T>
void transformEigenToKDLImpl(const T &e, KDL::Frame &k)
{
    for (unsigned int i = 0; i < 3; ++i)
        k.p[i] = e(i, 3);
    for (unsigned int i = 0; i < 9; ++i)
        k.M.data[i] = e(i/3, i%3);
}

KDL::Frame TransformUtils::kdl::fromAffine(Eigen::Affine3d affine) {
    auto frame = KDL::Frame::Identity();
    transformEigenToKDLImpl(affine, frame);
    return frame;
}

KDL::JntArray TransformUtils::kdl::fromVector6D(Eigen::Matrix<double, 6, 1> vector) {
    auto jnt_array = KDL::JntArray(6);
    for(int i = 0; i < 6; ++i)
        jnt_array(i) = vector[i];
    return jnt_array;
}

}