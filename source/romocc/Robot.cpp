#include "Robot.h"
#include <iostream>
#include <thread>
#include <functional>

namespace romocc
{

Robot::Robot()
{
    mCommunicationInterface = CommunicationInterface::New();
    mCommunicationInterface->registerObserver(this);
}


void Robot::configure(Manipulator manipulator, const std::string& host, const int& port)
{
    mCommunicationInterface->set_communication_protocol(manipulator);
    mCommunicationInterface->config_connection(host, port);
    mCurrentState.set_kdlchain(manipulator);
}

bool Robot::start()
{
    mCommunicationInterface->getNotifier()->setupNotifier(5557);
    return mCommunicationInterface->connectToRobot();
}

bool Robot::isConnected() const
{
    return mCommunicationInterface->isConnected();
}

bool Robot::disconnectFromRobot()
{
    return mCommunicationInterface->disconnectFromRobot();
}

void Robot::shutdown()
{
    mCommunicationInterface->shutdownRobot();
}

RobotState Robot::getCurrentState() const
{
    return mCurrentState;
}

void Robot::update()
{
    JointState state = mCommunicationInterface->getCurrentState();
    mCurrentState.set_jointState(state.jointConfiguration, state.jointVelocity, state.timestamp);
    mCommunicationInterface->getNotifier()->broadcastUpdate();
}

void Robot::runMotionQueue(MotionQueue queue)
{
    mMotionQueue = std::move(queue);
    RobotMotion target = mMotionQueue.front();
    target.targetPose.translation() = target.targetPose.translation()/1000;
    this->move(MotionType::movep, target.targetPose, target.acceleration, target.velocity);
}

void Robot::stopRunMotionQueue()
{
    this->stopMove(MotionType::stopj, 1.0);
}

void Robot::waitForMove()
{
    auto target = mMotionQueue.front();

    double remainingDistance = ((mCurrentState.bMee*eeMt).translation()-target.targetPose.translation()).norm();

    if(remainingDistance<=target.blendRadius)
    {
        mMotionQueue.erase(mMotionQueue.begin());
        target = mMotionQueue.front();
    }

    if(target.motionType == MotionType::speedj)
    {
        auto targetJointVelocity = RobotMotionUtils::calcJointVelocity(target.targetPose,
                mCurrentState.bMee*eeMt, mCurrentState.getJacobian(), target.velocity);
        this->move(target.motionType, targetJointVelocity, target.acceleration, 0.0, 5.0, 0.0); // vel and r not used
    }


    if(mMotionQueue.empty())
    {
        this->stopMove(MotionType::stopj, target.acceleration);
    }
}

void Robot::stopMove(MotionType type, double acc)
{
    mCommunicationInterface->stopMove(type, acc);
};


void Robot::set_eeMt(Eigen::Affine3d eeMt)
{
    this->eeMt = eeMt;
}

void Robot::set_rMb(Eigen::Affine3d rMb)
{
    this->rMb = rMb;
}

Transform3d Robot::get_rMt()
{
    return rMb*getCurrentState().bMee*eeMt;
}

Transform3d Robot::get_rMb()
{
    return rMb;
}

Transform3d Robot::get_eeMt()
{
    return eeMt;
}

void Robot::updateSubscription(std::function<void()> updateSignal)
{
    std::thread thread_(std::bind(&Robot::newSubscription, this, updateSignal));
    thread_.detach();
}

void Robot::newSubscription(std::function<void()> updateSignal)
{
    void *subscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
    std::string msg_buffer;
    zmq_connect(subscriber, "tcp://localhost:5557");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
    zmq_msg_t message;
    zmq_msg_init(&message);

    while(true)
    {
        int rc = zmq_recvmsg(subscriber, &message, 0);
        assert(rc == 0);
        int size = zmq_msg_size(&message);
        msg_buffer.assign((const char *) zmq_msg_data(&message), size);
        updateSignal();
    }
}

Robot::~Robot()
{

}

}