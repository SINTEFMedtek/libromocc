#include "Robot.h"
#include <iostream>
#include <thread>
#include <functional>


namespace romocc
{

Robot::Robot()
{
    mCoordinateSystem = RobotCoordinateSystem::New();
    mCommunicationInterface = CommunicationInterface::New();
    mCurrentState = RobotState::New();
}


void Robot::configure(Manipulator manipulator, const std::string& host, const int& port)
{
    mCommunicationInterface->configConnection(host, port);
    mCommunicationInterface->setManipulator(manipulator);
    mCurrentState = mCommunicationInterface->getRobotState();
}

bool Robot::connect()
{
    bool connected = mCommunicationInterface->connectToRobot();
    if(!connected)
        std::cout << "Robot not connected. Please check address." << std::endl;

    return connected;
}

bool Robot::isConnected() const
{
    return mCommunicationInterface->isConnected();
}

bool Robot::disconnect()
{
    if(mActiveSubscription){
        mActiveSubscription = false;
        mThread->join();
    }
    return mCommunicationInterface->disconnectFromRobot();
}

void Robot::shutdown()
{
    mCommunicationInterface->shutdownRobot();
}

RobotState::pointer Robot::getCurrentState() const
{
    return mCurrentState;
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

    double remainingDistance = ((mCurrentState->get_bMee()*mCoordinateSystem->get_eeMt()).translation()
                                -target.targetPose.translation()).norm();

    if(remainingDistance<=target.blendRadius)
    {
        mMotionQueue.erase(mMotionQueue.begin());
        target = mMotionQueue.front();
    }

    if(target.motionType == MotionType::speedj)
    {
        auto targetJointVelocity = RobotMotionUtils::calcJointVelocity(target.targetPose,
                mCurrentState->get_bMee()*mCoordinateSystem->get_eeMt(), mCurrentState->getJacobian(), target.velocity);
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
}

void Robot::addUpdateSubscription(std::function<void()> updateSignal)
{
    mActiveSubscription = true;
    mThread = std::make_unique<std::thread>(std::bind(&Robot::startSubscription, this, updateSignal));
}

void Robot::startSubscription(std::function<void()> updateSignal)
{
    void *subscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
    std::string msg_buffer;
    zmq_connect(subscriber, "inproc://state_update_notifier");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
    zmq_msg_t message;
    zmq_msg_init(&message);

    while(mActiveSubscription)
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
    if(mActiveSubscription){
        mActiveSubscription = false;
        mThread->join();
    }
}

}