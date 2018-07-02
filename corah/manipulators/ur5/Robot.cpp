#include "cxUr5Robot.h"
#include "cxLogger.h"

namespace cx
{

Ur5Robot::Ur5Robot():
    isMoveInProgress(false),
    isVelocityMoveInProgress(false),
    mBlendRadius(1),
    rtPort(30003),
    secPort(30002),
    prMb(Transform3D::Identity()),
    eMt(Transform3D::Identity())
{
    connect(&mRTMonitor,&Ur5Connection::stateChanged,this,&Ur5Robot::updateCurrentState);
    connect(&mSecMonitor,&Ur5Connection::stateChanged,this,&Ur5Robot::updateCurrentState); 

    this->initializeOfflineRobot();
}

Ur5Robot::~Ur5Robot()
{
}

void Ur5Robot::initializeOfflineRobot()
{
    Ur5State currentState;

    currentState.jointConfiguration << 0,-M_PI/2,0,-M_PI/2,0,0;
    currentState.bMee = Ur5Kinematics::forward(currentState.jointConfiguration);
    currentState.cartAxis = Ur5Kinematics::T2transl(currentState.bMee);
    currentState.cartAngles = Ur5Kinematics::T2rangles(currentState.bMee);
    currentState.jacobian = Ur5Kinematics::jacobian2(currentState.jointConfiguration);

    this->setCurrentState(currentState);

    emit(stateUpdated());
}

void Ur5Robot::updateCurrentState()
{
    this->setPreviousState(mCurrentState);

    Ur5State currentState;

    currentState.timeSinceStart = mRTMonitor.getCurrentState().timeSinceStart;
    currentState.jointConfiguration = mRTMonitor.getCurrentState().jointConfiguration;
    currentState.jointVelocity = mRTMonitor.getCurrentState().jointVelocity;

    currentState.bMee = Ur5Kinematics::forward(currentState.jointConfiguration);
    currentState.cartAxis = Ur5Kinematics::T2transl(currentState.bMee);
    currentState.cartAngles = Ur5Kinematics::T2rangles(currentState.bMee);
    currentState.jacobian = Ur5Kinematics::jacobian2(currentState.jointConfiguration);

    prMt = prMb*currentState.bMee*eMt;

    emit transform("RobotTracker",currentState.bMee,currentState.timeSinceStart);

    this->setCurrentState(currentState);

    emit(stateUpdated());

    if(isMoveInProgress || isVelocityMoveInProgress)
    {
        if(this->isAtTargetState())
            this->nextMove();
    }
}

void Ur5Robot::nextMove()
{
    if(isMoveInProgress && !mMovementQueue.empty())
    {
        mMovementQueue.erase(mMovementQueue.begin());

        if(mMovementQueue.empty())
        {
            isMoveInProgress=false;
        }
        else
        {
            mTargetPose = mMovementQueue.front().target_xMe;
            this->move(mMovementQueue.front());
        }
    }
    else if(isVelocityMoveInProgress && !mMovementQueue.empty())
    {
        mMovementQueue.erase(mMovementQueue.begin());

        if(mMovementQueue.empty())
        {
            isVelocityMoveInProgress=false;
            this->stopMove("stopj", 0.3);
        }
        else
        {
            if(mMovementQueue.size()>1)
            {
                mTargetPose = mMovementQueue.front().target_xMe;

                Vector3D tangent = (Ur5Kinematics::T2transl(mTargetPose)-Ur5Kinematics::T2transl(mCurrentState.bMee*this->eMt))/1000;
                tangent = tangent/tangent.norm();
                Vector3D velocity =  tangent*mMovementQueue.front().velocity/1000;

                Vector3D rtangent = (Ur5Kinematics::T2rangles(mTargetPose)-Ur5Kinematics::T2rangles(mCurrentState.bMee*this->eMt));
                rtangent = rtangent/rtangent.norm();
                Vector3D rvelocity = rvelocity*mMovementQueue.front().velocity/1000;

                Eigen::RowVectorXd velocityEndEffector(6);
                velocityEndEffector << velocity(0),velocity(1),velocity(2), 0, 0, 0;

                mMovementQueue.front().targetJointVelocity = mCurrentState.jacobian.inverse()*velocityEndEffector.transpose();
                this->move(mMovementQueue.front());
            }
        }
    }
}

Ur5State Ur5Robot::getCurrentState()
{
    return (this->mCurrentState);
}

Ur5State Ur5Robot::getPreviousState()
{
    return (this->mPreviousState);
}

void Ur5Robot::setCurrentState(Ur5State currentState)
{
    this->mCurrentState=currentState;
}

void Ur5Robot::setPreviousState(Ur5State previousState)
{
    this->mPreviousState=previousState;
}

void Ur5Robot::setAddress(QString address)
{
    IPaddress = address;
}

QString Ur5Robot::getAddress()
{
    return IPaddress;
}

void Ur5Robot::connectToRobot(QString IPaddress)
{
    mRTMonitor.setAddress(IPaddress,rtPort);
    mSecMonitor.setAddress(IPaddress,secPort);

    connectToPort(rtPort);
    connectToPort(secPort);

    if(isConnectedToRobot())
        emit(connected());

}

void Ur5Robot::disconnectFromRobot()
{
    disconnectFromPort(rtPort);
    disconnectFromPort(secPort);

    if(!isConnectedToRobot())
        emit(disconnected());
}

void Ur5Robot::connectToPort(int port)
{
    if(port == 30003)
    {
        mRTMonitor.requestConnect();
    }
    else if(port == 30002)
    {
        mSecMonitor.requestConnect();
    }
}

void Ur5Robot::disconnectFromPort(int port)
{
    if(port == 30003)
    {
        mRTMonitor.requestDisconnect();
    }
    else if(port == 30002)
    {
        mSecMonitor.requestDisconnect();
    }
}

bool Ur5Robot::isConnectedToRobot()
{
    return (mRTMonitor.isConnectedToRobot() && mSecMonitor.isConnectedToRobot());
}

void Ur5Robot::shutdown()
{
    sendMessage(mMessageEncoder.powerdown());
    disconnectFromRobot();
    emit(shuttingdown());
}

void Ur5Robot::sendMessage(QString message)
{
    mSecMonitor.sendMessage(message);
}

void Ur5Robot::move(QString typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc, double vel, double t, double rad)
{
    mTargetState.jointConfiguration = targetConfiguration;

    if(typeOfMovement=="movej")
        sendMessage(mMessageEncoder.movej(targetConfiguration,acc,vel,t,rad));
    else if(typeOfMovement=="movejp")
        sendMessage(mMessageEncoder.movejp(targetConfiguration,acc, vel, t, rad));
    else if(typeOfMovement=="speedl")
        sendMessage(mMessageEncoder.speedl(targetConfiguration,acc,t));
    else if(typeOfMovement =="speedj")
        sendMessage(mMessageEncoder.speedj(targetConfiguration,acc,t));
    else if(typeOfMovement =="stopj")
        sendMessage(mMessageEncoder.stopj(acc));

}

void Ur5Robot::move(Ur5MovementInfo minfo)
{
    mTargetPose = minfo.target_xMe;

    if(minfo.typeOfMovement == Ur5MovementInfo::movej)
        sendMessage(mMessageEncoder.movej(minfo));
    else if(minfo.typeOfMovement == Ur5MovementInfo::speedj)
        sendMessage(mMessageEncoder.speedj(minfo));
    else if(minfo.typeOfMovement == Ur5MovementInfo::stopj)
        sendMessage(mMessageEncoder.stopj(minfo));

}

void Ur5Robot::clearMovementQueue()
{
    mMovementQueue.clear();
}

void Ur5Robot::stopMove(QString typeOfStop, double acc)
{
    if(typeOfStop=="stopl")
        sendMessage(mMessageEncoder.stopl(acc));
    else if(typeOfStop=="stopj")
        sendMessage(mMessageEncoder.stopj(acc));
}

void Ur5Robot::runMoveProgram(MovementQueue mq)
{
    if(mq.front().typeOfMovement == Ur5MovementInfo::movej)
    {
        mInitialState = this->getCurrentState();
        this->move(mq.front());
        isMoveInProgress=true;
    }
    else if(mq.front().typeOfMovement == Ur5MovementInfo::speedj)
    {
        mInitialState = this->getCurrentState();
        mq.front().typeOfMovement = Ur5MovementInfo::movej;
        mq.front().velocity = 30;
        this->move(mq.front());
        isVelocityMoveInProgress=true;
    }
    else
    {
        return;
    }

    mMovementQueue.clear();
    mMovementQueue = mq;
}

bool Ur5Robot::isAtTargetState()
{
    if((Ur5Kinematics::T2transl(mCurrentState.bMee*this->eMt)-Ur5Kinematics::T2transl(mTargetPose)).length()<mBlendRadius)
    {
        emit atTarget();
        return true;
    }
    return false;
}

void Ur5Robot::setBlendRadius(double blendRadius)
{
    mBlendRadius=blendRadius;
}

bool Ur5Robot::isValidWorkspace(Eigen::RowVectorXd jointPosition)
{
    return(abs(jointPosition.maxCoeff())<=2*3.15);
}

void Ur5Robot::set_eMt(Transform3D eMt)
{
    this->eMt = eMt;
    this->set_tcp(eMt);
    emit eMtChanged(eMt);
}

void Ur5Robot::set_prMb(Transform3D prMb)
{
    this->prMb = prMb;
    emit prMbChanged(prMb);
}

Transform3D Ur5Robot::get_eMt()
{
    return (this->eMt);
}

Transform3D Ur5Robot::get_prMb()
{
    return (this->prMb);
}


void Ur5Robot::set_tcp(Transform3D eMt)
{
    this->sendMessage(mMessageEncoder.set_tcp(eMt));
}

Vector3D Ur5Robot::getRobotVerifiedCartAngles(Vector3D cartAngles)
{
    if(cartAngles(0)>3.1402)
    {
        cartAngles(0) = cartAngles(0)-2*3.1402;
    }
    return cartAngles;
}

Transform3D Ur5Robot::get_prMt()
{
    return this->prMt;
}

} // cx
