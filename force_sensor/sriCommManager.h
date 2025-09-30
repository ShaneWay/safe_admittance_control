#ifndef SRI_COMM_MANAGER_H
#define SRI_COMM_MANAGER_H


#include "sriCommDefine.h"
#include "sriCommTCPClient.h"
#include "sriCommATParser.h"
#include "sriCommM8218Parser.h"
#include "KalmanFilter.h"

class CSRICommManager
{
public:
	CSRICommManager();
	~CSRICommManager();

	bool Init();
	bool Run();
	bool Stop();

	bool SendCommand(std::string command, std::string parames);
	bool SendGODCommand(std::string command, std::string parames);
	
	bool OnNetworkFailure(std::string infor);//ͨѶʧ��
	bool OnCommACK(std::string command);//ACKӦ�����ݴ���
	bool OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz);//GSD���ݴ���
	float force[6] = {0};
	Kalman myFilter;

	Eigen::Vector2d getBaseForce(Eigen::Vector3d eulerAngle);
	double getJointForce();

private:
	CSRICommTCPClient mTCPClient;
	CSRICommATParser mATParser;//ATָ�������
	CSRICommM8218Parser mM8218Parser;//GSD���ݽ�����

	bool mIsGetACK;
	std::string mCommandACK;
	std::string mParamesACK;

};

#endif

