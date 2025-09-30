#include <Eigen/Dense>
#include "sriCommManager.h"




CSRICommManager::CSRICommManager()
{
}


CSRICommManager::~CSRICommManager()
{
}
//��ʼ��
bool CSRICommManager::Init()
{
	mTCPClient.OpenTCP("192.168.1.108", 4008);
	mTCPClient.AddCommParser(&mATParser);
	mTCPClient.AddCommParser(&mM8218Parser);

	//bind Network communication failure processing function
	//������ͨѶʧ�ܴ������� 
	SRICommNetworkFailureCallbackFunction networkFailureCallback = std::bind(&CSRICommManager::OnNetworkFailure, this, std::placeholders::_1);
	mTCPClient.SetNetworkFailureCallbackFunction(networkFailureCallback);

	//bind ACK command Processing function
	//��ACKָ�������   	
	SRICommATCallbackFunction atCallback = std::bind(&CSRICommManager::OnCommACK, this, std::placeholders::_1);
	mATParser.SetATCallbackFunction(atCallback);

	//bind M8128 Data processing function
	//��M8128���ݴ�������
	SRICommM8218CallbackFunction m8218Callback = std::bind(&CSRICommManager::OnCommM8218, this, 
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
		std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
	mM8218Parser.SetM8218CallbackFunction(m8218Callback);

	myFilter.setParameters(0.125,32,1023);

	return true;
}
//����
bool CSRICommManager::Run()
{
	//creat a TCP client connection
	//����TCP����
	if (mTCPClient.Connect() == false)
	{
		return false;
	}
	// if (SendCommand("EIP", "192.168.1.108") == false)
	// {
	// 	return false;
	// }
	//Send command to M8128 to set data upload format
	//��m8128����ָ����������ϴ���ʽ
	if (SendCommand("SGDM", "(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)") == false)
	{
		return false;
	}
	//Send command to M8128 to set data check mode
	//��m8128����ָ���������У��ģʽ
	if (SendCommand("DCKMD", "SUM") == false)
	{
		return false;
	}
	//Send command to M8128 Start to upload sensor data continuously��
	//��m8128����ָ���ʼ�����ϴ�����������
	if (SendCommand("SMPF", "2000") == false)
	{
		//GSD has no ACK
		//return false;
	}
	// if (SendCommand("GSD", "") == false)
	// {
	// 	//GSD has no ACK
	// 	//return false;
	// }
	return true;
}

bool CSRICommManager::Stop()
{
	mTCPClient.CloseTCP();
	return true;
}

//ͨѶ�쳣����
bool CSRICommManager::OnNetworkFailure(std::string infor)
{
	printf("OnNetworkFailure = %s\n", infor.data());
	return true;
}


//send command 
//��������
bool CSRICommManager::SendCommand(std::string command, std::string parames)
{
	mIsGetACK = false;
	mCommandACK = "";
	mParamesACK = "";

	//Combination command
	//���ATָ��
	std::string atCommand = "AT+" + command + "=" + parames + "\r\n";
	if (mTCPClient.OnSendData((BYTE*)atCommand.data(), (int)atCommand.length()) == false)
	{
		return false;
	}
	//wait ACK
#ifdef  IS_WINDOWS_OS
	std::clock_t start = clock();
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		std::clock_t end = clock();
		long span = end - start;
		if (span >= 10000)//10s
		{
			return false;
		}
	}
#else
	timeval start, end;
	gettimeofday(&start, NULL);
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		gettimeofday(&end, NULL);
		long span = 1000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000;
		if (span >= 10000)//10s
		{
			return false;
		}
	}
#endif
	
	if (mCommandACK != command)
	{
		//ACK command error
		return false;
	}
	
	printf("ACK+%s=%s", mCommandACK.data(), mParamesACK.data());
	//
	return true;
}

bool CSRICommManager::SendGODCommand(std::string command, std::string parames)
{
	mIsGetACK = false;
	mCommandACK = "";
	mParamesACK = "";

	//Combination command
	//���ATָ��
	std::string atCommand = "AT+" + command + "=" + parames + "\r\n";
	if (mTCPClient.OnSendData((BYTE*)atCommand.data(), (int)atCommand.length()) == false)
	{
		return false;
	}
	// printf("ACK+%s=%s", mCommandACK.data(), mParamesACK.data());
}

//ACK command processing
//ACKӦ����
bool CSRICommManager::OnCommACK(std::string command)
{
	int index = (int)command.find('=');
	if (index == -1)
	{
		mCommandACK = command;
		mParamesACK = "";
	}
	else
	{
		mCommandACK = command.substr(0, index);
		mParamesACK = command.substr(index+1);
	}
	mCommandACK = mCommandACK.substr(4);	
	
	mIsGetACK = true;//Ӧ����ȷ

	return true;
}
//M8128 data
//M8128����
bool CSRICommManager::OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz)
{
	
	Eigen::VectorXd force_sensor(6);
	Eigen::VectorXd force_end(6);
	Eigen::MatrixXd adj(6,6);
	adj.setZero();

	// force_sensor[0] = myFilter.getFilteredValue(fx);
	// force_sensor[1] = myFilter.getFilteredValue(fy);
	// force_sensor[2] = myFilter.getFilteredValue(fz);
	// force_sensor[3] = myFilter.getFilteredValue(mx);
	// force_sensor[4] = myFilter.getFilteredValue(my);
	// force_sensor[5] = myFilter.getFilteredValue(mz);

	force_sensor[0] = fx;
	force_sensor[1] = fy;
	force_sensor[2] = fz;
	force_sensor[3] = mx;
	force_sensor[4] = my;
	force_sensor[5] = mz;

	Eigen::Matrix3d R;
	Eigen::Matrix3d P;
	Eigen::Matrix3d P_R;
	R << 1.,0.,0.,
		 0.,1.,0.,
		 0.,0.,1.;
	P << 0.,-0.1095,0.,
		 0.1095, 0., 0.,
		 0.,0.,0.;
	P_R = P * R;
	adj.block(0,0,3,3) = R;
	// adj.block(0,3,3,3) = Eigen::Vector3d::Zero();
	adj.block(3,0,3,3) = P * R;
	adj.block(3,3,3,3) = R;
	
	force_end = adj * force_sensor;
	for (int i = 0; i < 6; i++)
	{
		force[i] = force_end[i];
	}
	// for (int i = 0; i < 6; i++)
	// {
	// 	std::cout << force[i] << " ";
	// }
	// std::cout << std::endl;
	// printf("M8218 = %f, %f, %f,   %f, %f, %f\n", fx, fy, fz, mx, my, mz); 109.5

	return true;
}

Eigen::Vector2d CSRICommManager::getBaseForce(Eigen::Vector3d eulerAngle)
{
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));

	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=yawAngle*pitchAngle*rollAngle;
	Eigen::Vector3d efffect_force(force[0], force[1], force[2]);
	Eigen::Vector3d Base_force = rotation_matrix.inverse() * efffect_force;
	Eigen::Vector2d output_force(Base_force[0], Base_force[1]);
	return output_force;
}

double CSRICommManager::getJointForce()
{
	double length = 0.5;
	double joint_force = length * force[0];
	return joint_force;
}