// DRFTWin32.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

#ifdef __XENO__
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#else
// #include "stdafx.h"
// #include <Windows.h>
// #include <process.h>
// #include <conio.h>
#endif // __XENO__
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <pthread.h>
#include <mutex>
#include <cmath>
#include <vector>
#include <fstream>
#include <sched.h>
#include <csignal>
#include <regex>
#include "../../include/DRFLEx.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctime>
#include <queue>
#include <condition_variable>
#include <atomic>


using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>
// #include "../../include/matplotlibcpp.h"
// namespace plt = matplotlibcpp;


CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;
string strDrl =
    "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";

bool bAlterFlag = FALSE;
float k_ratio = 1.5;
int sock = socket(AF_INET, SOCK_STREAM, 0);
const char* server_ip = "192.168.137.30";


int linux_kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;

	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );


	ch = getchar();

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);          
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);         
    newattr.c_cc[VMIN] = 1;                      
    newattr.c_cc[VTIME] = 0;                     
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  
    c = getchar();                               
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  
    return c;
}

void OnTpInitializingCompleted() {
  // Tp 
  g_TpInitailizingComplted = TRUE;
  Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted() {
  // 50msec
  cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE) {
  assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
  // 50msec
  // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
  cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData) {
  // 50msec

  return;
  cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
       << pData->_tCtrl._tTask._fActualPos[0][1]
       << pData->_tCtrl._tTask._fActualPos[0][2]
       << pData->_tCtrl._tTask._fActualPos[0][3]
       << pData->_tCtrl._tTask._fActualPos[0][4]
       << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
  return;
  cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
       << pData->_tCtrl._tWorld._fTargetPos[1]
       << pData->_tCtrl._tWorld._fTargetPos[2]
       << pData->_tCtrl._tWorld._fTargetPos[3]
       << pData->_tCtrl._tWorld._fTargetPos[4]
       << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData) {
  return;
  cout << "# monitoring ctrl 0 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData) {
  return;
  cout << "# monitoring ctrl 1 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tOutput._iTargetDO[i] << endl;
  }
}

void OnMonitoringStateCB(const ROBOT_STATE eState) {
  // 50msec
  switch ((unsigned char)eState) {
#if 0  // TP �ʱ�ȭ�� ����ϴ� ���������� API ���������� ������� ����.(TP����
       // �ܵ� ����� ���, ���)
    case STATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
        break;
    case STATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
        break;
#endif
    case STATE_EMERGENCY_STOP:
      // pop/home/ms/git_repo/DRFL-External/DRFL/DRFL-LX/Release/lib64/libDRFL.aup
      break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
      break;
    case STATE_SAFE_STOP:
      if (g_bHasControlAuthority) {
        Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
        Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
      }
      break;
    case STATE_SAFE_OFF:
      // cout << "STATE_SAFE_OFF1" << endl;c++
      if (g_bHasControlAuthority) {
        // cout << "STATE_SAFE_OFF2" << endl;
        Drfl.SetRobotControl(CONTROL_SERVO_ON);
        Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

      }
      break;
    case STATE_SAFE_STOP2:
      if (g_bHasControlAuthority)
        Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
      break;
    case STATE_SAFE_OFF2:
      if (g_bHasControlAuthority) {
        Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
      }
      break;
    case STATE_RECOVERY:
      // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
      break;
    default:
      break;
  }
  return;
  cout << "current state: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
  // 50msec

  switch (eTrasnsitControl) {
    case MONITORING_ACCESS_CONTROL_REQUEST:
      assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
      // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
      break;
    case MONITORING_ACCESS_CONTROL_GRANT:
      g_bHasControlAuthority = TRUE;
      // cout << "GRANT1" << endl;
      // cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
      OnMonitoringStateCB(Drfl.GetRobotState());
      // cout << "GRANT2" << endl;
      break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
      g_bHasControlAuthority = FALSE;
      if (g_TpInitailizingComplted) {
        // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
      }
      break;
    default:
      break;
  }
}

void OnLogAlarm(LPLOG_ALARM tLog) {
  g_mStat = true;
  cout << "Alarm Info: "
       << "group(" << (unsigned int)tLog->_iGroup << "), index("
       << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
       << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup) {
  cout << "Popup Message: " << tPopup->_szText << endl;
  cout << "Message Level: " << tPopup->_iLevel << endl;
  cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char* strLog) { cout << "Log Message: " << strLog << endl; }

void OnTpProgress(LPMESSAGE_PROGRESS tProgress) {
  cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
  cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput) {
  cout << "User Input : " << tInput->_szText << endl;
  cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
    // static int td = 0;
    // if (td++ == 1000) {
    // 	td = 0;
    
    // printf("timestamp : %.3f\n", tData->time_stamp);
		// printf("q = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
		// 		tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2],
		// 		tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
    // printf("q_d = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
		// 		tData->target_joint_position[0], tData->target_joint_position[1], tData->target_joint_position[2],
		// 		tData->target_joint_position[3], tData->target_joint_position[4], tData->target_joint_position[5]);
		// printf("qdot = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
		// 		tData->actual_joint_velocity[0], tData->actual_joint_velocity[1], tData->actual_joint_velocity[2],
		// 		tData->actual_joint_velocity[3], tData->actual_joint_velocity[4], tData->actual_joint_velocity[5]);
		
    // printf("trq_g = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
		// 		tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2],
		// 		tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
    // }
    return;
}


uint32_t ThreadFunc(void* arg) {
	printf("start ThreadFunc\n");

	while (true) {
		if(linux_kbhit()){
			char ch = getch();
			switch (ch) {
				case 's': {
					printf("Stop!\n");
					g_Stop = true;
					Drfl.MoveStop(STOP_TYPE_SLOW);
				} break;
				case 'p': {
					printf("Pause!\n");
					Drfl.MovePause();
				} break;
				case 'r': {
					printf("Resume!\n");
					Drfl.MoveResume();
				} break;
			}
		}

		//Sleep(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "exit ThreadFunc" << std::endl;

	return 0;
}

void OnDisConnected() {
  while (!Drfl.open_connection("192.168.137.100")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

struct PlanParam
{
	float time;

	float ps[6];
	float vs[6];
	float as[6];
	float pf[6];
	float vf[6];
	float af[6];

	float A0[6];
	float A1[6];
	float A2[6];
	float A3[6];
	float A4[6];
	float A5[6];
};

struct TraParam
{
	float time;

	float pos[6];
	float vel[6];
	float acc[6];
};

void TrajectoryPlan(PlanParam* plan)
{
    float ps[6],vs[6],as[6];
    float pf[6],vf[6],af[6];
    float tf;

	tf = plan->time;

    for(int i=0; i<6; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for(int i=0; i<6; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i]/2;
        plan->A3[i] = (20*pf[i]-20*ps[i]-(8*vf[i]+12*vs[i])*tf-(3*as[i]-af[i])*tf*tf)/(2*tf*tf*tf);
        plan->A4[i] = (30*ps[i]-30*pf[i]+(14*vf[i]+16*vs[i])*tf+(3*as[i]-2*af[i])*tf*tf)/(2*tf*tf*tf*tf);
        plan->A5[i] = (12*pf[i]-12*ps[i]-(6*vf[i]+6*vs[i])*tf-(as[i]-af[i])*tf*tf)/(2*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[6],A1[6],A2[6],A3[6],A4[6],A5[6];
	double t = tra->time;

    for(int i=0; i<6; i++)
    {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for(int i=0; i<6; i++)
    {
        tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
        tra->vel[i] = A1[i] + 2*A2[i]*t + 3*A3[i]*t*t + 4*A4[i]*t*t*t + 5*A5[i]*t*t*t*t;
        tra->acc[i] = 2*A2[i] + 6*A3[i]*t + 12*A4[i]*t*t + 20*A5[i]*t*t*t;
    }
}

struct LogData {
    std::chrono::steady_clock::time_point timestamp;
    std::string message;
};

std::mutex log_mutex;
std::ofstream logfile("Doosanrobotics_realtime_api_logs_011425.txt", std::ios::app);

void log_timestamp(const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex);
    auto now = std::chrono::steady_clock::now();
    auto time_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    logfile << time_since_epoch.count() << " us: " << message << std::endl;
    logfile.flush(); // Ensure data is written to file immediately
}

void handle_signal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::lock_guard<std::mutex> lock(log_mutex);
        logfile << "Program interrupted by signal " << signal << std::endl;
        logfile.close();
        exit(0);
    }
}

void MoveJ_ServoJ(float target_posj[6], float target_time, float ratio)
{
	const float None=-10000;
	float vel[6] = {100, 100, 100, 100, 100, 100};
	float acc[6] = {120, 120, 120, 120, 120, 120};

	Drfl.set_velj_rt(vel);
	Drfl.set_accj_rt(acc);
	Drfl.set_velx_rt(250);
	Drfl.set_accx_rt(1000);

	const float st=0.02; // sampling time
//	const float ratio=1;

	float count=0;
	float time=0;

	TraParam tra;

	// Plan1
	PlanParam plan1;
	plan1.time=target_time;
	memcpy(plan1.ps, Drfl.read_data_rt()->target_joint_position, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.pf, target_posj, NUMBER_OF_JOINT*sizeof(float));

	plan1.vs[0]=0; plan1.vs[1]=0; plan1.vs[2]=0; plan1.vs[3]=0; plan1.vs[4]=0; plan1.vs[5]=0;
	plan1.vf[0]=0; plan1.vf[1]=0; plan1.vf[2]=0; plan1.vf[3]=0; plan1.vf[4]=0; plan1.vf[5]=0;
	plan1.as[0]=0; plan1.as[1]=0; plan1.as[2]=0; plan1.as[3]=0; plan1.as[4]=0; plan1.as[5]=0;
	plan1.af[0]=0; plan1.af[1]=0; plan1.af[2]=0; plan1.af[3]=0; plan1.af[4]=0; plan1.af[5]=0;
	TrajectoryPlan(&plan1);

	printf("Start servoj motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			plan1.ps[0],plan1.ps[1],plan1.ps[2],
			plan1.ps[3],plan1.ps[4],plan1.ps[5]);

	while(1)
	{
		time=(++count)*st;
		tra.time=time;

		TrajectoryGenerator(&plan1,&tra);
			// printf("tra.pos, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			// 		tra.pos[0],tra.pos[1],tra.pos[2],
			// 		tra.pos[3],tra.pos[4],tra.pos[5]);
//				for(int i=0; i<6; i++)
//				{
//					tra.vel[i]=None;
//					tra.acc[i]=None;
//				}

		Drfl.servoj_rt(tra.pos, tra.vel, tra.acc, st*ratio);

		if(time > plan1.time)
		{
			time=0;
//			Drfl.stop(STOP_TYPE_SLOW);
			// printf("Finish movej with servoj, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			// 		target_posj[0],target_posj[1],target_posj[2],
			// 		target_posj[3],target_posj[4],target_posj[5]);


			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		// rt_task_wait_period(NULL);
	}
}
vector<float> path_command(const string& line) {
  vector<float> values;
  regex pattern(R"(servol_rt\(\[(.*?)\])");
  smatch match;
  if (regex_search(line, match, pattern)) {
    string strValues = match.str();
    stringstream ss(strValues);
    string temp;
    while (getline(ss, temp, ' ')) {
      try {
        values.push_back(stod(temp));
      } catch (const std::exception& e) {
        cout << "Error parsing value: " << temp << endl;
      }
    }
  }
  return values;
}
void* realtime_task(void* arg) {
    float home[6] = {0, 0, 0, 0, 0, 0};
    Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl.movej(home, 60, 60);
    float posj_s[6] = {0, 0, 0, 0, 0, 0};
    float posj_f[6] = {30, 30, 60, -30, -90, 30};
	  const float None=-10000;

    float target_time = 8.0;

    float vel[6] = {100, 100, 100, 100, 100, 100};
    float acc[6] = {120, 120, 120, 120, 120, 120};
    float vs[6] = {0, 0, 0, 0, 0, 0};
    float vf[6] = {0, 0, 0, 0, 0, 0};
    float as[6] = {0, 0, 0, 0, 0, 0};
    float af[6] = {0, 0, 0, 0, 0, 0};
    Drfl.set_velj_rt(vel);
    Drfl.set_accj_rt(acc);
    float nodata[6] = {-10000,-10000,-10000,-10000,-10000,-10000};
    int count = 0;
    float time = 0;
    float ratio = 1.0;

    TraParam tra1;
    TraParam tra2;

    // Plan1
    PlanParam plan1;
    PlanParam plan2;
    plan1.time = target_time;
    plan2.time = target_time;
    memcpy(plan1.ps, posj_s, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.pf, posj_f, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.ps, posj_f, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.pf, posj_s, NUMBER_OF_JOINT * sizeof(float));

    memcpy(plan1.vs, vs, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.vf, vf, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.as, as, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.af, af, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.vs, vs, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.vf, vf, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.as, as, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.af, af, NUMBER_OF_JOINT * sizeof(float));
    TrajectoryPlan(&plan1);
    TrajectoryPlan(&plan2);

    const int loop_period_ms = 1;
    float dt = 0.001*loop_period_ms; // sampling time
    std::chrono::milliseconds loop_period(loop_period_ms);

    // printf("Start servoj test\n");
    log_timestamp("Start servoj test");
    Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

    int repeat = 100000;
    ratio = 15;
    bool flag = true;
    double tt = dt*ratio;
    for (int i = 0; i < repeat; i++)
    {
        flag = true;
        count = 0;
        time = 0;
        if (i%2 == 0) {
            // std::cout << "Start ServoJ Motion";
            log_timestamp("Start ServoJ Motion");
            std::cout << "loop " << i << ") ps: ";
            for (size_t k=0; k < 6; k++) {
                std::cout << plan1.ps[k] << ", ";
            }
            std::cout << std::endl;
            while (flag)
            {
              auto loop_start_time = std::chrono::steady_clock::now();

              time = (++count) * dt;
              tra1.time = time;
              // std::cout << "time: " << time << std::endl;

              TrajectoryGenerator(&plan1, &tra1);
              // for(int i=0; i<6; i++)
              // {
              //   tra1.vel[i]=None;
              //   tra1.acc[i]=None;
              // }

              // Drfl.servoj_rt(tra1.pos, tra1.vel, tra1.acc, dt * ratio);
              Drfl.servoj_rt(tra1.pos, tra1.vel, tra1.acc, tt);
              // Drfl.servoj_rt(tra1.pos, nodata, nodata, tt);
              // Drfl.speedj_rt(tra1.vel, tra1.acc, tt);
              if (time > plan1.time)
              {
                  // time = 0;
                  flag = false;
                  // std::cout << "Finish ServoJ Motion";
                  log_timestamp("Finish ServoJ Motion");  

                  std::cout << "loop " << i << ") pf: ";
                  for (size_t k=0; k < 6; k++) {
                      std::cout << plan1.pf[k] << ", ";
                  }
                  std::cout << std::endl;
                  break;
              }
              auto loop_end_time = std::chrono::steady_clock::now();
              auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
              if (elapsed_time < loop_period)
              {
                  std::this_thread::sleep_for(loop_period - elapsed_time);
              }
              else
              {
                  log_timestamp("Loop time is over");
              }
            }
        } else {
            log_timestamp("Start ServoJ Motion");

            std::cout << "loop " << i << ") ps: ";
            for (size_t k=0; k < 6; k++) {
                std::cout << plan2.ps[k] << ", ";
            }
            std::cout << std::endl;
            while (flag)
            {
              auto loop_start_time = std::chrono::steady_clock::now();

              time = (++count) * dt;
              tra2.time = time;

              TrajectoryGenerator(&plan2, &tra2);
              for(int i=0; i<6; i++)
              {
                tra2.vel[i]=None;
                tra2.acc[i]=None;
              }

              // Drfl.servoj_rt(tra2.pos, tra2.vel, tra2.acc, dt * ratio);
              Drfl.servoj_rt(tra2.pos, tra2.vel, tra2.acc, tt);
              // Drfl.servoj_rt(tra2.pos, nodata, nodata, tt);
              // Drfl.speedj_rt(tra2.vel, tra2.acc, tt);
              auto loop_end_time = std::chrono::steady_clock::now();
              
              auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
              if (time > plan1.time)
              {
                  // time = 0;
                  flag = false;
                  log_timestamp("Finish ServoJ Motion");
                  std::cout << "loop " << i << ") pf: ";
                  for (size_t k=0; k < 6; k++) {
                      std::cout << plan2.pf[k] << ", ";
                  }
                  std::cout << std::endl;
              }
              if (elapsed_time < loop_period)
              {
                  std::this_thread::sleep_for(loop_period - elapsed_time);
              }
              else
              {
                  log_timestamp("Loop time is over");
              }
            }
        }
      }
      log_timestamp("Finish servoj test");
    return nullptr;
}

bool toggle = true; 
void TrajectoryGenerator(float* q_d, float* q_dot_d) {
        float target_position1[NUMBER_OF_JOINT] = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
        float target_position2[NUMBER_OF_JOINT] = {0.0, 0.0, -90.0, 0.0, 0.0, 0.0};
        float* target_position = toggle ? target_position1 : target_position2;
        float target_velocity[NUMBER_OF_JOINT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 


        for (int i = 0; i < NUMBER_OF_JOINT; i++) {
            q_d[i] = target_position[i]; // 목표 위치
            q_dot_d[i] = target_velocity[i]; // 목표 속도
        }
}

// constexpr int DOF = 6;
// constexpr float TOTAL_DURATION = 0.02f; // 20ms
// constexpr int NUM_STEPS = 9;
// constexpr float DT = TOTAL_DURATION / NUM_STEPS; // 0.002 = 2ms


constexpr int DOF = 6;
constexpr int BUFFER_SIZE = DOF * 3 * sizeof(float);
constexpr float TOTAL_DURATION = 0.02f;  // 20ms
constexpr int NUM_STEPS = 15;
constexpr float DT = TOTAL_DURATION / NUM_STEPS;

constexpr int PRIORITY_RECV = 80;
constexpr int PRIORITY_CTRL = 90;
int PORT = 5005;
std::queue<std::vector<float>> recv_queue;
std::mutex queue_mutex;
std::condition_variable data_cv;
std::atomic<bool> running{true};

// int sock;

struct JointState {
    float pos[DOF];
    float vel[DOF];
    float acc[DOF];
};

// 큐빅 보간: pos(t), vel(t), acc(t) 모두 계산
JointState cubic_interpolate(float t,
                             const float p0[DOF],
                             const float v0[DOF],
                             const float a0[DOF]) {
    JointState result;
    for (int i = 0; i < DOF; ++i) {
        result.pos[i] = p0[i] + v0[i] * t + 0.5f * a0[i] * t * t;
        result.vel[i] = v0[i] + a0[i] * t;
        result.acc[i] = a0[i]; // 가속도는 상수로 가정
    }
    return result;
}

void* realtime_task_isaac_sim(void* arg)
{
        using namespace std::chrono;
        float vel[6] = {100, 100, 100, 100, 100, 100};
        float acc[6] = {100, 100, 100, 100, 100, 100};
        float vel2[6] = {-10000, -10000,-10000,-10000,-10000,-10000};
        float acc2[6] = {-10000, -10000,-10000,-10000,-10000,-10000};
        Drfl.set_velj_rt(vel);
        Drfl.set_accj_rt(acc);
        const int port = 5005;
        const int dof = 6;
        const int total_floats = dof *3;
        const int buffer_size = 3 * dof * sizeof(float); // 6 * 4 = 24 bytes
        float dt = 0.1;
        std::cout << "start realtime task.\n" << std::endl;

        if (sock < 0) {
            std::cerr << "Socket creation error\n";
            return (void*) -1;
        }
        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        const int retry_delay_ms = 1000;  // 재시도 간격 (1초)
        while (true) {
            if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address / Address not supported\n";
                return (void*) -1;
            }
            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Waiting for server... retrying in " << retry_delay_ms << " ms\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                continue;
            }
            std::cout << "Connected to server.\n";
            break;  // 연결 성공 시 loop 탈출
        }
        int receive_count = 0;  // 데이터 수신 횟수 카운터

        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            std::time_t startnow_c = std::chrono::system_clock::to_time_t(start);
            std::tm* startmslocal_tm = std::localtime(&startnow_c);
            auto startms = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;

            // 출력
            std::cout << "STD 현재 시간: "
                      << std::put_time(startmslocal_tm, "%Y-%m-%d %H:%M:%S") << '.'
                      << std::setw(3) << std::setfill('0') << startms.count()
                      << std::endl;
            char buffer[buffer_size];
            int total_received = 0;

            while (total_received < buffer_size) {
                int bytes_received = recv(sock, buffer + total_received, buffer_size - total_received, 0);
                if (bytes_received <= 0) {
                    std::cerr << "Connection closed or error occurred\n";
                    close(sock);
                    return 0;
                }
                total_received += bytes_received;
            }
            auto mid = std::chrono::high_resolution_clock::now();
            std::time_t mnow_c = std::chrono::system_clock::to_time_t(mid);
            std::tm* mlocal_tm = std::localtime(&mnow_c);
            auto midms = std::chrono::duration_cast<std::chrono::milliseconds>(mid.time_since_epoch()) % 1000;
            // 출력
            std::cout << "Mid 현재 시간: "
                      << std::put_time(mlocal_tm, "%Y-%m-%d %H:%M:%S") << '.'
                      << std::setw(3) << std::setfill('0') << midms.count()
                      << std::endl;
            float data[total_floats];
            std::memcpy(data, buffer, buffer_size);

            // 받은 데이터를 float 배열로 변환
            float joints[dof];
            float vel_rt[dof];
            float acc_rt[dof];

            for (int i = 0; i < dof; ++i) {
                joints[i] = data[i] * (180.0f / static_cast<float>(M_PI));  // rad → deg
                vel_rt[i] = std::abs(data[i + dof] * (180.0f / static_cast<float>(M_PI)));
                acc_rt[i] = std::abs(data[i + dof*2] * (180.0f / static_cast<float>(M_PI)));
            }
            for (int step = 1; step <= NUM_STEPS; ++step) {
              float t = DT * step;
              JointState state = cubic_interpolate(t, joints, vel_rt, acc_rt);
              Drfl.servoj_rt(state.pos, state.vel, state.acc, dt);  
              std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            // joints[5] = 0;
            // vel_rt[5] = 0;
            // std::cout << "Position: ";
            // for (int i = 0; i < 6; ++i)
            //     std::cout << joints[i] << " ";
            // std::cout << std::endl;

            // std::cout << "Velocity: ";
            // for (int i = 0; i < 6; ++i)
            // {
            //     std::cout << vel_rt[i] << " ";
            // }
            // std::cout << std::endl;
            // std::cout << "Acceleration: ";
            // for (int i = 0; i < 6; ++i)
            // {
            //     std::cout << acc_rt[i] << " ";
            // }
            // std::cout << std::endl;

            // // Drfl.servoj_rt(joints, vel2, acc2, dt);  
            // Drfl.servoj_rt(joints, vel_rt, acc_rt, dt);  

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::time_t now_c = std::chrono::system_clock::to_time_t(end);
            std::tm* local_tm = std::localtime(&now_c);

            // 밀리초 계산: now.time_since_epoch() 전체 duration에서 초 단위만 빼고 밀리초만 추출
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end.time_since_epoch()) % 1000;

            // 출력
            std::cout << "END 현재 시간: "
                      << std::put_time(local_tm, "%Y-%m-%d %H:%M:%S") << '.'
                      << std::setw(3) << std::setfill('0') << ms.count()
                      << std::endl;

            //this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        close(sock);
}

// ⏬ 수신 쓰레드
void* recv_thread_func(void*) {
    while (running) {
        char buffer[BUFFER_SIZE];
        int total_received = 0;

        while (total_received < BUFFER_SIZE) {
            int bytes = recv(sock, buffer + total_received, BUFFER_SIZE - total_received, 0);
            if (bytes <= 0) {
                std::cerr << "[recv_thread] Connection closed or error\n";
                running = false;
                return nullptr;
            }
            total_received += bytes;
        }

        std::vector<float> data(DOF * 3);
        std::memcpy(data.data(), buffer, BUFFER_SIZE);

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            recv_queue.push(data);
        }
        data_cv.notify_one();
    }
    return nullptr;
}
// std::ofstream interval_log("servo_intervals.csv", std::ios::app);  // append 모드
// std::ofstream interval_log("servo_intervals.csv", std::ios::trunc);
constexpr auto interval = std::chrono::milliseconds(2);  // 2ms
int servoj_rt_cmd_times = 0;
// ⏫ 제어 쓰레드
void* control_thread_func(void*) {
    
    while (running) {
        float local_num_steps = NUM_STEPS;
        std::vector<float> data;
        {
            if ( recv_queue.empty()) continue;
            size_t qsize = recv_queue.size();
            // std::cout << "qsize : " << qsize << std::endl;
            
            // 단계별 NUM_STEPS 조절
            if (qsize >= 16)
                local_num_steps = 10;
            else if (qsize >= 8)
                local_num_steps = 12;
            else if (qsize >= 4)
                local_num_steps = 14;
            else if (qsize >= 2)
                local_num_steps = 15;
            else
                local_num_steps = 15;

            data = recv_queue.front();
            recv_queue.pop();
        }

        float joints[DOF], vel_rt[DOF], acc_rt[DOF];
        for (int i = 0; i < DOF; ++i) {
            joints[i] = data[i] * (180.0f / static_cast<float>(M_PI));
            vel_rt[i] = std::abs(data[i + DOF] * (180.0f / static_cast<float>(M_PI)));
            acc_rt[i] = std::abs(data[i + 2 * DOF] * (180.0f / static_cast<float>(M_PI)));
        }

        auto prev_time = std::chrono::high_resolution_clock::now();
        auto predict_time = std::chrono::high_resolution_clock::now();

        for (int step = 1; step <= local_num_steps; ++step) {

            auto loop_start = std::chrono::high_resolution_clock::now();
            float t = DT * step;
            JointState state = cubic_interpolate(t, joints, vel_rt, acc_rt);
            
            auto sleep_duration = predict_time - std::chrono::high_resolution_clock::now();
            float sleep_duration_ms = std::chrono::duration<float, std::milli>(sleep_duration).count();

            if (sleep_duration.count() > 0)
                std::this_thread::sleep_for(sleep_duration);

            auto loop_end = std::chrono::high_resolution_clock::now();
            float loop_interval_ms = std::chrono::duration<float, std::milli>(loop_end - loop_start).count();
            if (step ==1){
                std::ofstream interval_log("/dev/shm/servo_intervals.csv", std::ios::trunc);
                interval_log << servoj_rt_cmd_times << "," << (2.0f-sleep_duration_ms) << std::endl;
                // std::cout << "sleep_duration_ms : " << (2.0f-sleep_duration_ms) << std::endl;

                for (int i = 0; i < DOF; ++i)
                    interval_log << "," << state.pos[i] << "," << state.vel[i] << "," << state.acc[i];
                interval_log << std::endl;
            }
            else{
              float sleep_time = 2.0f - loop_interval_ms;
            if (sleep_time > 0)
              std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sleep_time * 1000)));
              auto loop_final = std::chrono::high_resolution_clock::now();
              float loop_final_interval_ms = std::chrono::duration<float, std::milli>(loop_final - loop_start).count();
              if (loop_final_interval_ms > 2.1f){
                // std::cout << "step : " << step << std::endl;
                // std::cout << "sleep_duration_ms : " << sleep_duration_ms << std::endl;
                // std::cout << "sleep_time : " << sleep_time << std::endl;
              }
              else {
                std::ofstream interval_log("/dev/shm/servo_intervals.csv", std::ios::trunc);
                interval_log << servoj_rt_cmd_times << "," << loop_final_interval_ms;
                // std::cout << "loop_interval_ms : " << loop_final_interval_ms << std::endl;

                for (int i = 0; i < DOF; ++i)
                    interval_log << "," << state.pos[i] << "," << state.vel[i] << "," << state.acc[i];
                interval_log << std::endl;
                // interval_log << servoj_rt_cmd_times << "," << loop_final_interval_ms << std::endl;
              }
            }

            Drfl.servoj_rt(state.pos, state.vel, state.acc, 0.40);

            predict_time += interval;
        }

    }
    return nullptr;
}
void* control_thread_func_speedj(void*) {
    
    while (running) {
        float local_num_steps = NUM_STEPS;
        std::vector<float> data;
        {
            if ( recv_queue.empty()) continue;
            size_t qsize = recv_queue.size();
            std::cout << "qsize : " << qsize << std::endl;
            
            // 단계별 NUM_STEPS 조절
            if (qsize >= 16)
                local_num_steps = 10;
            else if (qsize >= 8)
                local_num_steps = 11;
            else if (qsize >= 4)
                local_num_steps = 14;
            else if (qsize >= 2)
                local_num_steps = 15;
            else
                local_num_steps = 15;

            data = recv_queue.front();
            recv_queue.pop();
        }

        float joints[DOF], vel_rt[DOF], acc_rt[DOF];
        for (int i = 0; i < DOF; ++i) {
            joints[i] = data[i] * (180.0f / static_cast<float>(M_PI));
            vel_rt[i] = std::abs(data[i + DOF] * (180.0f / static_cast<float>(M_PI)));
            acc_rt[i] = std::abs(data[i + 2 * DOF] * (180.0f / static_cast<float>(M_PI)));
        }

        auto prev_time = std::chrono::high_resolution_clock::now();
        auto predict_time = std::chrono::high_resolution_clock::now();

        
      for (int step = 1; step <= local_num_steps; ++step) {
          // servoj_rt_cmd_times += 1;
          auto loop_start = std::chrono::high_resolution_clock::now();

          float t = DT * step;
          JointState state = cubic_interpolate(t, joints, vel_rt, acc_rt);

          auto sleep_duration = predict_time - std::chrono::high_resolution_clock::now();
          float sleep_duration_ms = std::chrono::duration<float, std::milli>(sleep_duration).count();

          if (sleep_duration.count() > 0)
              std::this_thread::sleep_for(sleep_duration);

          auto loop_end = std::chrono::high_resolution_clock::now();
          float loop_interval_ms = std::chrono::duration<float, std::milli>(loop_end - loop_start).count();

          if (step == 1) {
              // interval_log << servoj_rt_cmd_times << "," << (2.0f - sleep_duration_ms) << std::endl;
          } else {
              float sleep_time = 2.0f - loop_interval_ms;
              if (sleep_time > 0)
                  std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sleep_time * 1000)));

              auto loop_final = std::chrono::high_resolution_clock::now();
              float loop_final_interval_ms = std::chrono::duration<float, std::milli>(loop_final - loop_start).count();

              // interval_log << servoj_rt_cmd_times << "," << loop_final_interval_ms << std::endl;
          }

          // 👇 핵심 변경점: servoj_rt → speedj_rt (joint vel & acc)
          Drfl.speedj_rt(state.vel, state.acc, 0.3);  // DT = 0.002s
          predict_time += interval;
      }
    }
    return nullptr;
}


int main(int argc, char** argv) {

  Drfl.set_on_homming_completed(OnHommingCompleted);
  Drfl.set_on_monitoring_data(OnMonitoringDataCB);
  Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
  Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
  Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
  Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
  Drfl.set_on_log_alarm(OnLogAlarm);
  Drfl.set_on_tp_popup(OnTpPopup);
  Drfl.set_on_tp_log(OnTpLog);
  Drfl.set_on_tp_progress(OnTpProgress);
  Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
  Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

  Drfl.set_on_program_stopped(OnProgramStopped);
  Drfl.set_on_disconnected(OnDisConnected);

  assert(Drfl.open_connection("192.168.137.100"));

  SYSTEM_VERSION tSysVerion = {
      '\0',
  };
  Drfl.get_system_version(&tSysVerion);
  assert(Drfl.setup_monitoring_version(1));
  //Drfl.set_robot_control(CONTROL_SERVO_ON);
  //Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
  cout << "System version: " << tSysVerion._szController << endl;
  cout << "Library version: " << Drfl.get_library_version() << endl;

  while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
    this_thread::sleep_for(std::chrono::milliseconds(1000));
		cout << "get_robot_state : " << Drfl.get_robot_state() << endl;
    cout << "API control authority : " << g_bHasControlAuthority << endl;

  assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
  assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));


  typedef enum {
    EXAMPLE_JOG,
    EXAMPLE_HOME,
    EXAMPLE_MOVEJ_ASYNC,
    EXAMPLE_MOVEL_SYNC,
    EXAMPLE_MOVEJ_SYNC,
    EXAMPLE_DRL_PROGRAM,
    EXAMPLE_GPIO,
    EXAMPLE_MODBUS,
    EXAMPLE_LAST,
    EXAMPLE_SERVO_OFF
  } EXAMPLE;

  EXAMPLE eExample = EXAMPLE_LAST;

#ifdef __XENO__
  RT_TASK sub_task;
  char sub_task_name[256] = {
      '\0',
  };
  sprintf(sub_task_name, "drfl_sub_t");
  uint32_t stack_size = 1024 * 64;
  uint32_t prio = 50;
  if (rt_task_spawn(&sub_task, sub_task_name, stack_size, prio,
                    T_CPU(3) | /*T_SUSP |*/ T_JOINABLE,
                    (void (*)(void*)) & ThreadFunc, nullptr) != 0) {
    cout << "Can not create sub task" << endl;
  }
#else
#endif

  bool bLoop = TRUE;
  while (bLoop) {
    g_mStat = false;
    g_Stop = false;
#ifdef __XENO__
    unsigned long overrun = 0;
    const double tick = 1000000;  // 1ms
    rt_task_set_periodic(nullptr, TM_NOW, tick);
    if (rt_task_wait_period(&overrun) == -ETIMEDOUT) {
      std::cout << __func__ << ": \x1B[37m\x1B[41mover-runs: " << overrun
                << "\x1B[0m\x1B[0K" << std::endl;
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif  // __XENO__
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
    cout << "\ninput key : ";
    // char ch = _getch();
    char ch;
    cin >> ch;
    cout << ch << endl;
#endif

    switch (ch) {
      case 'q':
        bLoop = FALSE;
        break;
      case '0': {
        switch ((int)eExample) {
          case EXAMPLE_JOG:
            assert(Drfl.Jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, 0.f));
            cout << "jog stop" << endl;
            break;
          case EXAMPLE_HOME:
            assert(Drfl.Home((unsigned char)0));
            cout << "home stop" << endl;
            break;
          case EXAMPLE_MOVEJ_ASYNC:
            assert(Drfl.MoveStop(STOP_TYPE_SLOW));
            cout << "movej async stop" << endl;
            break;
          case EXAMPLE_MOVEL_SYNC:
          case EXAMPLE_MOVEJ_SYNC:
            break;
          case EXAMPLE_DRL_PROGRAM:
            assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
            // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
            // assert(Drfl.SetRobotSystem(ROBOT_SYSTEM_REAL));
            cout << "drl player stop" << endl;
            break;
          case EXAMPLE_GPIO:
            cout << "reset gpio" << endl;
            for (int i = 0; i < NUM_DIGITAL; i++) {
              // assert(Drfl.SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)i,
                                                  // FALSE));
            }
            break;
          case EXAMPLE_MODBUS:
            cout << "reset modbus" << endl;
            assert(Drfl.SetModbusValue("mr1", 0));
            break;
          default:
            break;
        }
      } break;
      case '1':
          {
            Drfl.connect_rt_control();
            string version = "v1.0";
            float period = 0.1;
            int losscount = 4;
            Drfl.set_rt_control_output(version, period, losscount);
            Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
            Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
						Drfl.start_rt_control();	
            float posj[6] = {0, 0, 90.0, 90, 0.0, 0};
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            Drfl.movej(posj, 30, 30);
            Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
            // 소켓 생성 및 서버 연결
            // sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                std::cerr << "Socket creation failed\n";
                return -1;
            }

            sockaddr_in serv_addr{};
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(PORT);
            inet_pton(AF_INET, server_ip, &serv_addr.sin_addr);  // 서버 IP 설정

            while (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Waiting for server...\n";
                usleep(100000);  // 100ms
            }

            std::cout << "Connected to server\n";

            // ------------------ 실시간 recv_thread 설정 ------------------
            pthread_t recv_thread;
            pthread_attr_t attr_recv;
            pthread_attr_init(&attr_recv);
            pthread_attr_setinheritsched(&attr_recv, PTHREAD_EXPLICIT_SCHED);
            pthread_attr_setschedpolicy(&attr_recv, SCHED_FIFO);

            sched_param param_recv;
            param_recv.sched_priority = PRIORITY_RECV;
            pthread_attr_setschedparam(&attr_recv, &param_recv);

            // ------------------ 실시간 control_thread 설정 ------------------
            pthread_t control_thread;
            pthread_attr_t attr_ctrl;
            pthread_attr_init(&attr_ctrl);
            pthread_attr_setinheritsched(&attr_ctrl, PTHREAD_EXPLICIT_SCHED);
            pthread_attr_setschedpolicy(&attr_ctrl, SCHED_FIFO);

            sched_param param_ctrl;
            param_ctrl.sched_priority = PRIORITY_CTRL;
            pthread_attr_setschedparam(&attr_ctrl, &param_ctrl);

            // ------------------ 스레드 시작 ------------------
            if (pthread_create(&recv_thread, &attr_recv, recv_thread_func, nullptr) != 0) {
                std::cerr << "Failed to create recv thread\n";
                return -1;
            }

            if (pthread_create(&control_thread, &attr_ctrl, control_thread_func, nullptr) != 0) {
                std::cerr << "Failed to create control thread\n";
                return -1;
        }		
          }
          break;
      case '2':
          {
              // string version = "v1.0";
              // float period = 0.1;
              // int losscount = 4;
              // Drfl.set_rt_control_output(version, period, losscount);
          }
          break;
      case '3':
          {
            // Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
            // Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
						// Drfl.start_rt_control();						
          }
          break;
      case '4': // servoj
      {
        float posj[6] = {0, 0, 90.0, 0, 90.0, 0};

        Drfl.movej(posj, 60, 60);
        Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

        float vel[6] = {100, 100, 100, 100, 100, 100};
        float acc[6] = {100, 100, 100, 100, 100, 100};
        float vel2[6] = {-10000, -10000,-10000,-10000,-10000,-10000};
        float acc2[6] = {-10000, -10000,-10000,-10000,-10000,-10000};
        Drfl.set_velj_rt(vel);
        Drfl.set_accj_rt(acc);
        const int port = 5005;
        const int dof = 6;
        const int total_floats = dof *3;
        const int buffer_size = 3 * dof * sizeof(float); // 6 * 4 = 24 bytes
        float dt = 0.1;
        if (sock < 0) {
            std::cerr << "Socket creation error\n";
            return -1;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        const int retry_delay_ms = 1000;  // 재시도 간격 (1초)

        while (true) {
            if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address / Address not supported\n";
                return -1;
            }

            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Waiting for server... retrying in " << retry_delay_ms << " ms\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                continue;
            }

            std::cout << "Connected to server.\n";
            break;  // 연결 성공 시 loop 탈출
        }
        std::cout << "Connected to server\n";
        int receive_count = 0;  // 데이터 수신 횟수 카운터

        while (true) {
            char buffer[buffer_size];
            int total_received = 0;

            while (total_received < buffer_size) {
                int bytes_received = recv(sock, buffer + total_received, buffer_size - total_received, 0);
                if (bytes_received <= 0) {
                    std::cerr << "Connection closed or error occurred\n";
                    close(sock);
                    return 0;
                }
                total_received += bytes_received;
            }
            
            float data[total_floats];
            std::memcpy(data, buffer, buffer_size);

            // 받은 데이터를 float 배열로 변환
            float joints[dof];
            float vel_rt[dof];
            float acc_rt[dof];

            for (int i = 0; i < dof; ++i) {
                joints[i] = data[i] * (180.0f / static_cast<float>(M_PI));  // rad → deg
                vel_rt[i] = std::abs(data[i + dof] * (180.0f / static_cast<float>(M_PI)));
                acc_rt[i] = std::abs(data[i + dof*2] * (180.0f / static_cast<float>(M_PI)));
            }
            // joints[5] = 0;
            // vel_rt[5] = 0;
            std::cout << "Position: ";
            for (int i = 0; i < 6; ++i)
                std::cout << joints[i] << " ";
            std::cout << std::endl;

            std::cout << "Velocity: ";
            for (int i = 0; i < 6; ++i)
            {
                std::cout << vel_rt[i] << " ";
            }
            std::cout << std::endl;
            std::cout << "Acceleration: ";
            for (int i = 0; i < 6; ++i)
            {
                std::cout << acc_rt[i] << " ";
            }
            std::cout << std::endl;

            Drfl.servoj_rt(joints, vel2, acc2, dt);  
            std::cout << "Executed servoj_rt()" << std::endl;
            //this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        close(sock);
        // pthread_t rt_thread;
        // pthread_attr_t attr;
        // struct sched_param param;
        // pthread_attr_init(&attr);
        // pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        // pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

        // param.sched_priority = 99;
        // pthread_attr_setschedparam(&attr, &param);
        // int v = pthread_create(&rt_thread, &attr, realtime_task, nullptr);
        // if (v != 0) {
        //     perror("Failed to create real-time thread ");
        //     std::cout << "Err " << v << std::endl;
        //     return 1;
        // }
        // pthread_join(rt_thread, nullptr);
        // pthread_attr_destroy(&attr);
      }
      break;
      case '5':
      {
        const int port = 5005;
        
        // int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation error\n";
            return -1;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        const int retry_delay_ms = 1000;  // 재시도 간격 (1초)
        
        while (true) {
            if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address / Address not supported\n";
                return -1;
            }

            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Waiting for server... retrying in " << retry_delay_ms << " ms\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                continue;
            }

            std::cout << "Connected to server.\n";
            break;  // 연결 성공 시 loop 탈출
        }
        std::cout << "Connected to server\n";
        pthread_t rt_thread;
        pthread_attr_t attr;
        struct sched_param param;
        pthread_attr_init(&attr);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

        param.sched_priority = 99;
        pthread_attr_setschedparam(&attr, &param);
        // int v = pthread_create(&rt_thread, &attr, realtime_task_speedj, nullptr);
        int v = pthread_create(&rt_thread, &attr, realtime_task_isaac_sim, nullptr);
        if (v != 0) {
            perror("Failed to create real-time thread ");
            std::cout << "Err " << v << std::endl;
            return 1;
        }
      }break; 
      case '6':
      {
        float posj[6] = {0, 0, 90.0, 90, 0.0, 0};

        Drfl.movej(posj, 30, 30);
        Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        // 소켓 생성 및 서버 연결
        // sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation failed\n";
            return -1;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        inet_pton(AF_INET, server_ip, &serv_addr.sin_addr);  // 서버 IP 설정

        while (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "Waiting for server...\n";
            usleep(100000);  // 100ms
        }

        std::cout << "Connected to server\n";

        // ------------------ 실시간 recv_thread 설정 ------------------
        pthread_t recv_thread;
        pthread_attr_t attr_recv;
        pthread_attr_init(&attr_recv);
        pthread_attr_setinheritsched(&attr_recv, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_recv, SCHED_FIFO);

        sched_param param_recv;
        param_recv.sched_priority = PRIORITY_RECV;
        pthread_attr_setschedparam(&attr_recv, &param_recv);

        // ------------------ 실시간 control_thread 설정 ------------------
        pthread_t control_thread;
        pthread_attr_t attr_ctrl;
        pthread_attr_init(&attr_ctrl);
        pthread_attr_setinheritsched(&attr_ctrl, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_ctrl, SCHED_FIFO);

        sched_param param_ctrl;
        param_ctrl.sched_priority = PRIORITY_CTRL;
        pthread_attr_setschedparam(&attr_ctrl, &param_ctrl);

        // ------------------ 스레드 시작 ------------------
        if (pthread_create(&recv_thread, &attr_recv, recv_thread_func, nullptr) != 0) {
            std::cerr << "Failed to create recv thread\n";
            return -1;
        }

        if (pthread_create(&control_thread, &attr_ctrl, control_thread_func, nullptr) != 0) {
            std::cerr << "Failed to create control thread\n";
            return -1;
        }

        // close(sock);
      }
      break;
      case '7':
      {
        float posj[6] = {0, 0, 90.0, 0, 90.0, 0};

        Drfl.movej(posj, 60, 60);
        Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        // 소켓 생성 및 서버 연결
        // sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation failed\n";
            return -1;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        inet_pton(AF_INET, server_ip, &serv_addr.sin_addr);  // 서버 IP 설정

        while (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "Waiting for server...\n";
            usleep(100000);  // 100ms
        }

        std::cout << "Connected to server\n";

        // ------------------ 실시간 recv_thread 설정 ------------------
        pthread_t recv_thread;
        pthread_attr_t attr_recv;
        pthread_attr_init(&attr_recv);
        pthread_attr_setinheritsched(&attr_recv, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_recv, SCHED_FIFO);

        sched_param param_recv;
        param_recv.sched_priority = PRIORITY_RECV;
        pthread_attr_setschedparam(&attr_recv, &param_recv);

        // ------------------ 실시간 control_thread 설정 ------------------
        pthread_t control_thread;
        pthread_attr_t attr_ctrl;
        pthread_attr_init(&attr_ctrl);
        pthread_attr_setinheritsched(&attr_ctrl, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_ctrl, SCHED_FIFO);

        sched_param param_ctrl;
        param_ctrl.sched_priority = PRIORITY_CTRL;
        pthread_attr_setschedparam(&attr_ctrl, &param_ctrl);

        // ------------------ 스레드 시작 ------------------
        if (pthread_create(&recv_thread, &attr_recv, recv_thread_func, nullptr) != 0) {
            std::cerr << "Failed to create recv thread\n";
            return -1;
        }

        if (pthread_create(&control_thread, &attr_ctrl, control_thread_func_speedj, nullptr) != 0) {
            std::cerr << "Failed to create control thread\n";
            return -1;
        }

      }
      break;
      case 'exit':
		  {
			  Drfl.stop_rt_control();
				Drfl.disconnect_rt_control();

		  }
		  break;
      default:
        break;
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Drfl.disconnect_rt_control();
  Drfl.CloseConnection();

#ifdef __XENO__
  rt_task_join(&sub_task);
#endif // __XENO__
  return 0;

}
