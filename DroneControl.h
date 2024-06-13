#ifndef PRECISIONLANDING_DRONECONTROL_H
#define PRECISIONLANDING_DRONECONTROL_H

#include "DroneSearch.h"
#include <iostream>

using namespace std;
using namespace mavsdk;
using std::chrono::seconds;

class DroneControl : public DroneSearch{
public:
    //생성자 및 소멸자
    DroneControl(mavsdk::Mavsdk &mavsdk, std::string port);

    ~DroneControl(){};

    bool inAir();

    float getAltitude();
    //가속도계, 자이로미터, 자력계, GPS 등 교정 상태 확인 여부
    bool armable();

    // 시스템 무장
    bool arm();

    //$(takeoff_altitude)m 시스템 이륙
    bool takeoff(const float takeoff_altitude);

    //자동적으로 현재의 위치, 기수 방향 및 고도를 유지하려고 시도
    //조종사가 콥터를 마치 수동 비행 모드인 것처럼 날릴 수 있지만, 스틱을 놓으면 천천히 정지하고
    //기압, 바람 등 다수의 외력의 영향을 받지 않고 위치를 유지
    //bool loiter();

    //시스템 착륙
    bool land();

    //Return to Launch의 약자로 telemetry.home()에 정의된 경도, 위도로 이동 후 시스템 착륙
    //따로 지정하지 않으면 이륙을 했던 경, 위도에서 착륙
    bool RTL(const float RTL_altitude);

    //telemetry.home()에 정의된 경도, 위도로 이동, 착륙은 하지 않음
    //bool RTL_notLand();

    //**주의** | 가급적 사용 지양
    //착륙 또는 비행 여부에 관계없이 시스템 무장 해제 / 비행 중에 사용 시 시스템이 하늘에서 떨어짐
    bool danger();

    //몸체 좌표계에서 north, east, down의 방향 속도(m/s)와 yaw의 각속도(radian/s)를 설정
    bool move_vel_byBody(float north=0, float east=0, float down=0, float yaw=0);

    //몸체 좌표계에서 yaw의 각속도(radian/s)를 설정
    //bool turn_body_byBody(float yaw=0);

    //NED 좌표계(North-East-Down)에서 north, east, down의 방향 속도(m/s)와
    //NED 좌표계에서의 자세 회전 각도를 설정
    //bool move_vel_byNed(float north=0, float east=0, float down=0, float yaw=0);

    //NED 좌표계(North-East-Down)에서 north, east, down의 목표 위치 (m) 와
    //NED 좌표계에서의 자세 회전 각도를 설정
    //bool move_pos_byNed(float north, float east, float down=0, float yaw=400);

    bool move_altitude(float down=0);

    //NED 좌표계에서 자세 회전 각도를 설정
    bool turn_body_byNed(float yaw=0);
};


#endif //PRECISIONLANDING_DRONECONTROL_H
