/*********************************************************************
 * Copyright (C) 2015- Future Robotics Technology Center (fuRo),
 *                     Chiba Institute of Technology.
 *                     All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * fuRo ("Confidential Information"). You shall not disclose such
 * Confidential Information and shall use it only in accordance with
 * the terms of the license agreement you entered into with fuRo.
 *
 * @author Yoshitaka Hara
 *********************************************************************/

#ifndef RRC_CMD_HPP
#define RRC_CMD_HPP

#include <string>

// RRC 側の状態遷移
enum RrcStateT {
    STATE_POWER_ON_WAIT,
    STATE_POWER_ON,
//    STATE_IDLE,
//    STATE_LIMIT,
//    STATE_OUTPUT,
    STATE_RUNNING,
    STATE_POWER_OFF_WAIT,
    STATE_POWER_OFF
};

// RRC 受信ステータス情報
struct RrcRecvStatus {
    int timestamp_ms;  // typeS
    int v_pct_in;   // [%]
    int w_pct_in;   // [%]
    int v_pct_out;  // [%]
    int w_pct_out;  // [%]
    int voltage_gauge;
    int speed_mode;
    int speed_up_cnt;
    int speed_down_cnt;
    int alarm_cnt;
    int enc0_cnt;  // typeB or later
    int enc1_cnt;  // typeB or later
    // TODO: typeS のメンバ変数を追加
};

namespace rrc_cmd
{

// serialize (send)
std::string encLimit(int v_pct, int w_pct, int vw_pct_sum, int vw_pct_product, int vw_pct_length, bool checksum);  // vw_pct_sum：車体外周速度相当値、vw_pct_product：遠心加速度相当値
std::string encOutput(int v_pct, int w_pct, bool checksum);
std::string encIdle(bool checksum);

std::string encDevice(bool checksum);
std::string encVersion(bool checksum);
std::string encDate(bool checksum);

std::string encSpeedMode(int speedmode, bool lock, bool checksum);
std::string encSpeedModeArea(int min, int max, bool checksum);

std::string s1set(int time, bool checksum);
std::string s2set(int jm, int pm, int jm_led, int jm_button, bool checksum);
std::string s3set(bool checksum);



std::string encJoystick(int vl, int wl, int vh, int wh, int ve, int we, int bm, int rs, int fbr, int br, bool checksum);
std::string encJoystickAdjust(int pvl, int pvh, int mvl, int mvh, int pwl, int pwh, int mwl, int mwh, int pve, int mve, int pwe, int mwe, bool checksum);
std::string encJoystickBack(int bm, int rs, int fbr, int br, bool checksum);

std::string encWatchdog(int t_10ms, bool checksum);
std::string encStableMode(bool enable, bool checksum);

std::string encStatus(char mode,bool enable, bool checksum);
std::string encEncoderResistance(char mode, bool checksum);
std::string encRelayError(int count, bool checksum);
std::string encUnlockBrake(int val, bool checksum);
std::string encForcePowerOff(bool checksum);
std::string encBeep(unsigned char t_10ms[10], bool checksum);

// deserialize (recv)
RrcRecvStatus decStatusAB(const std::string &recv_line);  // recv_line.size() == 17 || 21
RrcRecvStatus decStatusS(const std::string &recv_line);

char makeCheckSum(const std::string::const_iterator &cmd_first, const std::string::const_iterator &cmd_last);

inline bool recvChecksumOk(const std::string &recv_line)
{
    return (makeCheckSum(recv_line.begin(), recv_line.end() - 1) == recv_line.back());
}

} // namespace rrc_cmd

#endif // RRC_CMD_HPP
