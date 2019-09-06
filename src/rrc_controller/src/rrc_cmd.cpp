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

#include "rrc_cmd.hpp"

namespace rrc_cmd
{

char int2char(int val)
{
    char ret = '0';
    if (0 <= val && val <= 9)
    {
        ret = static_cast<char>(val + '0');
    }
    else if (10 <= val && val <= 16)
    {
        ret = static_cast<char>(val + 'A' - 10);  // 大文字
    }
    return ret;
}

int char2int(char val)
{
    int ret = 0;
    if ('0' <= val && val <= '9' )
    {
        ret = val - '0';
    }
    else if ('a' <= val && val <= 'f')
    {
        ret = val - 'a' + 10;
    }
    else if ('A' <= val && val <= 'F')
    {
        ret = val - 'A' + 10;
    }
    return ret;
}

char makeCheckSum(const std::string::const_iterator &cmd_first, const std::string::const_iterator &cmd_last)
{
    char sum = 0x00;
    for (auto iter = cmd_first; iter != cmd_last; ++iter)
    {
        sum += *iter;
    }
    return int2char(static_cast<int>(sum & 0x0f));
}

void addCmdEnd(std::string *cmd, bool checksum)
{
    if (checksum)
    {
        cmd->push_back(makeCheckSum(cmd->begin(), cmd->end()));
    }
    cmd->push_back('\n');
}

// TODO: 以下、引数が値域を超えている場合にエラーではなく最大値でコマンドを生成する？

std::string encLimit(int v_pct, int w_pct, int vw_pct_sum, int vw_pct_product, int vw_pct_length, bool checksum)
{
    bool c1 = (0 <= v_pct) && (v_pct <= 100);
    bool c2 = (0 <= w_pct) && (w_pct <= 100);
    bool c3 = (0 <= vw_pct_sum) && (vw_pct_sum <= 255);
    bool c4 = (0 <= vw_pct_product) && (vw_pct_product <= 255);
    bool c5 = (0 <= vw_pct_length) && (vw_pct_length <= 255);
    if (!(c1 && c2 && c3 && c4 && c5)) { return "err:func:limit\n"; }

    std::string cmd("limit");
    cmd.push_back(int2char((v_pct >> 4) & 0x0F));
    cmd.push_back(int2char(v_pct & 0x0F));
    cmd.push_back(int2char((w_pct >> 4) & 0x0F));
    cmd.push_back(int2char(w_pct & 0x0F));
    cmd.push_back(int2char((vw_pct_sum >> 4) & 0x0F));
    cmd.push_back(int2char(vw_pct_sum & 0x0F));
    cmd.push_back(int2char((vw_pct_product >> 4) & 0x0F));
    cmd.push_back(int2char(vw_pct_product & 0x0F));
    cmd.push_back(int2char((vw_pct_length >> 4) & 0x0F));
    cmd.push_back(int2char(vw_pct_length & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encOutput(int v_pct, int w_pct, bool checksum)
{
    bool c1 = (-100 <= v_pct) && (v_pct <= 100);
    bool c2 = (-100 <= w_pct) && (w_pct <= 100);
    if (!(c1 && c2)) { return "err:func:output\n"; }

    std::string cmd("output");
    cmd.push_back(int2char((v_pct >> 4) & 0x0F));
    cmd.push_back(int2char(v_pct & 0x0F));
    cmd.push_back(int2char((w_pct >> 4) & 0x0F));
    cmd.push_back(int2char(w_pct & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encIdle(bool checksum)
{
    std::string cmd("idle");
    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encDevice(bool checksum)
{
    std::string cmd("device");
    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encVersion(bool checksum)
{
    std::string cmd("version");
    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encDate(bool checksum)
{
    std::string cmd("date");
    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encSpeedMode(int speedmode, bool lock, bool checksum)
{
    bool c1 = (1 <= speedmode) && (speedmode <= 5);
    if (!c1) { return "err:func:speedmode\n"; }

    std::string cmd("speedmode");
    cmd.push_back(int2char(speedmode));
    cmd.push_back(lock ? '1' : '0');

    addCmdEnd(&cmd, checksum);
    
    return cmd;

}

  std::string s1set(int time, bool checksum)
{
    std::string cmd("s1set");
    cmd.push_back(int2char(time & 0x0F));
    addCmdEnd(&cmd, checksum);    
    return cmd;

}

std::string s2set(int jm, int pm, int jm_led, int jm_button, bool checksum)
{
  //echo 's2set1011' > /dev/ttyACM0
    std::string cmd("s2set");

    //cmd.push_back(int2char((jm >> 4) & 0x0F));
    cmd.push_back(int2char(jm & 0x0F));

    //cmd.push_back(int2char((pm >> 4) & 0x0F));
    cmd.push_back(int2char(pm & 0x0F));

    //cmd.push_back(int2char((jm_led >> 4) & 0x0F));
    cmd.push_back(int2char(jm_led & 0x0F));
    
    //cmd.push_back(int2char((jm_button >> 4) & 0x0F));
    cmd.push_back(int2char(jm_button & 0x0F));

    addCmdEnd(&cmd, checksum);    
    return cmd;

}

std::string s3set(bool checksum)
{
    std::string cmd("s3set10");
    addCmdEnd(&cmd, checksum);    
    return cmd;

}

std::string encSpeedModeArea(int min, int max, bool checksum)
{
    bool c1 = (1 <= min) && (min <= 5);
    bool c2 = (1 <= max) && (max <= 5);
    if (!c1 || !c2) { return "err:func:speedmodearea\n"; }

    std::string cmd("speedmodearea");
    cmd.push_back(int2char(min));
    cmd.push_back(int2char(max));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encJoystick(int vl, int wl, int vh, int wh, int ve, int we, int bm, int rs, int fbr, int br, bool checksum)
{
    bool c1 = (vl < vh) && (0 <= vl) && (vl <= 100) && (0 <= vh) && (vh <= 100);
    bool c2 = (wl < wh)	&& (0 <= wl) && (wl <= 100) && (0 <= wh) && (wh <= 100);
    bool c3 = (0 <= ve) && (ve <= 15);
    bool c4 = (0 <= we) && (we <= 15);
    bool c5 = (0 <= bm) && (bm <= 3);
    bool c6 = (0 <= rs) && (rs <= 100);
    bool c7 = (0 <= fbr) && (fbr <= 90);
    bool c8 = (0 <= br) && (br <= 90);
    if (!(c1 && c2 && c3 && c4 && c5 && c6 && c7 && c8)) { return "err:func:joystick\n"; }

    std::string cmd("joystick");
    cmd.push_back(int2char((vl >> 4) & 0x0F));
    cmd.push_back(int2char(vl & 0x0F));
    cmd.push_back(int2char((wl >> 4) & 0x0F));
    cmd.push_back(int2char(wl & 0x0F));
    cmd.push_back(int2char((vh >> 4) & 0x0F));
    cmd.push_back(int2char(vh & 0x0F));
    cmd.push_back(int2char((wh >> 4) & 0x0F));
    cmd.push_back(int2char(wh & 0x0F));
    cmd.push_back(int2char(ve & 0x0F));
    cmd.push_back(int2char(we & 0x0F));
    cmd.push_back(int2char(bm & 0x0F));
    cmd.push_back(int2char((rs >> 4) & 0x0F));
    cmd.push_back(int2char(rs & 0x0F));
    cmd.push_back(int2char((fbr >> 4) & 0x0F));
    cmd.push_back(int2char(fbr & 0x0F));
    cmd.push_back(int2char((br >> 4) & 0x0F));
    cmd.push_back(int2char(br & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encJoystickAdjust(int pvl, int pvh, int mvl, int mvh, int pwl, int pwh, int mwl, int mwh, int pve, int mve, int pwe, int mwe, bool checksum)
{
    bool c1 = (pvl < pvh) && (0 <= pvl) && (pvl <= 100) && (0 <= pvh) && (pvh <= 100);
    bool c2 = (mvl < mvh) && (0 <= mvl) && (mvl <= 100) && (0 <= mvh) && (mvh <= 100);
    bool c3 = (pwl < pwh)	&& (0 <= pwl) && (pwl <= 100) && (0 <= pwh) && (pwh <= 100);
    bool c4 = (mwl < mwh)	&& (0 <= mwl) && (mwl <= 100) && (0 <= mwh) && (mwh <= 100);
    bool c5 = (0 <= pve) && (pve <= 15);
    bool c6 = (0 <= mve) && (mve <= 15);
    bool c7 = (0 <= pwe) && (pwe <= 15);
    bool c8 = (0 <= mwe) && (mwe <= 15);
    if (!(c1 && c2 && c3 && c4 && c5 && c6 && c7 && c8)) { return "err:func:joystickadjust\n"; }

    std::string cmd("joystickadjust");
    cmd.push_back(int2char((pvl >> 4) & 0x0F));
    cmd.push_back(int2char(pvl & 0x0F));
    cmd.push_back(int2char((pvh >> 4) & 0x0F));
    cmd.push_back(int2char(pvh & 0x0F));
    cmd.push_back(int2char((mvl >> 4) & 0x0F));
    cmd.push_back(int2char(mvl & 0x0F));
    cmd.push_back(int2char((mvh >> 4) & 0x0F));
    cmd.push_back(int2char(mvh & 0x0F));
    cmd.push_back(int2char((pwl >> 4) & 0x0F));
    cmd.push_back(int2char(pwl & 0x0F));
    cmd.push_back(int2char((pwh >> 4) & 0x0F));
    cmd.push_back(int2char(pwh & 0x0F));
    cmd.push_back(int2char((mwl >> 4) & 0x0F));
    cmd.push_back(int2char(mwl & 0x0F));
    cmd.push_back(int2char((mwh >> 4) & 0x0F));
    cmd.push_back(int2char(mwh & 0x0F));
    cmd.push_back(int2char(pve & 0x0F));
    cmd.push_back(int2char(mve & 0x0F));
    cmd.push_back(int2char(pwe & 0x0F));
    cmd.push_back(int2char(mwe & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encJoystickBack(int bm, int rs, int fbr, int br, bool checksum)
{
    bool c1 = (0 <= bm) && (bm <= 3);
    bool c2 = (0 <= rs) && (rs <= 100);
    bool c3 = (0 <= fbr) && (fbr <= 90);
    bool c4 = (0 <= br) && (br <= 90);
    if (!(c1 && c2 && c3 && c4)) { return "err:func:joystickback\n"; }

    std::string cmd("joystickback");
    cmd.push_back(int2char(bm & 0x0F));
    cmd.push_back(int2char((rs >> 4) & 0x0F));
    cmd.push_back(int2char(rs & 0x0F));
    cmd.push_back(int2char((fbr >> 4) & 0x0F));
    cmd.push_back(int2char(fbr & 0x0F));
    cmd.push_back(int2char((br >> 4) & 0x0F));
    cmd.push_back(int2char(br & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encWatchdog(int t_10ms, bool checksum)
{
    std::string cmd("watchdog");
    cmd.push_back(int2char((t_10ms >> 4) & 0x0F));
    cmd.push_back(int2char(t_10ms & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encStableMode(bool enable, bool checksum)
{
    std::string cmd("stablemode");
    cmd.push_back(enable ? '1' : '0');

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encStatus(char mode, bool enable, bool checksum)
{
    bool c1 = (mode == 'S') || (mode == 'B');
    if (!c1) { return "err:func:status\n"; }

    std::string cmd("status");
    cmd.push_back(mode);
    cmd.push_back(enable ? '1' : '0');

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encEncoderResistance(char mode, bool checksum)
{
    bool c1 = (mode == '0') || (mode == '1') || (mode == '2');
    if (!c1) { return "err:func:encoderresistance\n"; }

    std::string cmd("encoderresistance");
    cmd.push_back(mode);

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encRelayError(int count, bool checksum)
{
    bool c1 = (1 <= count) && (count <= 9);
    if (!c1) { return "err:func:relayerror\n"; }

    std::string cmd("relayerror");
    cmd.push_back(int2char(count));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encUnlockBrake(int val, bool checksum)
{
    bool c1 = (-100 <= val) && (val <= 100);
    if (!c1) { return "err:func:unlockbrake"; }

    std::string cmd("unlockbrake");
    cmd.push_back(int2char((val >> 4) & 0x0F));
    cmd.push_back(int2char(val & 0x0F));

    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encForcePowerOff(bool checksum)
{
    std::string cmd("forcepoweroff");
    addCmdEnd(&cmd, checksum);

    return cmd;
}

std::string encBeep(unsigned char t_10ms[10], bool checksum)
{
    std::string cmd("beep");
    for (std::size_t idx = 0; idx <= 9; ++idx)
    {
        cmd.push_back(int2char((t_10ms[idx] >> 4) & 0x0F));
        cmd.push_back(int2char(t_10ms[idx] & 0x0F));
    }

    addCmdEnd(&cmd, checksum);

    return cmd;
}


  //SFF00FF008100000004
RrcRecvStatus decStatusAB(const std::string &recv_line)
{
    RrcRecvStatus status;

    status.timestamp_ms = static_cast<signed char>((char2int(recv_line[1]) << 4) | (char2int(recv_line[2]))) + 125 ; 
    status.v_pct_in = static_cast<signed char>((char2int(recv_line[3]) << 4) | (char2int(recv_line[4])));
    status.w_pct_in = static_cast<signed char>((char2int(recv_line[5]) << 4) | (char2int(recv_line[6])));
    status.v_pct_out = static_cast<signed char>((char2int(recv_line[7]) << 4) | (char2int(recv_line[8])));
    status.w_pct_out = static_cast<signed char>((char2int(recv_line[9]) << 4) | (char2int(recv_line[10])));
    status.voltage_gauge = char2int(recv_line[11]);
    status.speed_mode = char2int(recv_line[12]);
    status.speed_up_cnt = char2int(recv_line[13]);
    status.speed_down_cnt = char2int(recv_line[14]);
    status.alarm_cnt = char2int(recv_line[15]);

    status.enc0_cnt = (char2int(recv_line[16]) << 4) | (char2int(recv_line[17])) ;  // 左エンコーダ
    status.enc1_cnt = (char2int(recv_line[18]) << 4) | (char2int(recv_line[19])) ;  // 右エンコーダ
    //bool is_status_B = (recv_line.substr(0, 3) == "SB:");
    //status.enc0_cnt = is_status_B ? (char2int(recv_line[16]) << 4) | (char2int(recv_line[17])) : 0;  // 左エンコーダ
    //status.enc1_cnt = is_status_B ? (char2int(recv_line[18]) << 4) | (char2int(recv_line[19])) : 0;  // 右エンコーダ

    return status;
}

RrcRecvStatus decStatusS(const std::string &recv_line)
{
    RrcRecvStatus status;

    // TODO: 実装
//    status.timestamp_ms = ;



    return status;
}

} // namespace rrc_cmd
