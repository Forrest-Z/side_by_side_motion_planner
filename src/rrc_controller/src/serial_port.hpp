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
 * @author Yoshitaka Hara, Kiyoshi Irie
 *********************************************************************/

/*
 * 参考 URL:
 * Streams, Short Reads and Short Writes
 * http://www.boost.org/doc/libs/release/doc/html/boost_asio/overview/core/streams.html
 * Boost.Asio streambufからstd::stringに変換する - Faith and Brave - C++で遊ぼう
 * http://faithandbrave.hateblo.jp/entry/20110324/1300950590
 * Boost.Asio バッファの消費 - Faith and Brave - C++で遊ぼう
 * http://faithandbrave.hateblo.jp/entry/20110830/1314716828
 * boost::streambufの内部構造の調査 - catalinaの備忘録
 * http://catalina1344.hatenablog.jp/entry/2014/11/14/094608
 * Boost.Asio 非同期処理のタイムアウトを設定する - Faith and Brave - C++で遊ぼう
 * http://faithandbrave.hateblo.jp/entry/20110325/1301051480
 * C++で文字列のsplit | Story of Your Life
 * http://shnya.jp/blog/?p=195
 */

#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
//#include <thread>

#define LF (0x0A)
#define CR (0x0D)
#define CRLF (0x0D0A)

class SerialPort {
public:
    enum FlowControlT {
        FLOW_CTRL_NONE,
        FLOW_CTRL_SOFTWARE,
        FLOW_CTRL_HARDWARE
    };
    enum ParityT {
        PARITY_NONE,
        PARITY_ODD,
        PARITY_EVEN
    };
    enum StopBitsT {
        STOP_BITS_ONE,
        STOP_BITS_ONEPOINTFIVE,
        STOP_BITS_TWO
    };

    SerialPort() : port_(io_srv_), dl_timer_(io_srv_) {}

    virtual ~SerialPort() = default;

    // ポート開閉
    boost::system::error_code open(const std::string &dev_name)
    {
        boost::system::error_code err_code;
        port_.open(dev_name, err_code);
        return err_code;
    }
    boost::system::error_code close()
    {
        boost::system::error_code err_code;
        port_.close(err_code);
        return err_code;
    }

    // 設定
    void setBaudRate(unsigned int rate);
    void setFlowControl(FlowControlT flow_ctrl = FLOW_CTRL_NONE);
    void setParity(ParityT parity = PARITY_NONE);
    void setStopBits(StopBitsT stop_bits = STOP_BITS_ONE);
    void setCharacterSize(unsigned int char_size = 8);

    // 送信
    std::size_t write(const std::string &buffer);
    std::size_t write(const char *buffer, std::size_t len);
    std::size_t write(const std::vector<char> &buffer);

    // 受信
    // TODO: char* 型などの関数を作成
    // TODO: TimeOut 系の関数を追加
    std::size_t read(std::string &buffer);
    std::size_t readNBytes(std::string &buffer, const std::size_t read_size);
    std::size_t readUntil(std::string &buffer, const std::string &delimiter);
    std::size_t readUntilTimeOut(std::string &buffer, const std::string &delimiter, const int timeout_ms);

private:
    void timeoutCallback(const boost::system::error_code &err_code);
    void asyncReadCallback(std::size_t &read_size, const boost::system::error_code &err_code, const std::size_t bytes_transferred);

    boost::asio::io_service io_srv_;
    boost::asio::serial_port port_;
    boost::asio::streambuf receive_buf_;
    boost::asio::deadline_timer dl_timer_;
    boost::mutex write_mutex_;
    boost::mutex read_mutex_;
    boost::mutex recv_buf_mutex_;
};

inline void SerialPort::setBaudRate(unsigned int rate)
{
    port_.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

inline void SerialPort::setCharacterSize(unsigned int char_size)
{
    port_.set_option(boost::asio::serial_port_base::character_size(char_size));
}

inline std::size_t SerialPort::write(const std::string &buffer)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    const std::vector<char> buf_cvec(buffer.begin(), buffer.end());
    return boost::asio::write(port_, boost::asio::buffer(buf_cvec));
}

inline std::size_t SerialPort::write(const char *buffer, std::size_t len)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    return boost::asio::write(port_, boost::asio::buffer(buffer, len));
}

inline std::size_t SerialPort::write(const std::vector<char> &buffer)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    return boost::asio::write(port_, boost::asio::buffer(buffer));
}

#endif // SERIAL_PORT_HPP
