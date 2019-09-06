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

#include "serial_port.hpp"

#include <iostream>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
//#include <boost/utility.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>

void SerialPort::setFlowControl(FlowControlT flow_ctrl)
{
    switch (flow_ctrl)
    {
    case FLOW_CTRL_NONE:
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        break;
    case FLOW_CTRL_SOFTWARE:
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
        break;
    case FLOW_CTRL_HARDWARE:
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));
        break;
    }
}

void SerialPort::setParity(ParityT parity)
{
    switch (parity)
    {
    case PARITY_NONE:
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        break;
    case PARITY_ODD:
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
        break;
    case PARITY_EVEN:
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
        break;
    }
}

void SerialPort::setStopBits(StopBitsT stop_bits)
{
    switch (stop_bits)
    {
    case STOP_BITS_ONE:
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        break;
    case STOP_BITS_ONEPOINTFIVE:
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::onepointfive));
        break;
    case STOP_BITS_TWO:
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
        break;
    }
}

#if 0  // short write してしまう
std::size_t SerialPort::writeSome(const std::string &buffer)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    const std::vector<char> buf_cvec(buffer.begin(), buffer.end());
    return port_.write_some(boost::asio::buffer(buf_cvec));
}

std::size_t SerialPort::writeSome(const char *buffer, std::size_t len)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    return port_.write_some(boost::asio::buffer(buffer, len));
}

std::size_t SerialPort::writeSome(const std::vector<char> &buffer)
{
    boost::lock_guard<boost::mutex> write_lock(write_mutex_);
    return port_.write_some(boost::asio::buffer(buffer));
}
#endif

#if 0  // short read してしまう
std::size_t SerialPort::readSome(std::string &buffer)
{
    boost::lock_guard<boost::mutex> read_lock(read_mutex_);
    std::vector<char> buffer;
    return port_.read_some(boost::asio::buffer(buffer));
    buffer.assign(buffer.begin(), buffer.end());
}

std::size_t SerialPort::readSome(char *buffer, std::size_t len)
{
    boost::lock_guard<boost::mutex> read_lock(read_mutex_);
    return port_.read_some(boost::asio::buffer(buffer, len));
}

std::size_t SerialPort::readSome(std::vector<char> &buffer)
{
    boost::lock_guard<boost::mutex> read_lock(read_mutex_);
    return port_.read_some(boost::asio::buffer(buffer));
}
#endif

std::size_t SerialPort::read(std::string &buffer)
{
    boost::lock_guard<boost::mutex> recv_buf_lock(recv_buf_mutex_);
    std::size_t bytes_transferred;
    {
        boost::lock_guard<boost::mutex> read_lock(read_mutex_);
        bytes_transferred = boost::asio::read(port_, receive_buf_, boost::asio::transfer_at_least(1));
//            bytes_transferred = boost::asio::read(port_, receive_buf_, boost::asio::transfer_all());
    }
//        buffer.assign(boost::asio::buffer_cast<const char*>(receive_buf_.data()));  // 以前のデータが残るのでダメ
    buffer.assign(boost::asio::buffer_cast<const char*>(receive_buf_.data()), receive_buf_.size());
    receive_buf_.consume(receive_buf_.size());  // 受信サイズ分のバッファを消費する
    return bytes_transferred;
}

std::size_t SerialPort::readNBytes(std::string &buffer, const std::size_t read_size)
{
    boost::lock_guard<boost::mutex> recv_buf_lock(recv_buf_mutex_);
    std::size_t bytes_transferred;
    {
        boost::lock_guard<boost::mutex> read_lock(read_mutex_);
        bytes_transferred = boost::asio::read(port_, receive_buf_, boost::asio::transfer_exactly(read_size));
    }
//        buffer.assign(boost::asio::buffer_cast<const char*>(receive_buf_.data()));  // 以前のデータが残るのでダメ
    buffer.assign(boost::asio::buffer_cast<const char*>(receive_buf_.data()), receive_buf_.size());
    receive_buf_.consume(receive_buf_.size());  // 受信サイズ分のバッファを消費する
    return bytes_transferred;
}

std::size_t SerialPort::readUntil(std::string &buffer, const std::string &delimiter)
{
    boost::lock_guard<boost::mutex> recv_buf_lock(recv_buf_mutex_);

    // TODO: read_until() する前に receive_buf_ に delimiter がないか調べる


    {
        boost::lock_guard<boost::mutex> read_lock(read_mutex_);
        boost::asio::read_until(port_, receive_buf_, delimiter);
    }
    // read_until() はデリミタ以降の数文字も一緒に読む場合があるので分割する
    std::string recv_buf(boost::asio::buffer_cast<const char*>(receive_buf_.data()), receive_buf_.size());
#if 0 // デリミタが複数文字の場合に is_any_of() の1文字で区切ってしまうのでダメ
    std::vector<std::string> split_str;
    boost::split(split_str, recv_buf, boost::is_any_of(delimiter));
//        boost::split(split_str, recv_buf, boost::is_equal(delimiter));  // コンパイルエラー
//        boost::split(split_str, recv_buf, boost::bind(std::equal_to<std::string>(), delimiter));  // コンパイルエラー
    buffer.assign(split_str.front());
    receive_buf_.consume(buffer.size() + 1);  // デリミタまでのバッファを消費する
//        boost::trim_left_if(buffer, boost::is_any_of(delimiter));  // 先頭に残ったデリミタを削除→デリミタに対して split() は空文字列を返すので不要
#else
    std::size_t delim_pos = recv_buf.find(delimiter, 0);
    if (delim_pos != std::string::npos)  // デリミタを発見
    {
        buffer.assign(recv_buf, 0, delim_pos);
        receive_buf_.consume(buffer.size() + delimiter.size());  // デリミタまでのバッファを消費する
    }
    else  // デリミタまで受信していない
    {
        buffer.clear();
    }
#endif
    return buffer.size();
}

std::size_t SerialPort::readUntilTimeOut(std::string &buffer, const std::string &delimiter, const int timeout_ms)
{
    boost::lock_guard<boost::mutex> recv_buf_lock(recv_buf_mutex_);

    // TODO: async_read_until() する前に receive_buf_ に delimiter がないか調べる


    std::size_t read_size;
    {
        boost::lock_guard<boost::mutex> read_lock(read_mutex_);
        boost::asio::async_read_until(port_, receive_buf_, delimiter,
                                      boost::bind(&SerialPort::asyncReadCallback, this,
                                                  boost::ref(read_size), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        dl_timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
        dl_timer_.async_wait(boost::bind(&SerialPort::timeoutCallback, this, boost::asio::placeholders::error));
        io_srv_.reset();
        io_srv_.run();  // will block until async callbacks are finished
    }

//        if (read_size == 0)  // バッファに残っている場合があるので read_size での判断はダメ
    if (receive_buf_.size() == 0)  // empty() メンバ関数はない
    {
        buffer.clear();
	std::cout << "  \n Returning empty";
        return buffer.size();
    }

    // read_until() はデリミタ以降の数文字も一緒に読む場合があるので分割する
    std::string recv_buf(boost::asio::buffer_cast<const char*>(receive_buf_.data()), receive_buf_.size());
#if 0 // デリミタが複数文字の場合に is_any_of() の1文字で区切ってしまうのでダメ
    std::vector<std::string> split_str;
    boost::split(split_str, recv_buf, boost::is_any_of(delimiter));
//        boost::split(split_str, recv_buf, boost::is_equal(delimiter));  // コンパイルエラー
//        boost::split(split_str, recv_buf, boost::bind(std::equal_to<std::string>(), delimiter));  // コンパイルエラー
    buffer.assign(split_str.front());
    receive_buf_.consume(buffer.size() + 1);  // デリミタまでのバッファを消費する
//        boost::trim_left_if(buffer, boost::is_any_of(delimiter));  // 先頭に残ったデリミタを削除→デリミタに対して split() は空文字列を返すので不要
#else
    std::size_t delim_pos = recv_buf.find(delimiter, 0);
    if (delim_pos != std::string::npos)  // デリミタを発見
    {
        buffer.assign(recv_buf, 0, delim_pos);
        receive_buf_.consume(buffer.size() + delimiter.size());  // デリミタまでのバッファを消費する
    }
    else  // デリミタまで受信していない
    {
        buffer.clear();
    }
#endif
    return buffer.size();
}

#if 0
int SerialPort::read(char *buffer, int len, boost::system::error_code &err_code)
{
    size_t num_read = 0;
    boost::asio::deadline_timer timeout(io_srv_);
    while (num_read < len)
    {
        size_t async_num_read = 0;
        //std::cout << "i";
        timeout.expires_from_now(boost::posix_time::milliseconds(100));
        timeout.async_wait(boost::bind(&SerialPort::timeoutCallback, this, boost::ref(port_), boost::asio::placeholders::error));
        boost::asio::async_read(port_, boost::asio::buffer(&buffer[num_read], len - num_read), boost::asio::transfer_at_least(len - num_read), boost::bind(&SerialPort::asyncReadCallback, this, boost::ref(timeout), boost::ref(async_num_read), _1, _2));
        io_srv_.run();
        //std::cout << "/o ";
        if (async_num_read == 0)
            break;
        num_read += async_num_read;
    }
    io_srv_.reset();
    //std::cout << "\nread: " << std::string(buffer, num_read) << std::endl;
    return num_read;
}

int SerialPort::readLine(char *buffer, int len, boost::system::error_code &err_code)
{
    size_t num_read = 0;
    boost::asio::deadline_timer timeout(io_srv_);
    while (num_read < len)
    {
        size_t async_num_read = 0;
        timeout.expires_from_now(boost::posix_time::milliseconds(100));
        timeout.async_wait(boost::bind(&SerialPort::timeoutCallback, this, boost::ref(port_), boost::asio::placeholders::error));
        boost::asio::async_read(port_, boost::asio::buffer(&buffer[num_read], 1), boost::asio::transfer_at_least(1), boost::bind(&SerialPort::asyncReadCallback, this, boost::ref(timeout), boost::ref(async_num_read), _1, _2));
        io_srv_.run();
        if (async_num_read == 0)
            break;
        else if (buffer[num_read] == '\n')
            break;
        num_read += async_num_read;
    }
    io_srv_.reset();
    return num_read;
}
#endif

void SerialPort::timeoutCallback(const boost::system::error_code &err_code)
{
    if (!err_code)  // エラーでない場合はタイムアウトしている
    {
        port_.cancel();  // asyncReadCallback() にエラーを渡す
//            std::cout << "INFO: SerialPort read timeout." << std::endl;
    }
}

void SerialPort::asyncReadCallback(std::size_t &read_size, const boost::system::error_code &err_code, const std::size_t bytes_transferred)
{
    if (err_code || bytes_transferred == 0)  // 受信失敗
    {
        read_size = 0;
    }
    else  // 受信成功
    {
        dl_timer_.cancel();  // タイムアウトをキャンセルして timeoutCallback() にエラーを渡す
        read_size = bytes_transferred;
    }
}
