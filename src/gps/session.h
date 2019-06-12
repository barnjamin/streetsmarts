#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "ublox/Message.h"
#include "ublox/message/NavPvt.h"
#include "ublox/frame/UbloxFrame.h"
#include "../config.h"
#include "ntrip.h"

class Session 
{
    using InMessage =
        ublox::Message<
            comms::option::ReadIterator<const std::uint8_t*>,
            comms::option::Handler<Session> // Dispatch to this object
        >;

    using OutBuffer = std::vector<std::uint8_t>;
    using OutMessage =
        ublox::Message<
            comms::option::IdInfoInterface,
            comms::option::WriteIterator<std::back_insert_iterator<OutBuffer> >,
            comms::option::LengthInfoInterface
        >;

    using InNavPvt = ublox::message::NavPvt<InMessage>;

public:
    Session(boost::asio::io_service& io, Config conf);
    ~Session();

    bool start();

    void handle(InNavPvt& msg);

    void handle(InMessage& msg);

private:

    using AllInMessages =
        std::tuple<
            InNavPvt
        >;

    using Frame = ublox::frame::UbloxFrame<InMessage, AllInMessages>;

    using SerialPort = boost::asio::serial_port;

    void performRead();
    void processInputData();
    void sendSolPoll();
    void sendMessage(const OutMessage& msg);
    void configureUbxOutput();

    SerialPort m_serial;
    boost::asio::deadline_timer m_pollTimer;
    std::string m_device;
    boost::array<std::uint8_t, 512> m_inputBuf;
    std::vector<std::uint8_t> m_inData;
    Frame m_frame;

    NtripClient ntrip;
};
