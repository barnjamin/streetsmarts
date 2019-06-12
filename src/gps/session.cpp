#include "session.h"

#include <thread>
#include <curl/curl.h>
#include <iostream>
#include <cassert>

#include "ublox/message/CfgPrtUsb.h"
#include "ublox/message/NavPvtPoll.h"
#include "ublox/message/NavPvt.h"
#include "comms/units.h"
#include "ntrip.h"

Session::Session(boost::asio::io_service& io, Config conf)
  : m_serial(io),
    m_pollTimer(io),
    ntrip(conf.ntrip_host, conf.ntrip_port,
            conf.ntrip_mount, conf.ntrip_user, conf.ntrip_pw),
    m_device("/dev/ttyACM1")//TODO
{
}

Session::~Session() = default;

bool Session::start()
{
    boost::system::error_code ec;
    m_serial.open(m_device, ec);
    if (ec) {
        std::cerr << "ERROR: Failed to open " << m_device << std::endl;
        return false;
    }

    m_serial.set_option(SerialPort::baud_rate(115200));
    m_serial.set_option(SerialPort::character_size(8));
    m_serial.set_option(SerialPort::parity(SerialPort::parity::none));
    m_serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
    m_serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

    configureUbxOutput();

    ntrip.start(m_serial);

    sendSolPoll();
    performRead();

    return true;
}

void Session::handle(InNavPvt& msg)
{
    using gfix = ublox::field::GpsFix<>;
    std::cout << "POS: lat=" << msg.field_lat().value() <<
        "; lon=" << msg.field_lon().value() <<
        "; fix=" << gfix::valueName(msg.field_fixType().value()) << std::endl;
}

void Session::handle(InMessage& msg)
{
    static_cast<void>(msg); // ignore
}

void Session::performRead()
{
    m_serial.async_read_some(
        boost::asio::buffer(m_inputBuf),
        [this](const boost::system::error_code& ec, std::size_t bytesCount)
        {
            if (ec == boost::asio::error::operation_aborted) {
                return;
            }

            //if (ec) {
            //    std::cerr << "ERROR: read failed with message: " << ec.message() << std::endl;
            //}

            auto dataBegIter = m_inputBuf.begin();
            auto dataEndIter = dataBegIter + bytesCount;
            m_inData.insert(m_inData.end(), dataBegIter, dataEndIter);
            processInputData();
            performRead();
        });
}

void Session::processInputData()
{
    std::size_t consumed = 0U;
    while (consumed < m_inData.size()) {
        // Smart pointer to the message object.
        Frame::MsgPtr msgPtr;

        // Type of the message interface class
        using MsgType = Frame::MsgPtr::element_type;

        // Get the iterator for reading
        auto begIter = comms::readIteratorFor<MsgType>(&m_inData[0] + consumed);
        auto iter = begIter;

        // Do the read
        auto es = m_frame.read(msgPtr, iter, m_inData.size() - consumed);
        if (es == comms::ErrorStatus::NotEnoughData) {
            break; // Not enough data in the buffer, stop processing
        }

        if (es == comms::ErrorStatus::ProtocolError) {
            // Something is not right with the data, remove one character and try again
            ++consumed;
            continue;
        }

        if (es == comms::ErrorStatus::Success) {
            assert(msgPtr); // If read is successful, msgPtr is expected to hold a valid pointer
            msgPtr->dispatch(*this); // Dispatch message for handling
        }

        // The iterator for reading has been advanced, update the difference
        consumed += std::distance(begIter, iter);
    }

    m_inData.erase(m_inData.begin(), m_inData.begin() + consumed);    
}

void Session::sendSolPoll()
{
    using OutNavPvtPoll = ublox::message::NavPvtPoll<OutMessage>;
    sendMessage(OutNavPvtPoll());

    m_pollTimer.expires_from_now(boost::posix_time::seconds(1));
    m_pollTimer.async_wait(
        [this](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted) {
                return;
            }

            sendSolPoll();
        });
}


void Session::sendMessage(const OutMessage& msg)
{
    OutBuffer buf;
    buf.reserve(m_frame.length(msg)); // Reserve enough space
    auto iter = std::back_inserter(buf);
    auto es = m_frame.write(msg, iter, buf.max_size());
    if (es == comms::ErrorStatus::UpdateRequired) {
        auto* updateIter = &buf[0];
        es = m_frame.update(updateIter, buf.size());
    }
    static_cast<void>(es);
    assert(es == comms::ErrorStatus::Success); // do not expect any error

    while (!buf.empty()) {
        boost::system::error_code ec;
        auto count = m_serial.write_some(boost::asio::buffer(buf), ec);

        if (ec) {
            std::cerr << "ERROR: write failed with message: " << ec.message() << std::endl;
            m_serial.get_io_service().stop();
            return;
        }

        buf.erase(buf.begin(), buf.begin() + count);
    }
}

void Session::configureUbxOutput()
{
    using OutCfgPrtUsb = ublox::message::CfgPrtUsb<OutMessage>;

    OutCfgPrtUsb msg;
    auto& outProtoMaskField = msg.field_outProtoMask();
    using OutProtoMaskField = typename std::decay<decltype(outProtoMaskField)>::type;
    outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outUbx, true);
    outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outNmea, false);

    auto& inProtoMaskField = msg.field_inProtoMask();
    using InProtoMaskField = typename std::decay<decltype(inProtoMaskField)>::type;
    inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inUbx, true);
    inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inNmea, false);

    sendMessage(msg);
}
