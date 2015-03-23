#include <iostream>
#include <string.h>
#include <Leap/Leap.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace Leap;
using boost::asio::ip::udp;

struct LeapFrameInfo
{
    int64_t id = -1;
    int64_t ts = -1;
    
    float height = 250.0;
    float roll = 0.0;
    float pitch = 0.0;
    
    bool leftIndexTap = false;
};

class LeapListener;

class LeapNetworkProvider
{
private:
    LeapListener & leap_;
    
    boost::asio::io_service sockio_;
    udp::endpoint ep_;
    udp::socket sock_;
    
    boost::asio::io_service timerio_;
    boost::asio::deadline_timer timer_;
    
    mutable std::mutex mutex_;
    LeapFrameInfo lastFrame_;
    
public:
    LeapNetworkProvider(LeapListener &);
    
    void setFrame(LeapFrameInfo const&);
    
    void sendLastFrame();
};

class LeapListener : public Listener
{
private:
    LeapNetworkProvider net_;
    
    bool leftIndexTap_ = false;
    
public:
    LeapListener() : net_(*this) {}
    
    void clearGestures();
    
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
};

LeapNetworkProvider::LeapNetworkProvider(LeapListener & leap)
: leap_(leap)
, timerio_()
, timer_(timerio_, boost::posix_time::millisec(35))
, sockio_()
, sock_(sockio_)
{
    udp::resolver resolver(sockio_);
    udp::resolver::query query(udp::v4(), "localhost", "45002");
    ep_ = *resolver.resolve(query);
    
    sock_.open(udp::v4());
    
    timer_.async_wait(boost::bind(&LeapNetworkProvider::sendLastFrame, this));
    
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &timerio_));
}

void LeapNetworkProvider::setFrame(const LeapFrameInfo & frame)
{
    std::unique_lock<std::mutex> lk_(mutex_);
    lastFrame_ = frame;
}

void LeapNetworkProvider::sendLastFrame()
{
    boost::array<LeapFrameInfo, 1> send_buf;
    
    {
        std::unique_lock<std::mutex> lk_(mutex_);
        send_buf[0] = lastFrame_;
    }
    
    sock_.send_to(boost::asio::buffer(send_buf), ep_);
    
    leap_.clearGestures();
    
    timer_.expires_at(timer_.expires_at() + boost::posix_time::millisec(35));
    timer_.async_wait(boost::bind(&LeapNetworkProvider::sendLastFrame, this));
}

void LeapListener::clearGestures()
{
    leftIndexTap_ = false;
}

void LeapListener::onInit(const Controller& controller)
{
    std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Controller& controller)
{
    std::cout << "Connected" << std::endl;
    controller.enableGesture(Gesture::TYPE_KEY_TAP);
    controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);
}

void LeapListener::onDisconnect(const Controller& controller)
{
    std::cout << "Disconnected" << std::endl;
}

void LeapListener::onExit(const Controller& controller)
{
    std::cout << "Exited" << std::endl;
}

void LeapListener::onFrame(const Controller& controller)
{
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    LeapFrameInfo info;
    
    info.id = frame.id();
    info.ts = frame.timestamp();
    
    HandList hands = frame.hands();
    
    if (hands.count() == 2)
    {
        const Hand left = hands.leftmost();
        const Hand right = hands.rightmost();
        
        info.height = left.palmPosition().y;
        
        info.roll = right.palmNormal().roll();
        info.pitch = right.direction().pitch();
        
        const GestureList gestures = frame.gestures();
        for (int g = 0; g < gestures.count(); ++g)
        {
            Gesture gesture = gestures[g];
            
            switch (gesture.type())
            {
                case Gesture::TYPE_KEY_TAP:
                {
                    KeyTapGesture tap = gesture;
                    
                    if (tap.hands().leftmost().id() == left.id() &&
                        tap.pointable() == left.fingers().fingerType(Finger::TYPE_INDEX)[0])
                    {
                        leftIndexTap_ = true;
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
    
    info.leftIndexTap = leftIndexTap_;
    
    net_.setFrame(info);
}

void LeapListener::onServiceConnect(const Controller& controller)
{
    std::cout << "Service Connected" << std::endl;
}

void LeapListener::onServiceDisconnect(const Controller& controller)
{
    std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv)
{
    // Create a sample listener and controller
    LeapListener listener;
    Controller controller;
    
    // Have the sample listener receive events from the controller
    controller.addListener(listener);
    
    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
    
    // Remove the sample listener when done
    controller.removeListener(listener);
    
    return 0;
}
