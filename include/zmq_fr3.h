#pragma once
#ifndef __ZMQ_FR3_H
#define __ZMQ_FR3_H

#include "zmq.hpp"
// using namespace std;













class CZmq_fr3
{

public:
    CZmq_fr3();
    virtual ~CZmq_fr3();

    void zmq_send(float tmp_Buffer[]);
    void zmq_recv();


    double _zmq_qdot[7];
    double _zmq_q[7];

    zmq::context_t context{ 14 };
    zmq::context_t context2{ 7 };
    zmq::socket_t socket{ context, zmq::socket_type::pull };
    zmq::socket_t socket2{ context2, zmq::socket_type::push };
    // socket.connect("tcp://161.122.114.37:5557");
    zmq::message_t reply{};
	std::array<float, 14> Buffer; // mat3x3 * 4 = 36	+ 1
    std::array<float, 14> Buffer_Robot; // mat3x3 * 4 = 36	+ 1
    zmq::message_t request{};

private:
};

#endif