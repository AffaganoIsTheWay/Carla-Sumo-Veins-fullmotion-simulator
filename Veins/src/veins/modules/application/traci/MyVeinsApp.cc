//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/MyVeinsApp.h"
#include <cmath>
#include <zmq.hpp>

using namespace veins;

static zmq::context_t ctx;
zmq::socket_t sock(ctx, zmq::socket_type::pub);
zmq::socket_t sock_web(ctx, zmq::socket_type::pub);


Define_Module(veins::MyVeinsApp);

void MyVeinsApp::initialize(int stage){
    sock.connect("tcp://10.196.36.90:5590");
    sock_web.connect("tcp://127.0.0.1:5600");
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        if (traciVehicle->getTypeId() == "violator") {
                scheduleAt(simTime() + 1.0, new cMessage("beacon_timer"));
        }
    }
}

void MyVeinsApp::onWSM(BaseFrame1609_4* frame){
    VBmessage* wsm = dynamic_cast<VBmessage*>(frame);
    if (!wsm) return;

    Coord myPos = mobility->getPositionAt(simTime());
    double mySpeed = traciVehicle->getSpeed();
    double myAngle = traciVehicle->getAngle();

    bool danger = checkTrajectoryCollision(
    myPos.x, myPos.y, mySpeed, myAngle,
    wsm->getPosX(), wsm->getPosY(),
    wsm->getSpeed(), wsm->getAngle()
    );

    if (danger) {
        EV << "COLLISION ALERT: Stopping Vehicle!" << endl;
        //traciVehicle->slowDown(0, 2.0);
        sock.send(zmq::str_buffer("1"), zmq::send_flags::none);
        sock_web.send(zmq::str_buffer("1"), zmq::send_flags::none);
    } else {
        sock.send(zmq::str_buffer("0"), zmq::send_flags::none);
        sock_web.send(zmq::str_buffer("0"), zmq::send_flags::none);
    }
}

void MyVeinsApp::handleSelfMsg(cMessage* msg){
    if (std::string(msg->getName()) == "beacon_timer") {

        VBmessage* wsm = new VBmessage();

        Coord pos = mobility->getPositionAt(simTime());
        wsm->setPosX(pos.x);
        wsm->setPosY(pos.y);
        wsm->setSpeed(traciVehicle->getSpeed());
        wsm->setAngle(traciVehicle->getAngle());

        populateWSM(wsm);
        sendDown(wsm);

        scheduleAt(simTime() + 0.1, msg);
    } else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

bool MyVeinsApp::checkTrajectoryCollision(double nX, double nY, double nSpeed, double nAngle,
                                      double vX, double vY, double vSpeed, double vAngle) {

    double dist = sqrt(pow(nX - vX, 2) + pow(nY - vY, 2));
    if (dist > 25.0) return false;

    double nVx = nSpeed * cos(nAngle - 90);
    double nVy = nSpeed * sin(nAngle - 90);
    double vVx = vSpeed * cos(vAngle - 90);
    double vVy = vSpeed * sin(vAngle - 90);

    double dx = vX - nX;
    double dy = vY - nY;
    double dVx = nVx - vVx;
    double dVy = nVy - vVy;

    double tX, tY;

    if (std::abs(dVx) > 1e-9 || std::abs(dVy) > 1e-9) {
        tX = dx / dVx;
        tY = dy / dVy;
    } else {
        return false;
    }

    /*
    if((tX>0 || tY >0) && std::abs(tX - tY) > 1e-9){
        double ydiffn = 2.5/nVy;
        double ydiffv = 2.5/vVy;
        double xdiffn = 2.5/nVx;
        double xdiffv = 2.5/vVx;

        double n_enterx = tX - xdiffn;
        double n_entery = tY - ydiffn;
        double v_enterx = tX - xdiffv;
        double v_entery = tY - ydiffv;

        double n_exitx = tX + xdiffn;
        double n_exity = tY + ydiffn;
        double v_exitx = tX + xdiffv;
        double v_exity = tY + ydiffv;

        double overlap_startx = std::max(n_enterx, v_enterx);
        double overlap_endx   = std::min(n_exitx, v_exitx);
        double overlap_starty = std::max(n_entery, v_entery);
        double overlap_endy   = std::min(n_exity, v_exity);

        if ((overlap_startx <= overlap_endx && overlap_endx > 0) && (overlap_starty <= overlap_endy && overlap_endy > 0)) return true;
    }
    */

    if((tX>0 || tY >0) && std::abs(tX - tY) > 1e-9) return true;

    return false;
}
