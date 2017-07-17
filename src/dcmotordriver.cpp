//
//  Copyright (c) 2017 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44bandit.
//
//  p44ayabd is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  p44ayabd is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with p44ayabd. If not, see <http://www.gnu.org/licenses/>.
//

#include "dcmotordriver.hpp"

#include "consolekey.hpp"
#include "application.hpp"

using namespace p44;

#define BANDIT_COMMPARAMS "1200,7,E,2"


#pragma mark - BanditComm

BanditComm::BanditComm(MainLoop &aMainLoop) :
	inherited(aMainLoop),
  banditState(banditstate_idle),
  endOnHandshake(false),
  timeoutTicket(0)
{
}


BanditComm::~BanditComm()
{
  stop();
}


void BanditComm::setConnectionSpecification(const char *aConnectionSpec, uint16_t aDefaultPort, const char *aRtsDtrOutput, const char *aCtsDsrDcdInput)
{
  LOG(LOG_DEBUG, "BanditComm::setConnectionSpecification: %s", aConnectionSpec);
  // setup serial
  inherited::setConnectionSpecification(aConnectionSpec, aDefaultPort, BANDIT_COMMPARAMS);
  // setup handshake lines
  rtsDtrOutput = DigitalIoPtr(new DigitalIo(aRtsDtrOutput, true, false));
  ctsDsrDcdInput = DigitalIoPtr(new DigitalIo(aCtsDsrDcdInput, false, false));
  // open serial device
  ErrorPtr err = establishConnection();
  if (!Error::isOK(err)) {
    LOG(LOG_ERR, "Cannot establish BANDIT connection: %s", err->description().c_str());
    return;
  }
  // connection ok, set handler
  setReceiveHandler(boost::bind(&BanditComm::receiveHandler, this, _1));
  // set handshake line monitor
  ctsDsrDcdInput->setInputChangedHandler(boost::bind(&BanditComm::handshakeChanged, this, _1), 0, 100*MilliSecond);
}


void BanditComm::stop()
{
  responseCB = NULL;
  banditState = banditstate_idle;
  MainLoop::currentMainLoop().cancelExecutionTicket(timeoutTicket);
  rtsDtrOutput->off();
}


void BanditComm::end(ErrorPtr aError, string aData)
{
  BanditResponseCB c = responseCB;
  stop();
  if (c) {
    c(aData, aError);
  }
}



void BanditComm::handshakeChanged(bool aNewState)
{
  LOG(LOG_INFO, "Handshake line changed to %d", aNewState);
  if (banditState==banditstate_receivewait) {
    // set handshake line now
    if (aNewState) {
      LOG(LOG_INFO, "Input handshake got active -> starting receive");
      startReceive();
    }
  }
  else if (banditState==banditstate_receiving) {
    if (endOnHandshake && aNewState==false) {
      LOG(LOG_INFO, "Input handshake got inactive -> assume all data received");
      end(ErrorPtr(), data);
    }
  }
}


#define RECEIVE_TIMEOUT (5*Second)

void BanditComm::receiveHandler(ErrorPtr aError)
{
  string d;
  ErrorPtr err = receiveAndAppendToString(d);
  if (Error::isOK(err)) {
    if (banditState==banditstate_receiving) {
      // accumulate
      MainLoop::currentMainLoop().rescheduleExecutionTicket(timeoutTicket, RECEIVE_TIMEOUT);
      LOG(LOG_INFO, "Received Data: %s", d.c_str());
      data.append(d);
    }
    else {
      LOG(LOG_NOTICE, "Received stray Data: %s", d.c_str());
      // stray data
    }
  }
  else {
    if (banditState!=banditstate_idle) {
      // report error and stop
      end(aError);
    }
  }
}


void BanditComm::startReceive()
{
  banditState = banditstate_receiving;
  // set handshake line right away
  rtsDtrOutput->on();
  // set timeout
  MainLoop::currentMainLoop().executeTicketOnce(timeoutTicket, boost::bind(&BanditComm::timeout, this), RECEIVE_TIMEOUT);
}



void BanditComm::timeout()
{
  LOG(LOG_NOTICE, "Timeout -> stopping");
  if (data.size()>0) {
    end(ErrorPtr(), data);
  }
  else {
    end(TextError::err("Timeout while waiting for data"));
  }
}


void BanditComm::receive(BanditResponseCB aResponseCB, bool aHandShakeOnStart, bool aWaitForHandshake, bool aEndOnHandshake)
{
  stop();
  endOnHandshake = aEndOnHandshake;
  responseCB = aResponseCB;
  data.clear();
  if (aHandShakeOnStart) {
    rtsDtrOutput->on();
  }
  if (aWaitForHandshake) {
    banditState = banditstate_receivewait;
  }
  else {
    startReceive();
  }
}


#define BYTE_TIME (Second/1200*11)
#define SEND_FINISH_DELAY (BYTE_TIME*4)

void BanditComm::send(StatusCB aStatusCB, string aData, bool aEnableHandshake)
{
  // FIXME: send line per line, maybe check handshake line, callback only when finished
  if (aEnableHandshake)
    rtsDtrOutput->on();
  MainLoop::currentMainLoop().executeTicketOnce(timeoutTicket, boost::bind(&BanditComm::dataSent, this, aStatusCB), BYTE_TIME*aData.size()+SEND_FINISH_DELAY);
  sendString(aData);
}


void BanditComm::dataSent(StatusCB aStatusCB)
{
  rtsDtrOutput->off();
  if (aStatusCB) aStatusCB(ErrorPtr());
}

