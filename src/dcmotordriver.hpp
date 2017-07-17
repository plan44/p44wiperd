//
//  Copyright (c) 2017 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44wiperd.
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

#ifndef __p44wiperd__dcmotordriver__
#define __p44wiperd__dcmotordriver__

#include "p44utils_common.hpp"

#include "serialcomm.hpp"
#include "digitalio.hpp"

using namespace std;

namespace p44 {


  class BanditComm;


  typedef boost::function<void (const string &aResponse, ErrorPtr aError)> BanditResponseCB;


  typedef boost::intrusive_ptr<BanditComm> BanditCommPtr;
  class BanditComm : public SerialComm
  {
    typedef SerialComm inherited;

    DigitalIoPtr rtsDtrOutput;
    DigitalIoPtr ctsDsrDcdInput;

    BanditResponseCB responseCB;

    enum {
      banditstate_idle,
      banditstate_receivewait,
      banditstate_receiving
    } banditState;

    string data;
    bool endOnHandshake;
    long timeoutTicket;

  public:

    BanditComm(MainLoop &aMainLoop);
    virtual ~BanditComm();

    /// set the connection parameters to connect to BANDIT controller
    /// @param aConnectionSpec serial device path (/dev/...) or host name/address[:port] (1.2.3.4 or xxx.yy)
    /// @param aDefaultPort default port number for TCP connection (irrelevant for direct serial device connection)
    void setConnectionSpecification(const char *aConnectionSpec, uint16_t aDefaultPort, const char *aRtsDtrOutput, const char *aCtsDsrDcdInput);


    /// stop actions, no callback
    void stop();

    /// receive data from bandit
    /// @param aResponseCB will be called after receiving a complete transmission from Bandit, or on error
    /// @param aEnableHandshake if set, handshake line will be set before starting to receive or waiting for handshake input
    /// @param aWaitForHandshake if set, receiving will not start before input handshake goes active
    /// @param aEndOnHandshake if set, receiving ends when input handshake goes inactive
    void receive(BanditResponseCB aResponseCB, bool aEnableHandshake, bool aWaitForHandshake, bool aEndOnHandshake);

    /// receive data from bandit
    /// @param aData data to send
    /// @param aStatusCB will be called after transmission to Bandit is complete, or on error
    /// @param aEnableHandshake if set, handshake line will be set before sending
    void send(StatusCB aStatusCB, string aData, bool aEnableHandshake);


  protected:


  private:

    void receiveHandler(ErrorPtr aError);
    void end(ErrorPtr aError, string aData="");
    void timeout();
    void handshakeChanged(bool aNewState);
    void startReceive();
    void dataSent(StatusCB aStatusCB);

  };



} // namespace p44

#endif /* defined(__p44wiperd__dcmotordriver__) */
