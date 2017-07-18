//
//  Copyright (c) 2017 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44wiperd.
//
//  pixelboardd is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  pixelboardd is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with pixelboardd. If not, see <http://www.gnu.org/licenses/>.
//

#include "application.hpp"

#include "jsoncomm.hpp"

#include "dcmotordriver.hpp"


using namespace p44;

#define MAINLOOP_CYCLE_TIME_uS 10000 // 10mS
#define DEFAULT_LOGLEVEL LOG_NOTICE


/// Main program for plan44.ch P44-DSB-DEH in form of the "vdcd" daemon)
class P44WiperD : public CmdLineApp
{
  typedef CmdLineApp inherited;

  // API Server
  SocketCommPtr apiServer;

  DcMotorDriverPtr motorDriver;
  DigitalIoPtr zeroPosInput;

  DigitalIoPtr movementInput;

  ButtonInputPtr button;
  IndicatorOutputPtr greenLed;
  IndicatorOutputPtr redLed;


  MLMicroSeconds starttime;

public:

  P44WiperD() :
    starttime(MainLoop::now())
  {
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
      "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "jsonapiport",    true,  "port;server port number for JSON API (default=none)" },
      { 0  , "jsonapinonlocal",false, "allow JSON API from non-local clients" },
      { 'l', "loglevel",       true,  "level;set max level of log message detail to show on stdout" },
      { 0  , "errlevel",       true,  "level;set max level for log messages to go to stderr as well" },
      { 0  , "dontlogerrors",  false, "don't duplicate error messages (see --errlevel) on stdout" },
      { 0  , "poweroutput",    true,  "analog output pinspec; analog output that drives the motor power" },
      { 0  , "cwoutput",       true,  "output pinspec; digital output for indicating clockwise operation" },
      { 0  , "ccwoutput",      true,  "output pinspec; digital output for indicating counter clockwise operation" },
      { 0  , "zeroposinput",   true,  "input pinspec; digital input indicating zero position" },
      { 0  , "movementinput",  true,  "input pinspec; digital input indicating movement" },
      { 0  , "button",         true,  "input pinspec; device button" },
      { 0  , "greenled",       true,  "output pinspec; green device LED" },
      { 0  , "redled",         true,  "output pinspec; red device LED" },
      { 0  , "initialpower",   true,  "float;initial power, 0..100" },
      { 0  , "power",          true,  "float;end-of-rampp power, 0..100" },
      { 0  , "initialdir",     true,  "int;initial direction -1,0,1" },
      { 0  , "dir",            true,  "int;direction -1,0,1" },
      { 0  , "exp",            true,  "float;exponent for ramp, 1=linear" },
      { 0  , "fullramp",       true,  "float;seconds for full ramp" },
      { 'h', "help",           false, "show this text" },
      { 0, NULL } // list terminator
    };

    // parse the command line, exits when syntax errors occur
    setCommandDescriptors(usageText, options);
    parseCommandLine(argc, argv);

    if ((numOptions()<1) || numArguments()>0) {
      // show usage
      showUsage();
      terminateApp(EXIT_SUCCESS);
    }

    // build objects only if not terminated early
    if (!isTerminated()) {
      int loglevel = DEFAULT_LOGLEVEL;
      getIntOption("loglevel", loglevel);
      SETLOGLEVEL(loglevel);
      int errlevel = LOG_ERR; // testing by default only reports to stdout
      getIntOption("errlevel", errlevel);
      SETERRLEVEL(errlevel, !getOption("dontlogerrors"));

      // - create button input
      button = ButtonInputPtr(new ButtonInput(getOption("button","missing")));
      button->setButtonHandler(boost::bind(&P44WiperD::buttonHandler, this, _1, _2, _3), true, true);
      // - create LEDs
      greenLed = IndicatorOutputPtr(new IndicatorOutput(getOption("greenled","missing")));
      redLed = IndicatorOutputPtr(new IndicatorOutput(getOption("redled","missing")));

      // - create motor driver
      motorDriver = DcMotorDriverPtr(new DcMotorDriver(
        getOption("poweroutput","missing"),
        getOption("cwoutput","missing"),
        getOption("ccwoutput","missing")
      ));
      // - create zero position input
      zeroPosInput = DigitalIoPtr(new DigitalIo(getOption("zeroposinput","missing"), false, false));
      zeroPosInput->setInputChangedHandler(boost::bind(&P44WiperD::zeroPosHandler, this, _1), 40*MilliSecond, 0);

      // movement detector input
      movementInput = DigitalIoPtr(new DigitalIo(getOption("movementInput","missing"), false, false));
      movementInput->setInputChangedHandler(boost::bind(&P44WiperD::movementHandler, this, _1), 0, 0);

      // - create and start API server and wait for things to happen
      string apiport;
      if (getStringOption("jsonapiport", apiport)) {
        apiServer = SocketCommPtr(new SocketComm(MainLoop::currentMainLoop()));
        apiServer->setConnectionParams(NULL, apiport.c_str(), SOCK_STREAM, AF_INET);
        apiServer->setAllowNonlocalConnections(getOption("jsonapinonlocal"));
        apiServer->startServer(boost::bind(&P44WiperD::apiConnectionHandler, this, _1), 3);
      }


    } // if !terminated
    // app now ready to run (or cleanup when already terminated)
    return run();
  }


  void buttonHandler(bool aState, bool aHasChanged, MLMicroSeconds aTimeSincePreviousChange)
  {
    // %%% Nop for now
    if (aHasChanged && aState) {
      motorDriver->stop();
    }
  }


  void zeroPosHandler(bool aNewState)
  {
    // %%% Nop for now
    LOG(LOG_NOTICE, "Zero position signal = %d", aNewState);
  }


  void movementHandler(bool aNewState)
  {
    // %%% Nop for now
    LOG(LOG_NOTICE, "Movement signal = %d", aNewState);
  }





  virtual void initialize()
  {
    string s;
    double initialPower = 0;
    if (getStringOption("initialpower",s)) {
      sscanf(s.c_str(),"%lf", &initialPower);
    }
    double exp = 1;
    if (getStringOption("exp",s)) {
      sscanf(s.c_str(),"%lf", &exp);
    }
    if (getStringOption("power",s)) {
      double power = 0;
      sscanf(s.c_str(),"%lf", &power);
      int dir = 0;
      getIntOption("dir", dir);
      int initialdir = dir;
      getIntOption("initialdir", initialdir);
      double ramp = 2; // 2 seconds default
      if (getStringOption("fullramp",s)) {
        sscanf(s.c_str(),"%lf", &ramp);
      }
      // start
      motorDriver->rampToPower(initialPower, initialdir, 0, exp);
      // now run motor this way
      motorDriver->rampToPower(power, dir, ramp, exp, boost::bind(&P44WiperD::rampComplete, this, _1, _2, _3));
    }
  }


  void rampComplete(double aCurrentPower, int aDirection, ErrorPtr aError)
  {
    if (Error::isOK(aError)) {
      // print data to stdout
      LOG(LOG_NOTICE, "Ramp complete, power=%.2f%%, direction=%d", aCurrentPower, aDirection);
    }
    else {
      LOG(LOG_ERR, "Error receiving data: %s", aError->description().c_str());
    }
    terminateAppWith(aError);
  }





  SocketCommPtr apiConnectionHandler(SocketCommPtr aServerSocketComm)
  {
    JsonCommPtr conn = JsonCommPtr(new JsonComm(MainLoop::currentMainLoop()));
    conn->setMessageHandler(boost::bind(&P44WiperD::apiRequestHandler, this, conn, _1, _2));
    conn->setClearHandlersAtClose(); // close must break retain cycles so this object won't cause a mem leak
    return conn;
  }


  void apiRequestHandler(JsonCommPtr aConnection, ErrorPtr aError, JsonObjectPtr aRequest)
  {
    // Decode mg44-style request (HTTP wrapped in JSON)
    if (Error::isOK(aError)) {
      LOG(LOG_INFO,"API request: %s", aRequest->c_strValue());
      JsonObjectPtr o;
      o = aRequest->get("method");
      if (o) {
        string method = o->stringValue();
        string uri;
        o = aRequest->get("uri");
        if (o) uri = o->stringValue();
//        JsonObjectPtr data;
//        bool upload = false;
//        bool action = (method!="GET");
//        // check for uploads
//        string uploadedfile;
//        if (aRequest->get("uploadedfile", o)) {
//          uploadedfile = o->stringValue();
//          upload = true;
//          action = false; // other params are in the URI, not the POSTed upload
//        }
//        if (action) {
//          // JSON data is in the request
//          data = aRequest->get("data");
//        }
//        else {
//          // URI params is the JSON to process
//          data = aRequest->get("uri_params");
//          if (data) action = true; // GET, but with query_params: treat like PUT/POST with data
//          if (upload) {
//            // move that into the request
//            data->add("uploadedfile", JsonObject::newString(uploadedfile));
//          }
//        }
//        // request elements now: uri and data
//        if (processRequest(uri, data, action, boost::bind(&PixelBoardD::requestHandled, this, aConnection, _1, _2))) {
//          // done, callback will send response and close connection
//          return;
//        }
        // request cannot be processed, return error
        LOG(LOG_ERR,"Invalid JSON request");
        aError = WebError::webErr(404, "No handler found for request to %s", uri.c_str());
      }
      else {
        LOG(LOG_ERR,"Invalid JSON request");
        aError = WebError::webErr(415, "Invalid JSON request format");
      }
    }
    // return error
    requestHandled(aConnection, JsonObjectPtr(), aError);
  }


  void requestHandled(JsonCommPtr aConnection, JsonObjectPtr aResponse, ErrorPtr aError)
  {
    if (!aResponse) {
      aResponse = JsonObject::newObj(); // empty response
    }
    if (!Error::isOK(aError)) {
      aResponse->add("Error", JsonObject::newString(aError->description()));
    }
    LOG(LOG_INFO,"API answer: %s", aResponse->c_strValue());
    aConnection->sendMessage(aResponse);
    aConnection->closeAfterSend();
  }


//  bool processRequest(string aUri, JsonObjectPtr aData, bool aIsAction, RequestDoneCB aRequestDoneCB)
//  {
//    ErrorPtr err;
//    JsonObjectPtr o;
//    if (aUri=="player1" || aUri=="player2") {
//      int side = aUri=="player2" ? 1 : 0;
//      if (aIsAction) {
//        if (aData->get("key", o)) {
//          string key = o->stringValue();
//          // Note: assume keys are already released when event is reported
//          if (key=="left")
//            keyHandler(side, keycode_left, keycode_none);
//          else if (key=="right")
//            keyHandler(side, keycode_right, keycode_none);
//          else if (key=="turn")
//            keyHandler(side, keycode_middleleft, keycode_none);
//          else if (key=="drop")
//            keyHandler(side, keycode_middleright, keycode_none);
//        }
//      }
//      aRequestDoneCB(JsonObjectPtr(), ErrorPtr());
//      return true;
//    }
//    else if (aUri=="board") {
//      if (aIsAction) {
//        PageMode mode = pagemode_controls1; // default to bottom controls
//        if (aData->get("mode", o))
//          mode = o->int32Value();
//        if (aData->get("page", o)) {
//          string page = o->stringValue();
//          gotoPage(page, mode);
//        }
//      }
//      aRequestDoneCB(JsonObjectPtr(), ErrorPtr());
//      return true;
//    }
//    else if (aUri=="page") {
//      // ask each page
//      for (PagesMap::iterator pos = pages.begin(); pos!=pages.end(); ++pos) {
//        if (pos->second->handleRequest(aData, aRequestDoneCB)) {
//          // request will be handled by this page, done for now
//          return true;
//        }
//      }
//    }
//    else if (aUri=="/") {
//      string uploadedfile;
//      string cmd;
//      if (aData->get("uploadedfile", o))
//        uploadedfile = o->stringValue();
//      if (aData->get("cmd", o))
//        cmd = o->stringValue();
//      if (cmd=="imageupload" && displayPage) {
//        ErrorPtr err = displayPage->loadPNGBackground(uploadedfile);
//        gotoPage("display", false);
//        updateDisplay();
//        aRequestDoneCB(JsonObjectPtr(), err);
//        return true;
//      }
//    }
//    return false;
//  }
//
//
//  ErrorPtr processUpload(string aUri, JsonObjectPtr aData, const string aUploadedFile)
//  {
//    ErrorPtr err;
//
//    string cmd;
//    JsonObjectPtr o;
//    if (aData->get("cmd", o)) {
//      cmd = o->stringValue();
//      if (cmd=="imageupload") {
//        displayPage->loadPNGBackground(aUploadedFile);
//        gotoPage("display", false);
//        updateDisplay();
//      }
//      else {
//        err = WebError::webErr(500, "Unknown upload cmd '%s'", cmd.c_str());
//      }
//    }
//    return err;
//  }



};





int main(int argc, char **argv)
{
  // prevent debug output before application.main scans command line
  SETLOGLEVEL(LOG_EMERG);
  SETERRLEVEL(LOG_EMERG, false); // messages, if any, go to stderr
  // create the mainloop
  MainLoop::currentMainLoop().setLoopCycleTime(MAINLOOP_CYCLE_TIME_uS);
  // create app with current mainloop
  static P44WiperD application;
  // pass control
  return application.main(argc, argv);
}
