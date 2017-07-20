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
#include "persistentparams.hpp"

#include "dcmotordriver.hpp"


using namespace p44;

#define MAINLOOP_CYCLE_TIME_uS 10000 // 10mS
#define DEFAULT_LOGLEVEL LOG_NOTICE
#define DEFAULT_DBDIR "/tmp"



// MARK: ===== settings definitions

#define FIELD(ty,f) ((ty*)settings. )


typedef struct {
  const char *fieldName;
  const char *title;
  json_type jsonType;
  size_t offset;
  double min;
  double max;
} SettingsFieldDef;



#define OFFS(fld) offsetof(WiperSettings, fld)
#define FLD(ty,offs) (*((ty*)(((char *)&settings)+offs)))

typedef struct {
  double calibratePower; ///< calibration power [%]
  double calibrateRotationTime; ///< time a full rotation takes at calibration power [Seconds]
  double rezeroSwingAngle; ///< max rezero swing from initial position [degrees]
  double findZeroRamp; ///< full power ramp time during zero position find [Seconds]
  double swingMaxPower; ///< swing max power [%]
  double swingPeriod; ///< swing period [seconds]
  double swingCurveExp; ///< swing power curve exponent, -1.85 is near sine wave
  double midPointAdjustTime; ///< midpoint adjust ramp time [Seconds]
} WiperSettings;


static const SettingsFieldDef settingsFieldDefs[] = {
  {
    .fieldName = "calibratePower",
    .title =  "Motor power for calibration runs [%]",
    .jsonType = json_type_double,
    .offset = OFFS(calibratePower),
    .min = 20,
    .max = 100,
  },
  {
    .fieldName = "calibrateRotationTime",
    .title =  "Time for one full rotation [seconds]",
    .jsonType = json_type_double,
    .offset = OFFS(calibrateRotationTime),
    .min = 1,
    .max = 10,
  },
  {
    .fieldName = "rezeroSwingAngle",
    .title =  "Max angle to move left or right for rezeroing [degrees]",
    .jsonType = json_type_double,
    .offset = OFFS(rezeroSwingAngle),
    .min = 30,
    .max = 200,
  },
  {
    .fieldName = "findZeroRamp",
    .title =  "Full power ramp time during zero position find [seconds]",
    .jsonType = json_type_double,
    .offset = OFFS(findZeroRamp),
    .min = 0,
    .max = 1,
  },
  {
    .fieldName = "swingMaxPower",
    .title =  "Swing max power [%]",
    .jsonType = json_type_double,
    .offset = OFFS(swingMaxPower),
    .min = 0,
    .max = 100,
  },
  {
    .fieldName = "swingPeriod",
    .title =  "Swing period [seconds]",
    .jsonType = json_type_double,
    .offset = OFFS(swingPeriod),
    .min = -3,
    .max = 3,
  },
  {
    .fieldName = "swingCurveExp",
    .title =  "Swing power curve exponent, -1.85 is near sine wave",
    .jsonType = json_type_double,
    .offset = OFFS(swingCurveExp),
    .min = -3,
    .max = 3,
  },
  {
    .fieldName = "midPointAdjustTime",
    .title =  "Midpoint adjust ramp time [Seconds]",
    .jsonType = json_type_double,
    .offset = OFFS(midPointAdjustTime),
    .min = 0,
    .max = 1,
  },
};

static int numSettingsFields = sizeof(settingsFieldDefs)/sizeof(SettingsFieldDef);





// MARK: ===== settimgs DB database


// Version history
//  1 : initial version
#define WIPERPARAMS_SCHEMA_VERSION 1 // minimally supported version, anything older will be deleted
#define WIPERPARAMS_SCHEMA_MIN_VERSION 1 // current version

/// persistence for digitalSTROM paramters
class WiperParamStore : public ParamStore
{
  typedef SQLite3Persistence inherited;
protected:

  /// Get DB Schema creation/upgrade SQL statements
  string dbSchemaUpgradeSQL(int aFromVersion, int &aToVersion)
  {
    string sql;
    if (aFromVersion==0) {
      // create DB from scratch
      // - use standard globs table for schema version
      sql = inherited::dbSchemaUpgradeSQL(aFromVersion, aToVersion);
      // - no vdchost level table to create at this time
      //   (PersistentParams create and update their tables as needed)
      // reached final version in one step
      aToVersion = WIPERPARAMS_SCHEMA_VERSION;
    }
    return sql;
  }

};




/// Main program for plan44.ch P44-DSB-DEH in form of the "vdcd" daemon)
class P44WiperD : public CmdLineApp, public PersistentParams
{
  typedef CmdLineApp inherited;
  typedef PersistentParams inheritedParams;

  // API Server
  SocketCommPtr apiServer;

  // Motor driver
  DcMotorDriverPtr motorDriver;
  DigitalIoPtr zeroPosInput;

  // Movement sensor
  DigitalIoPtr movementInput;

  // LED+Button
  ButtonInputPtr button;
  IndicatorOutputPtr greenLed;
  IndicatorOutputPtr redLed;

  // settings
  WiperParamStore settingsStore; ///< the database for storing settings persistently
  WiperSettings settings; ///< the settings variables

  MLMicroSeconds starttime;
  MLMicroSeconds lastZeroPosTime;
  long opTicket;
  StatusCB opDoneCB;

  enum {
    mv_unknown,
    mv_busy,
    mv_calibrate_find_zero,
    mv_calibrate_measure,
    mv_return_zero_cw,
    mv_return_zero_ccw,
    mv_return_zero_more_ccw,
    mv_zeroed,
    mv_swing_before_zero,
    mv_swing_after_zero
  } mvState;
  int swingDirection;


public:

  P44WiperD() :
    inheritedParams(settingsStore),
    starttime(MainLoop::now()),
    mvState(mv_unknown),
    opTicket(0),
    lastZeroPosTime(Never)
  {
    // default settings
    settings.calibrateRotationTime = 1.413; // measured
    settings.calibratePower = 80; // moderate speed
    settings.rezeroSwingAngle = 90; // half circle max
    settings.findZeroRamp = 0.1; // not too sudden start+stop
    settings.swingCurveExp = -1.85; // near sine
    settings.swingMaxPower = 80; // moderate
    settings.swingPeriod = 1.5; // one swing time
    settings.midPointAdjustTime = 0.3; // midpoint max adjust time
  }


  virtual int main(int argc, char **argv)
  {
    const char *usageText =
      "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "jsonapiport",    true,  "port;server port number for JSON API (default=none)" },
      { 0  , "jsonapinonlocal",false, "allow JSON API from non-local clients" },
      { 's', "sqlitedir",      true,  "dirpath;set SQLite DB directory (default = " DEFAULT_DBDIR ")" },
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
      { 0  , "calibrate",      false, "measure one rotation at full speed and adjust setting" },
      { 0  , "power",          true,  "float;end-of-rampp power, 0..100" },
      { 0  , "initialpower",   true,  "float;initial power, 0..100" },
      { 0  , "initialdir",     true,  "int;initial direction -1,0,1" },
      { 0  , "dir",            true,  "int;direction -1,0,1" },
      { 0  , "exp",            true,  "float;exponent for ramp, 1=linear" },
      { 0  , "fullramp",       true,  "float;seconds for full ramp" },
      { 0  , "runfor",         true,  "float;seconds to keep running after end of ramp" },
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

      // - initialize settings
      string settingsdb = DEFAULT_DBDIR;
      getStringOption("sqlitedir", settingsdb);
      pathstring_format_append(settingsdb, "WiperSettings.sqlite3");
      ErrorPtr err = settingsStore.connectAndInitialize(settingsdb.c_str(), WIPERPARAMS_SCHEMA_VERSION, WIPERPARAMS_SCHEMA_MIN_VERSION, false);
      if (Error::isOK(err)) {
        // load the settings
        err = loadFromStore(NULL);
      }
      if (!Error::isOK(err)) {
        err->prefixMessage("Cannot load persistent settings: ");
        terminateAppWith(err);
      }

      // - show settings
      logParams();

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
      movementInput = DigitalIoPtr(new DigitalIo(getOption("movementinput","missing"), false, false));
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



  virtual void initialize()
  {
    // execute command line actions, if any
    if (!execCommandLineActions()) {
      // normal operation
      normalOperation();
    }
  }



  virtual void cleanup(int aExitCode)
  {
    save();
  }




  bool execCommandLineActions()
  {
    string s;
    if (getStringOption("calibrate",s)) {
      calibrate(boost::bind(&P44WiperD::terminateAppWith, this, _1));
      return true;
    }
    if (getStringOption("power",s)) {
      // manually drive a ramp
      double initialPower = 0;
      if (getStringOption("initialpower",s)) {
        sscanf(s.c_str(),"%lf", &initialPower);
      }
      double exp = 1;
      if (getStringOption("exp",s)) {
        sscanf(s.c_str(),"%lf", &exp);
      }
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
      // command line action has taken over
      return true;
    }
    return false; // no command line action
  }



  void normalOperation()
  {
    LOG(LOG_NOTICE, "Starting normal operation");
    findZero(boost::bind(&P44WiperD::zeroed, this, _1));
  }


  void zeroed(ErrorPtr aError)
  {
    if (!Error::isOK(aError)) {
      terminateAppWith(aError);
    }
    LOG(LOG_NOTICE, "Start swinging now");
    startSwing();
  }



  void rampComplete(double aCurrentPower, int aDirection, ErrorPtr aError)
  {
    if (Error::isOK(aError)) {
      // print data to stdout
      LOG(LOG_NOTICE, "Ramp complete, power=%.2f%%, direction=%d", aCurrentPower, aDirection);
      // keep running
      MLMicroSeconds runfor = 0;
      string s;
      if (getStringOption("runfor",s)) {
        double sec;
        if (sscanf(s.c_str(),"%lf", &sec)==1) {
          runfor = (double)sec*Second;
        }
      }
      // delay quit
      MainLoop::currentMainLoop().executeOnce(boost::bind(&Application::terminateApp, this, EXIT_SUCCESS), runfor);
    }
    else {
      LOG(LOG_ERR, "Error receiving data: %s", aError->description().c_str());
      terminateAppWith(aError);
    }
  }


  void buttonHandler(bool aState, bool aHasChanged, MLMicroSeconds aTimeSincePreviousChange)
  {
    // %%% Nop for now
    if (aHasChanged && aState) {
      motorDriver->stop();
    }
  }


  void movementHandler(bool aNewState)
  {
    // %%% Nop for now
    LOG(LOG_NOTICE, "Movement signal = %d", aNewState);
    redLed->steady(aNewState);
  }



  // MARK: ===== movement sequences


  void stopOps()
  {
    MainLoop::currentMainLoop().cancelExecutionTicket(opTicket);
  }


  void startOp(StatusCB aDoneCB)
  {
    stopOps();
    opDoneCB = aDoneCB;
  }


  void endOp(ErrorPtr aError = ErrorPtr())
  {
    stopOps();
    StatusCB cb = opDoneCB;
    opDoneCB = NULL;
    if (cb) cb(aError);
  }



  void zeroPosHandler(bool aNewState)
  {
    LOG(LOG_NOTICE, "Zero position signal = %d", aNewState);
    greenLed->steady(aNewState);
    if (aNewState) {
      // starting edge
      switch (mvState) {
        // calibration states
        case mv_calibrate_find_zero:
          // first zero pos pass, now start measuring
          mvState = mv_calibrate_measure;
          break;
        case mv_calibrate_measure:
          // second zero pos pass, done
          mvState = mv_zeroed;
          settings.calibrateRotationTime = (double)(MainLoop::now()-lastZeroPosTime)/Second;
          motorDriver->stop();
          LOG(LOG_NOTICE, "Calibration done, rotation time = %.2f Seconds", settings.calibrateRotationTime);
          saveChanges();
          endOp();
          break;
        // zero find states
        case mv_return_zero_cw:
        case mv_return_zero_ccw:
          mvState = mv_zeroed;
          motorDriver->stop();
          LOG(LOG_NOTICE, "Found zero position");
          endOp();
          break;
        // swing states ;-)
        case mv_swing_before_zero:
          LOG(LOG_INFO,"Swing midpoint -> decelerating");
          swingMidpoint();
          break;
        default:
          break;
      }
      // remember time
      lastZeroPosTime = MainLoop::now();
    }
  }



  void calibrate(StatusCB aDoneCB)
  {
    // smoothly start turning
    startOp(aDoneCB);
    mvState = mv_busy;
    motorDriver->rampToPower(settings.calibratePower, 1, 1, 0, boost::bind(&P44WiperD::calibrateUpToSpeed, this));
  }


  #define MAX_CALIBRATE_TIME (10*Second)

  void calibrateUpToSpeed()
  {
    // start actual calibration process now
    LOG(LOG_NOTICE, "Starting calibration round");
    mvState = mv_calibrate_find_zero;
    MainLoop::currentMainLoop().executeTicketOnce(opTicket, boost::bind(&P44WiperD::calibrateTimeout, this), MAX_CALIBRATE_TIME);
  }


  void calibrateTimeout()
  {
    motorDriver->stop();
    endOp(TextError::err("Calibration failed, no zero position found"));
  }



  void findZero(StatusCB aDoneCB)
  {
    startOp(aDoneCB);
    motorDriver->stop();
    if (zeroPosInput->isSet()) {
      zeroFindEnd(true);
      return;
    }
    // - move at max one quarter clockwise
    mvState = mv_return_zero_cw;
    motorDriver->rampToPower(settings.calibratePower, 1, settings.findZeroRamp);
    MainLoop::currentMainLoop().executeTicketOnce(opTicket, boost::bind(&P44WiperD::zeroFindTimeout, this), settings.calibrateRotationTime*settings.rezeroSwingAngle/360*Second);
  }


  void zeroFindTimeout()
  {
    LOG(LOG_DEBUG, "zeroFindTimeout");
    if (mvState==mv_return_zero_cw) {
      // try other direction
      mvState = mv_return_zero_ccw;
      motorDriver->rampToPower(settings.calibratePower, -1, settings.findZeroRamp);
      MainLoop::currentMainLoop().executeOnce(boost::bind(&P44WiperD::zeroFindTimeout, this), settings.calibrateRotationTime*settings.rezeroSwingAngle/360*2*Second);
    }
    else if (mvState==mv_return_zero_ccw) {
      // not found in other direction
      zeroFindEnd(false);
    }
  }
  

  void zeroFindEnd(bool aSuccess)
  {
    ErrorPtr err;
    motorDriver->stop();
    if (aSuccess) {
      mvState = mv_zeroed;
    }
    else {
      mvState = mv_unknown;
      err = TextError::err("Zero not within %d degrees range, needs calibration", (int)settings.rezeroSwingAngle);
    }
    endOp(err);
  }


  void startSwing()
  {
    if (mvState==mv_zeroed) {
      // start swing from "hanging" down position: move a bit to the side
      swingDirection = 1; // start clockwise
      swingMidpoint();
    }
    else {
      // just decelerate towards midpoint
      swingDecelerate();
    }
  }


  void stopSwing()
  {
    if (mvState==mv_swing_after_zero) {
      // was moving up, reverse direction for next restart
      swingDirection *= -1;
    }
    if (zeroPosInput->isSet()) {
      mvState = mv_zeroed;
    }
  }



  void swingAccelerate()
  {
    // assuming at left or right endpoint of swing
    mvState = mv_swing_before_zero;
    // - ramp power up twoards midpoint
    motorDriver->rampToPower(settings.swingMaxPower, swingDirection, settings.swingPeriod/2, settings.swingCurveExp, boost::bind(&P44WiperD::swingAccelerated, this));
  }


  void swingAccelerated()
  {
    LOG(LOG_INFO,"Swing accelerated to max, waiting for midpoint");
  }


  void swingMidpoint()
  {
    mvState = mv_swing_after_zero;
    // quickly set midpoint speed
    motorDriver->rampToPower(settings.swingMaxPower, swingDirection, settings.midPointAdjustTime, 0, boost::bind(&P44WiperD::swingDecelerate, this));
  }


  void swingDecelerate()
  {
    // assuming midpoint at full speed
    // - ramp power down twoards endpoint
    motorDriver->rampToPower(0, swingDirection, settings.swingPeriod/2, settings.swingCurveExp, boost::bind(&P44WiperD::swingDecelerated, this));
  }


  void swingDecelerated()
  {
    LOG(LOG_INFO,"Swing decelerated to zero, reversing direction and accelerating again");
    swingDirection *= -1;
    swingAccelerate();
  }

  

  // MARK: ===== API access


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



  void logParams()
  {
    for (int i=0; i<numSettingsFields; i++) {
      const SettingsFieldDef &fdef = settingsFieldDefs[i];
      string s;
      switch (fdef.jsonType) {
        case json_type_boolean: s = string_format("%s", FLD(bool, fdef.offset) ? "true" : "false"); break;
        case json_type_double: s = string_format("%.3f", FLD(double, fdef.offset)); break;
        case json_type_int: s = string_format("%d", FLD(int, fdef.offset)); break;
        case json_type_string: s = string_format("'%s'", FLD(string, fdef.offset).c_str()); break;
        default: s = "<unknown>"; break;
      }
      LOG(LOG_INFO, "%s = %s  (%s)", fdef.fieldName, s.c_str(), fdef.title);
    }
  }





  // MARK: ===== persistence implementation


  void saveChanges()
  {
    markDirty();
    save();
  }


  void save()
  {
    ErrorPtr err = saveToStore(NULL, false);
    if (!Error::isOK(err)) {
      LOG(LOG_ERR, "cannot save params: %s", err->description().c_str());
    }
  }



  // SQLIte3 table name to store these parameters to
  const char *tableName()
  {
    return "WiperSettings";
  }


  // data field definitions

  size_t numFieldDefs()
  {
    return inheritedParams::numFieldDefs()+numSettingsFields;
  }


  const FieldDefinition *getFieldDef(size_t aIndex)
  {
    static FieldDefinition fdef; // Warning: not thread safe

    if (aIndex<inheritedParams::numFieldDefs())
      return inheritedParams::getFieldDef(aIndex);
    aIndex -= inheritedParams::numFieldDefs();
    if (aIndex<numSettingsFields) {
      fdef.fieldName = settingsFieldDefs[aIndex].fieldName;
      switch (settingsFieldDefs[aIndex].jsonType) {
        case json_type_boolean: fdef.dataTypeCode = SQLITE_INTEGER; break;
        case json_type_double: fdef.dataTypeCode = SQLITE_FLOAT; break;
        case json_type_int: fdef.dataTypeCode = SQLITE_INTEGER; break;
        case json_type_string: fdef.dataTypeCode = SQLITE_TEXT; break;
        default: fdef.dataTypeCode = SQLITE_TEXT; break;
      }
      return &fdef;
    }
    return NULL;
  }


  /// load values from passed row
  void loadFromRow(sqlite3pp::query::iterator &aRow, int &aIndex, uint64_t *aCommonFlagsP)
  {
    inheritedParams::loadFromRow(aRow, aIndex, aCommonFlagsP);
    for (int i=0; i<numSettingsFields; i++) {
      const SettingsFieldDef &fdef = settingsFieldDefs[i];
      switch (fdef.jsonType) {
        case json_type_boolean: aRow->getIfNotNull(aIndex, FLD(bool, fdef.offset)); break;
        case json_type_double: aRow->getIfNotNull(aIndex, FLD(double, fdef.offset)); break;
        case json_type_int: aRow->getIfNotNull(aIndex, FLD(int, fdef.offset)); break;
        case json_type_string: {
          const char *s;
          if (aRow->getIfNotNull(aIndex, s)) {
            FLD(bool, fdef.offset) = s;
          }
          break;
        }
        default: break; // ignore others
      }
      aIndex++;
    }
  }


  // bind values to passed statement
  void bindToStatement(sqlite3pp::statement &aStatement, int &aIndex, const char *aParentIdentifier, uint64_t aCommonFlags)
  {
    inheritedParams::bindToStatement(aStatement, aIndex, aParentIdentifier, aCommonFlags);
    // bind the fields
    for (int i=0; i<numSettingsFields; i++) {
      const SettingsFieldDef &fdef = settingsFieldDefs[i];
      switch (fdef.jsonType) {
        case json_type_boolean: aStatement.bind(aIndex, FLD(bool, fdef.offset)); break;
        case json_type_double: aStatement.bind(aIndex, FLD(double, fdef.offset)); break;
        case json_type_int: aStatement.bind(aIndex, FLD(int, fdef.offset)); break;
        case json_type_string: aStatement.bind(aIndex, FLD(string, fdef.offset).c_str(), false); break;
        default: aStatement.bind(aIndex); // just bind NULL to unknown types
      }
      aIndex++;
    }
  }


};


// MARK: ===== main


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
