/* -*- c++ -*- */
/*
 * Copyright 2019 Franco Venturi
 * Copyright 2018 Jeff Long <willcode4@gmail.com>
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gnu Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sdrplay3_source_c.h"
#include <gnuradio/io_signature.h>
#include "osmosdr/source.h"
#include "arg_helpers.h"

#include <boost/algorithm/string.hpp>
#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <iostream>
#include <mutex>

#define MAX_SUPPORTED_DEVICES   4

using namespace boost::assign;

// Index by sdrplay_api_Bw_MHzT
static std::vector<double> bandwidths = {
  0,     // Dummy
  200e3,
  300e3,
  600e3,
  1536e3,
  5000e3,
  6000e3,
  7000e3,
  8000e3
};

// TODO - RSP1 lower freq is 10e3.
#define SDRPLAY3_FREQ_MIN 1e3
#define SDRPLAY3_FREQ_MAX 2000e6

static std::string hwName(int hwVer)
{
  if (hwVer == SDRPLAY_RSP1_ID)
    return "RSP1";
  if (hwVer == SDRPLAY_RSP2_ID)
    return "RSP2";
  if (hwVer == SDRPLAY_RSP1A_ID)
    return "RSP1A";
  if (hwVer == SDRPLAY_RSPduo_ID)
    return "RSPduo";
  return "UNK";
}

static std::string rspDuoModeName(sdrplay_api_RspDuoModeT rspDuoMode)
{
  if (rspDuoMode == sdrplay_api_RspDuoMode_Single_Tuner)
    return "Single Tuner";
  if (rspDuoMode == sdrplay_api_RspDuoMode_Dual_Tuner)
    return "Dual Tuner";
  if (rspDuoMode == sdrplay_api_RspDuoMode_Master)
    return "Master";
  if (rspDuoMode == sdrplay_api_RspDuoMode_Slave)
    return "Slave";
  return "Unknown";
}

static std::string tunerName(sdrplay_api_TunerSelectT tuner)
{
  if (tuner == sdrplay_api_Tuner_A)
    return "A";
  if (tuner == sdrplay_api_Tuner_B)
    return "B";
  if (tuner == sdrplay_api_Tuner_Both)
    return "Both";
  return "Neither";
}

sdrplay3_source_c_sptr
make_sdrplay3_source_c (const std::string &args)
{
  return gnuradio::get_initial_sptr(new sdrplay3_source_c (args));
}

// 0 inputs, 1/2 output
static const int MIN_IN = 0;
static const int MAX_IN = 0;
static const int MIN_OUT = 1;
static const int MAX_OUT = 2;

static bool isApiOpen = false;

bool sdrplay3_source_c::apiOpen(void)
{
  if (!isApiOpen) {
    sdrplay_api_ErrT err;
    if ((err = sdrplay_api_Open()) != sdrplay_api_Success)
      return false;
    isApiOpen = true;
    atexit(apiClose);
    return true;
  }
  return true;
}

void sdrplay3_source_c::apiClose(void)
{
  if (isApiOpen) {
    sdrplay_api_Close();
    isApiOpen = false;
  }
}

sdrplay3_source_c::sdrplay3_source_c (const std::string &args)
  : gr::sync_block ("sdrplay3_source_c",
                    gr::io_signature::make(MIN_IN, MAX_IN, sizeof (gr_complex)),
                    gr::io_signature::make(MIN_OUT, MAX_OUT, sizeof (gr_complex))),
  _auto_gain(true),
  _gRdB(40),
  _lna(0),
  _bcastNotch(0),
  _dabNotch(0),
  _fsHz(2e6),
  _decim(1),
  _rfHz(100e6),
  _bwType(sdrplay_api_BW_1_536),
  _ifType(sdrplay_api_IF_Zero),
  _loMode(sdrplay_api_LO_Auto),
  _dcMode(true),
  _iqMode(true),
  _bufferA(new Buffer()),
  _bufferB(NULL),
  _streaming(false),
  _flowgraphRunning(false),
  _reinit(false),
  _device({}),
  _tuner(sdrplay_api_Tuner_A),
  _rspDuoMode(sdrplay_api_RspDuoMode_Unknown)
{
  dict_t dict = params_to_dict(args);
  if (dict.count("sdrplay3")) {
    _devIndex = boost::lexical_cast<unsigned int>(dict["sdrplay3"]);
  }
  else {
    _devIndex = 0;
  }

  if (!apiOpen())
    return;

  unsigned int numDevices;
  sdrplay_api_DeviceT sdrplayDevices[MAX_SUPPORTED_DEVICES];
  sdrplay_api_LockDeviceApi();
  sdrplay_api_GetDevices(sdrplayDevices, &numDevices, MAX_SUPPORTED_DEVICES);
  sdrplay_api_UnlockDeviceApi();
  if (_devIndex+1 > numDevices) {
    std::cerr << "Failed to open SDRplay device " + std::to_string(_devIndex) << std::endl;
    throw std::runtime_error("Failed to open SDRplay device " + std::to_string(_devIndex));
  }

  _hwVer = sdrplayDevices[_devIndex].hwVer;

  sdrplay_api_DbgLvl_t debug = sdrplay_api_DbgLvl_Disable;
  if (dict.count("debug") && (boost::lexical_cast<int>(dict["debug"]) != 0))
    debug = sdrplay_api_DbgLvl_Verbose;
  sdrplay_api_DebugEnable(sdrplayDevices[_devIndex].dev, debug);

  if (_hwVer == SDRPLAY_RSP2_ID) {
    _antenna = "A";
  }
  else if (_hwVer == SDRPLAY_RSPduo_ID) {
    _antenna = "T1_50ohm";
  }
  else {
    _antenna = "RX";
  }

  // bias=[0|1] to turn [off|on] bias tee. Default is off.
  _biasT = 0;
  if (dict.count("bias")) {
    _biasT = boost::lexical_cast<int>(dict["bias"]);
  }

  // lo=[120|144|168] to set first LO to 120/144/168 MHz. Default is Auto.
  if (dict.count("lo")) {
    int loMode = boost::lexical_cast<int>(dict["lo"]);
    if (loMode == 120)
      _loMode = sdrplay_api_LO_120MHz;
    else if (loMode == 144)
      _loMode = sdrplay_api_LO_144MHz;
    else if (loMode == 168)
      _loMode = sdrplay_api_LO_168MHz;
  }

  // tuner=[A|B]. Default is A
  if (_hwVer == SDRPLAY_RSPduo_ID) {
    if (dict.count("tuner")) {
      if (boost::iequals(dict["tuner"], "A")) {
        _tuner = sdrplay_api_Tuner_A;
      } else if (boost::iequals(dict["tuner"], "B")) {
        _tuner = sdrplay_api_Tuner_B;
      }
    }
  }

  // rspduo_mode=[single tuner,single|dual tuner,dual|master/slave,ms]. Default is 'single tuner'
  if (_hwVer == SDRPLAY_RSPduo_ID) {
    _rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
    if (dict.count("rspduo_mode")) {
      if (boost::iequals(dict["rspduo_mode"], "single tuner") ||
          boost::iequals(dict["rspduo_mode"], "single")) {
        _rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
      } else if (boost::iequals(dict["rspduo_mode"], "dual tuner") ||
                 boost::iequals(dict["rspduo_mode"], "dual")) {
        _rspDuoMode = sdrplay_api_RspDuoMode_Dual_Tuner;
      } else if (boost::iequals(dict["rspduo_mode"], "master/slave") ||
                 boost::iequals(dict["rspduo_mode"], "ms")) {
        _rspDuoMode = (sdrplay_api_RspDuoModeT)(sdrplay_api_RspDuoMode_Master | sdrplay_api_RspDuoMode_Slave);
      }
    }
    if (_rspDuoMode == sdrplay_api_RspDuoMode_Dual_Tuner)
      _bufferB = new Buffer();
  }
}

sdrplay3_source_c::~sdrplay3_source_c ()
{
  if (_streaming) {
    stopStreaming();
  }
  if (_device.dev) {
    sdrplay_api_ReleaseDevice(&_device);
    _device = {};
  }
  apiClose();
}

bool sdrplay3_source_c::start(void)
{
  boost::mutex::scoped_lock lock(_bufferA->mutex);
  if (_bufferB)
    boost::mutex::scoped_lock lock(_bufferB->mutex);
  _flowgraphRunning = true;
  return true;
}

bool sdrplay3_source_c::stop(void)
{
  boost::mutex::scoped_lock lock(_bufferA->mutex);
  if (_bufferB)
    boost::mutex::scoped_lock lock(_bufferB->mutex);
  _flowgraphRunning = false;
  // FG may be modified, so assume copied pointer is invalid
  _bufferA->buffer = NULL;
  if (_bufferB)
    _bufferB->buffer = NULL;
  return true;
}

int sdrplay3_source_c::work(int noutput_items,
                           gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items)
{
  gr_complex *outA = (gr_complex *)output_items[0];
  gr_complex *outB = NULL;
  if (output_items.size() > 1)
    outB = (gr_complex *)output_items[1];

  if (!_streaming)
    startStreaming();

  // case: single tuner
  if (!(_bufferB && outB)) {
    boost::mutex::scoped_lock lockA(_bufferA->mutex);
    _bufferA->buffer = outA;
    _bufferA->spaceRemaining = noutput_items;
    _bufferA->offset = 0;
    _bufferA->ready.notify_one();

    while (_bufferA->buffer && _streaming) {
      _bufferA->ready.wait(lockA);
    }

    if (!_streaming) {
      return 0;
    }

    return noutput_items - _bufferA->spaceRemaining;
  }

  // case: dual tuner
  boost::mutex::scoped_lock lockA(_bufferA->mutex);
  boost::mutex::scoped_lock lockB(_bufferB->mutex);
  _bufferA->buffer = outA;
  _bufferB->buffer = outB;
  _bufferA->spaceRemaining = noutput_items;
  _bufferB->spaceRemaining = noutput_items;
  _bufferA->offset = 0;
  _bufferB->offset = 0;
  _bufferA->ready.notify_one();
  _bufferB->ready.notify_one();

  while (true) {
    if (!_streaming)
      break;
    if (_bufferA->buffer)
      _bufferA->ready.wait(lockA);
    if (!_streaming)
      break;
    if (_bufferB->buffer)
      _bufferB->ready.wait(lockB);
    if (!_streaming)
      break;
    if (!_bufferA->buffer && !_bufferB->buffer)
      break;
  }

  if (!_streaming) {
    return 0;
  }

  return noutput_items - std::max(_bufferA->spaceRemaining, _bufferB->spaceRemaining);
}

// Called by sdrplay streamer thread when data is available
void sdrplay3_source_c::streamCallback(short *xi, short *xq,
                                      sdrplay_api_StreamCbParamsT *params,
                                      unsigned int numSamples,
                                      unsigned int reset,
                                      int idxBuffer)
{
  Buffer *buffer = NULL;
  if (idxBuffer == 0) {
    buffer = _bufferA;
  } else if (idxBuffer == 1) {
    buffer = _bufferB;
  }

  unsigned int i = 0;
  _reinit = false;

  boost::mutex::scoped_lock lock(buffer->mutex);

  while (i < numSamples) {

    // Discard samples if not streaming, if flowgraph not running, or reinit needed.
    if (!_streaming || _reinit || !_flowgraphRunning)
      return;

    // While buffer is not ready for write, wait a short time.
    while (!buffer->buffer) {
      if (boost::cv_status::timeout ==
          buffer->ready.wait_for(lock, boost::chrono::milliseconds(250)))
        return;
    }

    // Copy until out of samples or buffer is full
    while ((i < numSamples) && (buffer->spaceRemaining > 0)) {
      buffer->buffer[buffer->offset] =
        gr_complex(float(xi[i]) / 32768.0, float(xq[i]) / 32768.0);

      i++;
      buffer->offset++;
      buffer->spaceRemaining--;
    }

    if (buffer->spaceRemaining == 0) {
      buffer->buffer = NULL;
      buffer->ready.notify_one();
    }
  }
}

// Callback wrappers
void sdrplay3_source_c::streamACallbackWrap(short *xi, short *xq,
                                           sdrplay_api_StreamCbParamsT *params,
                                           unsigned int numSamples,
                                           unsigned int reset,
                                           void *cbContext)
{
  sdrplay3_source_c *obj = (sdrplay3_source_c *)cbContext;
  obj->streamCallback(xi, xq,
                      params,
                      numSamples, reset, 0);
}

void sdrplay3_source_c::streamBCallbackWrap(short *xi, short *xq,
                                           sdrplay_api_StreamCbParamsT *params,
                                           unsigned int numSamples,
                                           unsigned int reset,
                                           void *cbContext)
{
  sdrplay3_source_c *obj = (sdrplay3_source_c *)cbContext;
  obj->streamCallback(xi, xq,
                      params,
                      numSamples, reset, 1);
}

// Called by strplay streamer thread when an event occurs.
void sdrplay3_source_c::eventCallback(sdrplay_api_EventT eventId,
                                     sdrplay_api_TunerSelectT tuner,
                                     sdrplay_api_EventParamsT *params)
{
  if (eventId == sdrplay_api_GainChange)
  {
    int gRdB = params->gainParams.gRdB;
    int lnaGRdB = params->gainParams.lnaGRdB;
    std::cerr << "GR change, BB+MIX -" << gRdB << "dB, LNA -" << lnaGRdB << std::endl;
    // params->gainParams.curr is a calibrated gain value
  }
  else if (eventId == sdrplay_api_PowerOverloadChange)
  {
    if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Detected)
    {
      sdrplay_api_Update(_device.dev, _device.tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck);
      // OVERLOAD DETECTED
    }
    else if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Corrected)
    {
      sdrplay_api_Update(_device.dev, _device.tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck);
      // OVERLOAD CORRECTED
    }
  }
}

// Callback wrapper
void sdrplay3_source_c::eventCallbackWrap(sdrplay_api_EventT eventId,
                                         sdrplay_api_TunerSelectT tuner,
                                         sdrplay_api_EventParamsT *params,
                                         void *cbContext)
{
  sdrplay3_source_c *obj = (sdrplay3_source_c *)cbContext;
  obj->eventCallback(eventId, tuner, params);
}

void sdrplay3_source_c::startStreaming(void)
{
  if (_streaming)
    return;

  unsigned int numDevices;
  sdrplay_api_DeviceT sdrplayDevices[MAX_SUPPORTED_DEVICES];
  if (_device.dev) {
    sdrplay_api_ReleaseDevice(&_device);
    _device = {};
  }
  sdrplay_api_LockDeviceApi();
  sdrplay_api_GetDevices(sdrplayDevices, &numDevices, MAX_SUPPORTED_DEVICES);
  _device = sdrplayDevices[_devIndex];

  // set rspDuo mode
  if (_hwVer == SDRPLAY_RSPduo_ID) {
    // if master device is available, select device as master
    if ((_rspDuoMode & sdrplay_api_RspDuoMode_Master) && (_device.rspDuoMode & sdrplay_api_RspDuoMode_Master)) {
      _rspDuoMode = sdrplay_api_RspDuoMode_Master;
    } else if (_rspDuoMode & sdrplay_api_RspDuoMode_Slave) {
      _rspDuoMode = sdrplay_api_RspDuoMode_Slave;
    }
    if (_rspDuoMode == sdrplay_api_RspDuoMode_Dual_Tuner)
      _tuner = sdrplay_api_Tuner_Both;
    _device.tuner = _tuner;
    _device.rspDuoMode = _rspDuoMode;
  }

  sdrplay_api_SelectDevice(&_device);
  sdrplay_api_UnlockDeviceApi();

  std::cerr << "Using SDRplay API 3.x " << hwName(_hwVer) << " "
            << _device.SerNo << std::endl;
  if (_hwVer == SDRPLAY_RSPduo_ID) {
    std::cerr << "RSPduo mode " << rspDuoModeName(_rspDuoMode) << " - "
              << "tuner " << tunerName(_tuner) << std::endl;
  }

  sdrplay_api_GetDeviceParams(_device.dev, &_deviceParams);
  _chParams = _device.tuner == sdrplay_api_Tuner_B ? _deviceParams->rxChannelB : _deviceParams->rxChannelA;

  // Set bias voltage on/off (RSP1A/RSP2).
  std::cerr << "Bias voltage: " << _biasT << std::endl;
  if (_hwVer == SDRPLAY_RSP2_ID)
    _chParams->rsp2TunerParams.biasTEnable = _biasT ? 1 : 0;
  else if (_hwVer == SDRPLAY_RSPduo_ID)
    _chParams->rspDuoTunerParams.biasTEnable = _biasT ? 1 : 0;
  else if (_hwVer == SDRPLAY_RSP1A_ID)
    _chParams->rsp1aTunerParams.biasTEnable = _biasT ? 1 : 0;

  // Set first LO frequency
  _chParams->tunerParams.loMode = _loMode;

  _streaming = true;

  set_gain_mode(get_gain_mode(/*channel*/ 0), /*channel*/ 0);

  _chParams->tunerParams.gain.gRdB = _gRdB;
  _deviceParams->devParams->fsFreq.fsHz = _fsHz;
  _chParams->tunerParams.rfFreq.rfHz = _rfHz;
  _chParams->tunerParams.bwType = _bwType;
  _chParams->tunerParams.ifType = _ifType;
  _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);

  // Set decimation with halfband filter
  _chParams->ctrlParams.decimation.enable = _decim != 1;
  _chParams->ctrlParams.decimation.decimationFactor = _decim;
  _chParams->ctrlParams.decimation.wideBandSignal = 1;

  _chParams->ctrlParams.dcOffset.DCenable = _dcMode;
  _chParams->ctrlParams.dcOffset.IQenable = _iqMode;

  // Model-specific initialization
  if (_hwVer == SDRPLAY_RSP2_ID) {
    set_antenna(get_antenna(), 0);
    _chParams->rsp2TunerParams.rfNotchEnable = _bcastNotch;
  }
  else if (_hwVer == SDRPLAY_RSPduo_ID) {
    set_antenna(get_antenna(), 0);
    if (_antenna == "HIGHZ")
      _chParams->rspDuoTunerParams.tuner1AmNotchEnable = _bcastNotch;
    else
      _chParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
    _chParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
  }
  else if (_hwVer == SDRPLAY_RSP1A_ID) {
    _deviceParams->devParams->rsp1aParams.rfNotchEnable = _bcastNotch;
    _deviceParams->devParams->rsp1aParams.rfDabNotchEnable = _dabNotch;
  }

  sdrplay_api_CallbackFnsT cbFns;
  cbFns.StreamACbFn = streamACallbackWrap;
  cbFns.StreamBCbFn = streamBCallbackWrap;
  cbFns.EventCbFn = eventCallbackWrap;

  sdrplay_api_Init(_device.dev, &cbFns, this);
}

void sdrplay3_source_c::stopStreaming(void)
{
  if (!_streaming)
    return;

  _streaming = false;

  sdrplay_api_Uninit(_device.dev);
  sdrplay_api_ReleaseDevice(&_device);
  _device = {};
}

void sdrplay3_source_c::reinitDevice(int reason)
{
  // If no reason given, reinit everything
  if (reason == sdrplay_api_Update_None)
    reason = (sdrplay_api_Update_Tuner_Gr |
              sdrplay_api_Update_Dev_Fs |
              sdrplay_api_Update_Tuner_Frf |
              sdrplay_api_Update_Tuner_BwType |
              sdrplay_api_Update_Tuner_IfType |
              sdrplay_api_Update_Tuner_LoMode |
              sdrplay_api_Update_Rsp2_AmPortSelect |
              sdrplay_api_Update_RspDuo_AmPortSelect);

  // Tell stream CB to return
  _reinit = true;

  _chParams->tunerParams.gain.gRdB = _gRdB;
  _deviceParams->devParams->fsFreq.fsHz = _fsHz;
  _chParams->tunerParams.rfFreq.rfHz = _rfHz;
  _chParams->tunerParams.bwType = _bwType;
  _chParams->tunerParams.ifType = _ifType;
  _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);

  // Set decimation with halfband filter
  _chParams->ctrlParams.decimation.enable = _decim != 1;
  _chParams->ctrlParams.decimation.decimationFactor = _decim;
  _chParams->ctrlParams.decimation.wideBandSignal = 1;

  sdrplay_api_Update(_device.dev, _device.tuner, (sdrplay_api_ReasonForUpdateT)reason);

  _bufferA->ready.notify_one();
  if (_bufferB) {
    _bufferB->ready.notify_one();
  }
}

std::vector<std::string> sdrplay3_source_c::get_devices()
{
  unsigned int numDevices;
  sdrplay_api_DeviceT sdrplayDevices[MAX_SUPPORTED_DEVICES];
  std::vector<std::string> devices;

  if (!apiOpen())
    return devices;

  sdrplay_api_LockDeviceApi();
  sdrplay_api_GetDevices(sdrplayDevices, &numDevices, MAX_SUPPORTED_DEVICES);
  sdrplay_api_UnlockDeviceApi();

  for (unsigned int i=0; i<numDevices; i++) {
    sdrplay_api_DeviceT *dev = &sdrplayDevices[i];
    std::string args = boost::str(boost::format("sdrplay3=%d,label='SDRplay API 3.x %s %s'")
                                  % i % hwName((int)dev->hwVer) % dev->SerNo );
    std::cerr << args << std::endl;
    devices.push_back( args );
  }

  // close sdrplay API in case some other driver needs it
  apiClose();

  return devices;
}

size_t sdrplay3_source_c::get_num_channels()
{
  if (_hwVer == SDRPLAY_RSPduo_ID && _device.rspDuoMode == sdrplay_api_RspDuoMode_Dual_Tuner)
    return 2;
  return 1;
}

osmosdr::meta_range_t sdrplay3_source_c::get_sample_rates()
{
  osmosdr::meta_range_t range;
  range += osmosdr::range_t( 62.5e3, 10e6 );
  return range;
}

double sdrplay3_source_c::set_sample_rate(double rate)
{
  rate = std::min( std::max(rate,62.5e3), 10e6 );
  _fsHz = rate;

  // Decimation is required for rates below 2MS/s
  _decim = 1;
  while (_fsHz < 2e6) {
    _decim *= 2;
    _fsHz *= 2;
  }

  if (_streaming)
    reinitDevice(sdrplay_api_Update_Dev_Fs | sdrplay_api_Update_Ctrl_Decimation);

  return get_sample_rate();
}

double sdrplay3_source_c::get_sample_rate()
{
  return _fsHz/_decim;
}

osmosdr::freq_range_t sdrplay3_source_c::get_freq_range(size_t chan)
{
  osmosdr::freq_range_t range;
  range += osmosdr::range_t(SDRPLAY3_FREQ_MIN,  SDRPLAY3_FREQ_MAX);
  return range;
}

double sdrplay3_source_c::set_center_freq(double freq, size_t chan)
{
  _rfHz = freq;

  if (_streaming)
    reinitDevice(sdrplay_api_Update_Tuner_Frf);

  return get_center_freq( chan );
}

double sdrplay3_source_c::get_center_freq(size_t chan)
{
  return _rfHz;
}

double sdrplay3_source_c::set_freq_corr(double ppm, size_t chan)
{
  return get_freq_corr( chan );
}

double sdrplay3_source_c::get_freq_corr(size_t chan)
{
  return 0;
}

std::vector<std::string> sdrplay3_source_c::get_gain_names(size_t chan)
{
  std::vector<std::string> gains;

  gains += "LNA_ATTEN_STEP";
  gains += "IF_ATTEN_DB";

  // RSP1A and RSP2 have broadcast notch filters, and RSP1A has a DAB
  // notch filter. Show all controls for all models, mainly because
  // gqrx gets confused when switching between sources with different
  // sets of gains.
  gains += "BCAST_NOTCH";
  gains += "DAB_NOTCH";

  return gains;
}

osmosdr::gain_range_t sdrplay3_source_c::get_gain_range(size_t chan)
{
  osmosdr::gain_range_t range;

  for (int i = 20; i <= 59; i++)
    range += osmosdr::range_t((float)i);

  return range;
}

osmosdr::gain_range_t sdrplay3_source_c::get_gain_range(const std::string & name, size_t chan)
{
  osmosdr::gain_range_t range;
  int maxLnaState;

  if (name == "LNA_ATTEN_STEP") {
    if (_hwVer == SDRPLAY_RSP2_ID)
      maxLnaState = 8;
    else if (_hwVer == SDRPLAY_RSPduo_ID)
      maxLnaState = 9;
    else if (_hwVer == SDRPLAY_RSP1A_ID)
      maxLnaState = 9;
    else
      maxLnaState = 3;
    for (int i = 0; i <= maxLnaState; i++)
      range += osmosdr::range_t((float)i);
  }
  // RSP1A, RSP2, RSPduo
  else if (name == "BCAST_NOTCH") {
    range += osmosdr::range_t((float)0);
    if (_hwVer == SDRPLAY_RSP2_ID || _hwVer == SDRPLAY_RSPduo_ID || _hwVer == SDRPLAY_RSP1A_ID)
      range += osmosdr::range_t((float)1);
  }
  // RSP1A, RSPduo
  else if (name == "DAB_NOTCH") {
    range += osmosdr::range_t((float)0);
    if (_hwVer == SDRPLAY_RSPduo_ID || _hwVer == SDRPLAY_RSP1A_ID)
      range += osmosdr::range_t((float)1);
  }
  else {
    for (int i = 20; i <= 59; i++)
      range += osmosdr::range_t((float)i);
  }

  return range;
}

bool sdrplay3_source_c::set_gain_mode(bool automatic, size_t chan)
{
  _auto_gain = automatic;
  if (_streaming) {
    if (automatic) {
      _chParams->ctrlParams.agc.enable = sdrplay_api_AGC_5HZ;
    }
    else {
      _chParams->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
    }
    _chParams->ctrlParams.agc.setPoint_dBfs = -30;
    _chParams->ctrlParams.agc.knee_dBfs = 0;
    _chParams->ctrlParams.agc.decay_ms = 0;
    _chParams->ctrlParams.agc.decay_ms = 0;
    _chParams->ctrlParams.agc.hang_ms = 0;
    _chParams->ctrlParams.agc.syncUpdate = 0;
    _chParams->ctrlParams.agc.LNAstate = checkLNA(_lna);
  }

  return _auto_gain;
}

bool sdrplay3_source_c::get_gain_mode(size_t chan)
{
  return _auto_gain;
}

int sdrplay3_source_c::checkLNA(int lna)
{
  // Clip LNA reduction step. See table in API section 5.3.
  if (_hwVer == SDRPLAY_RSP1_ID) {
    lna = std::min(3, lna);
  }
  else if (_hwVer == SDRPLAY_RSP1A_ID) {
    if (_rfHz < 60000000)
      lna = std::min(6, lna);
    else if (_rfHz >= 1000000000)
      lna = std::min(8, lna);
    else
      lna = std::min(9, lna);
  }
  else if (_hwVer == SDRPLAY_RSP2_ID) {
    if (_rfHz >= 420000000)
      lna = std::min(5, lna);
    else if (_rfHz < 60000000 && _antenna == "HIGHZ")
      lna = std::min(4, lna);
    else
      lna = std::min(8, lna);
  }
  else if (_hwVer == SDRPLAY_RSPduo_ID) {
    if (_rfHz >= 1000000000)
      lna = std::min(8, lna);
    else if (_rfHz < 60000000 && _antenna == "HIGHZ")
      lna = std::min(4, lna);
    else if (_rfHz < 60000000)
      lna = std::min(6, lna);
    else
      lna = std::min(9, lna);
  }

  return lna;
}

double sdrplay3_source_c::set_gain(double gain, size_t chan)
{
  set_gain(gain, "IF_ATTEN_DB");
  return get_gain("IF_ATTEN_DB");
}

double sdrplay3_source_c::set_gain(double gain, const std::string & name, size_t chan)
{
  bool bcastNotchChanged = false;
  bool dabNotchChanged = false;
  bool gainChanged = false;

  if (name == "LNA_ATTEN_STEP") {
    if (gain != _lna)
      gainChanged = true;
    _lna = int(gain);
  }
  else if (name == "IF_ATTEN_DB") {
    // Ignore out-of-bounds values, since caller knows limits. (GQRX spurious calls).
    if (gain >= 20.0 && gain <= 59.0 && gain != _gRdB) {
      gainChanged = true;
      _gRdB = int(gain);
    }
  }
  // RSP1A, RSP2
  else if (name == "BCAST_NOTCH" && (_hwVer == SDRPLAY_RSP2_ID || _hwVer == SDRPLAY_RSPduo_ID ||  _hwVer == SDRPLAY_RSP1A_ID)) {
    if (int(gain) != _bcastNotch)
      bcastNotchChanged = true;
    _bcastNotch = int(gain);
  }
  // RSP1A
  else if (name == "DAB_NOTCH" && (_hwVer == SDRPLAY_RSPduo_ID || _hwVer == SDRPLAY_RSP1A_ID)) {
    if (int(gain) != _dabNotch)
      dabNotchChanged = true;
    _dabNotch = int(gain);
  }

  if (_streaming) {
    if (gainChanged) {
      _chParams->tunerParams.gain.gRdB = _gRdB;
      _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);
      _chParams->tunerParams.gain.syncUpdate = 0; /* immediate */
    }

    if (bcastNotchChanged) {
      if (_hwVer == SDRPLAY_RSP1A_ID ) {
        _deviceParams->devParams->rsp1aParams.rfNotchEnable = _bcastNotch;
      }
      else if (_hwVer == SDRPLAY_RSP2_ID) {
        _chParams->rsp2TunerParams.rfNotchEnable = _bcastNotch;
      }
      else if (_hwVer == SDRPLAY_RSPduo_ID) {
        if (_antenna == "HIGHZ")
          _chParams->rspDuoTunerParams.tuner1AmNotchEnable = _bcastNotch;
        else
          _chParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
      }
    }

    if (dabNotchChanged) {
      if (_hwVer == SDRPLAY_RSP1A_ID) {
        _deviceParams->devParams->rsp1aParams.rfDabNotchEnable = _dabNotch;
      }
      else if (_hwVer == SDRPLAY_RSPduo_ID) {
        _chParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
      }
    }

    reinitDevice(sdrplay_api_Update_Tuner_Gr);
  }

  return get_gain(chan);
}

double sdrplay3_source_c::get_gain(size_t chan)
{
  return get_gain("IF_ATTEN_DB");
}

double sdrplay3_source_c::get_gain(const std::string & name, size_t chan)
{
  if (name == "LNA_ATTEN_STEP")
    return _lna;
  else if (name == "BCAST_NOTCH")
    return _bcastNotch;
  else if (name == "DAB_NOTCH")
    return _dabNotch;
  else if (name == "IF_ATTEN_DB")
    return _gRdB;
  else
    return 0;
}

std::vector<std::string> sdrplay3_source_c::get_antennas(size_t chan)
{
  std::vector<std::string> antennas;

  if (_hwVer == SDRPLAY_RSP2_ID) {
    antennas += "A";
    antennas += "B";
    antennas += "HIGHZ";
  }
  else if (_hwVer == SDRPLAY_RSPduo_ID) {
    antennas += "T1_50ohm";
    antennas += "T2_50ohm";
    antennas += "HIGHZ";
  }
  else {
    antennas += "RX";
  }

  return antennas;
}

std::string sdrplay3_source_c::set_antenna(const std::string & antenna, size_t chan)
{
  _antenna = antenna;

  if (_streaming) {
    if (_hwVer == SDRPLAY_RSP2_ID) {
      // HIGHZ is ANTENNA_B with AmPortSelect
      if (antenna == "HIGHZ") {
        _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
        _chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_1;
      }
      else {
        if (antenna == "A")
          _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
        else
          _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
        _chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
      }

      reinitDevice(sdrplay_api_Update_Rsp2_AmPortSelect);
    }
    else if (_hwVer == SDRPLAY_RSPduo_ID) {
      if (antenna == "HIGHZ")
      {
        _device.tuner = sdrplay_api_Tuner_A;
        _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
      }
      else if(antenna == "T1_50ohm")
      {
        _device.tuner = sdrplay_api_Tuner_A;
        _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
      }
      else
      {
        _device.tuner = sdrplay_api_Tuner_B;
        _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
      }

      reinitDevice(sdrplay_api_Update_RspDuo_AmPortSelect);
    }
  }

  return antenna;
}

std::string sdrplay3_source_c::get_antenna(size_t chan)
{
  return _antenna.c_str();
}

void sdrplay3_source_c::set_dc_offset_mode(int mode, size_t chan)
{
  _dcMode = (osmosdr::source::DCOffsetAutomatic == mode);
  
  if (_streaming) {
    _chParams->ctrlParams.dcOffset.DCenable = _dcMode;
    reinitDevice(sdrplay_api_Update_Ctrl_DCoffsetIQimbalance);
  }
}

void sdrplay3_source_c::set_dc_offset(const std::complex<double> &offset, size_t chan)
{
  std::cerr << "set_dc_offset(): not implemented" << std::endl;
}

void sdrplay3_source_c::set_iq_balance_mode(int mode, size_t chan)
{
  _iqMode = (osmosdr::source::IQBalanceAutomatic == mode);

  if (_streaming) {
    _chParams->ctrlParams.dcOffset.IQenable = _iqMode;
    reinitDevice(sdrplay_api_Update_Ctrl_DCoffsetIQimbalance);
  }
}

double sdrplay3_source_c::set_bandwidth(double bandwidth, size_t chan)
{
  _bwType = sdrplay_api_BW_8_000;

  for (double bw : bandwidths) {
    // Skip dummy value at index 0
    if (bw == 0)
      continue;
    if (bandwidth <= bw) {
      _bwType = (sdrplay_api_Bw_MHzT)(bw/1e3);
      break;
    }
  }

  int actual = get_bandwidth(chan);
  std::cerr << "SDRplay bandwidth requested=" << bandwidth
            << " actual=" << actual << std::endl;

  if (_streaming) {
    reinitDevice(sdrplay_api_Update_Tuner_BwType);
  }

  return actual;
}

double sdrplay3_source_c::get_bandwidth(size_t chan)
{
  return (double)_bwType * 1e3;
}

osmosdr::freq_range_t sdrplay3_source_c::get_bandwidth_range(size_t chan)
{
  osmosdr::freq_range_t range;

  // bandwidths[0] is a dummy
  for (unsigned int i=1; i<bandwidths.size(); i++)
    range += osmosdr::range_t(bandwidths[i]);

  return range;
}
