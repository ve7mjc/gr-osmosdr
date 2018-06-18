/* -*- c++ -*- */
/*
 * Copyright 2013 Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef INCLUDED_SPYSERVER_SOURCE_C_H
#define INCLUDED_SPYSERVER_SOURCE_C_H

#include <thread>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <gnuradio/sync_block.h>

#include "source_iface.h"
#include "spyserver_protocol.h"
#include "tcp_client.h"

#define SPYSERVER_SAMPLE_BUFFER_SIZE 256 * 1024

class spyserver_source_c;

/*
 * We use boost::shared_ptr's instead of raw pointers for all access
 * to gr::blocks (and many other data structures).  The shared_ptr gets
 * us transparent reference counting, which greatly simplifies storage
 * management issues.  This is especially helpful in our hybrid
 * C++ / Python system.
 *
 * See http://www.boost.org/libs/smart_ptr/smart_ptr.htm
 *
 * As a convention, the _sptr suffix indicates a boost::shared_ptr
 */
typedef boost::shared_ptr<spyserver_source_c> spyserver_source_c_sptr;

/*!
 * \brief Return a shared_ptr to a new instance of spyserver_source_c.
 *
 * To avoid accidental use of raw pointers, spyserver_source_c's
 * constructor is private.  make_spyserver_source_c is the public
 * interface for creating new instances.
 */
spyserver_source_c_sptr make_spyserver_source_c (const std::string & args = "");

/*!
 * \brief Provides a stream of complex samples.
 * \ingroup block
 */
class spyserver_source_c :
    public gr::sync_block,
    public source_iface
{
private:
  // The friend declaration allows make_spyserver_source_c to
  // access the private constructor.

  friend spyserver_source_c_sptr make_spyserver_source_c (const std::string & args);

  /*!
   * \brief Provides a stream of complex samples.
   */
  spyserver_source_c (const std::string & args);   // private constructor


public:
  ~spyserver_source_c ();  // public destructor

  bool start();
  bool stop();

  int work( int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items );

  static std::vector< std::string > get_devices();

  size_t get_num_channels( void );

  osmosdr::meta_range_t get_sample_rates( void );
  double set_sample_rate( double rate );
  double get_sample_rate( void );

  osmosdr::freq_range_t get_freq_range( size_t chan = 0 );
  double set_center_freq( double freq, size_t chan = 0 );
  double get_center_freq( size_t chan = 0 );
  double set_freq_corr( double ppm, size_t chan = 0 );
  double get_freq_corr( size_t chan = 0 );

  std::vector<std::string> get_gain_names( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( const std::string & name, size_t chan = 0 );
  bool set_gain_mode( bool automatic, size_t chan = 0 );
  bool get_gain_mode( size_t chan = 0 );
  double set_gain( double gain, size_t chan = 0 );
  double set_gain( double gain, const std::string & name, size_t chan = 0 );
  double get_gain( size_t chan = 0 );
  double get_gain( const std::string & name, size_t chan = 0 );

  double set_lna_gain( double gain, size_t chan = 0 );
  double set_mix_gain(double gain, size_t chan = 0 );
  double set_if_gain( double gain, size_t chan = 0 );
  double set_bb_gain( double gain, size_t chan = 0 ) { return set_mix_gain(gain, chan); };

  std::vector< std::string > get_antennas( size_t chan = 0 );
  std::string set_antenna( const std::string & antenna, size_t chan = 0 );
  std::string get_antenna( size_t chan = 0 );

  double set_bandwidth( double bandwidth, size_t chan = 0 );
  double get_bandwidth( size_t chan = 0 );
  osmosdr::freq_range_t get_bandwidth_range( size_t chan = 0 );

  void set_biast( bool enabled );
  bool get_biast();

private:
  static constexpr unsigned int BufferSize = 64 * 1024;
  const int DefaultDisplayPixels = 2000;
  const int DefaultFFTRange = 127;
  const uint32_t ProtocolVersion = SPYSERVER_PROTOCOL_VERSION;
  const std::string SoftwareID = std::string("gr-osmosdr");
  const std::string NameNoDevice = std::string("SpyServer - No Device");
  const std::string NameAirspyOne = std::string("SpyServer - Airspy One");
  const std::string NameAirspyHF = std::string("SpyServer - Airspy HF+");
  const std::string NameRTLSDR = std::string("SpyServer - RTLSDR");
  const std::string NameUnknown = std::string("SpyServer - Unknown Device");

  uint32_t minimumTunableFrequency;
  uint32_t maximumTunableFrequency;
  uint32_t deviceCenterFrequency;
  uint32_t channelCenterFrequency;
  uint32_t channelDecimationStageCount;
  int32_t gain;
  tcp_client client;

  void connect();
  void disconnect();
  void threadLoop();
  bool sayHello();
  void cleanup();
  void onConnect();

  bool setSetting(uint32_t settingType, std::vector<uint32_t> params);
  bool sendCommand(uint32_t cmd, std::vector<uint8_t> args);
  void parseMessage(char *buffer, uint32_t len);
  int parseHeader(char *buffer, uint32_t len);
  int parseBody(char *buffer, uint32_t len);
  void processDeviceInfo();
  void processClientSync();
  void processUInt8Samples();
  void processInt16Samples();
  void processFloatSamples();
  void processUInt8FFT();
  void handleNewMessage();
  void setStreamState();

  std::atomic_bool terminated;
  std::atomic_bool streaming;
  std::atomic_bool gotDeviceInfo;
  std::atomic_bool gotSyncInfo;
  std::atomic_bool canControl;
  std::atomic_bool isConnected;
  std::thread *receiverThread;

  uint32_t droppedBuffers;
  std::atomic<int64_t> down_stream_bytes;

  uint8_t *headerData;
  uint8_t *bodyBuffer;
  uint64_t bodyBufferLength;
  uint32_t parserPosition;
  uint32_t lastSequenceNumber;

  std::string ip;
  int port;

  DeviceInfo deviceInfo;
  MessageHeader header;

  uint32_t streamingMode;
  uint32_t parserPhase;

  boost::circular_buffer<gr_complex> *_fifo;
  boost::mutex _fifo_lock;
  boost::condition_variable _samp_avail;

  std::vector< std::pair<double, uint32_t> > _sample_rates;
  double _sample_rate;
  double _center_freq;
  double _freq_corr;
  bool _auto_gain;
  double _gain;
};

#endif /* INCLUDED_SPYSERVER_SOURCE_C_H */
