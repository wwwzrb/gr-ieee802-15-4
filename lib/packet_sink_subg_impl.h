/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_IEEE802_15_4_PACKET_SINK_SUBG_IMPL_H
#define INCLUDED_IEEE802_15_4_PACKET_SINK_SUBG_IMPL_H

#include <ieee802_15_4/packet_sink_subg.h>
#include <cstdio>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <cstring>
#include <gnuradio/blocks/count_bits.h>
#include <iostream>
#include <iomanip>

namespace gr {
  namespace ieee802_15_4 {

    // very verbose output for almost each sample
    #define VERBOSE 0
    // debug syncword and header detection
    #define VERBOSE1 0
    // less verbose output for higher level debugging
    #define VERBOSE2 0

    // this is the mapping between chips and symbols if we do
    // a fm demodulation of the O-QPSK signal. Note that this
    // is different than the O-QPSK chip sequence from the
    // 802.15.4 standard since there there is a translation
    // happening.
    // See "CMOS RFIC Architectures for IEEE 802.15.4 Networks",
    // John Notor, Anthony Caviglia, Gary Levy, for more details.
    // For sub 1GHz OQPSK, a new mapping is given below
    // Chip mapping for subGHz ZigBee
    static const unsigned int CHIP_MAPPING[] = {29794, 15640, 12102, 3025, 25332, 6333, 17967, 20875, 2973, 17127, 20665, 29742, 7435, 26434, 14800, 11892};
    // Chip mapping for ZigRa
    // static const unsigned int CHIP_MAPPING[] = {
    //           29794, 
    //           15640, 
    //           10922, 
    //           32767, 
    //           25332, 
    //           6333, 
    //           17967, 
    //           20875, 
    //           2973, 
    //           17127,
    //           20665, 
    //           29742, 
    //           7435, 
    //           26434, 
    //           14800, 
    //           11892};

    static const int MAX_PKT_LEN    = 128 ; // remove header and CRC
    static const int MAX_LQI_SAMPLES = 4; // Number of chip correlation samples to take

    class packet_sink_subg_impl : public packet_sink_subg
    {
     private:
      enum demod_state_t {STATE_SYNC_SEARCH, STATE_HAVE_SYNC, STATE_HAVE_HEADER};

      demod_state_t d_state;

      unsigned int      d_sync_vector;           // 802.15.4 standard is 4x 0 bytes and 1x0xA7
      unsigned int      d_threshold;             // how many bits may be wrong in sync vector

      unsigned int      d_shift_reg;             // used to look for sync_vector
      int               d_preamble_cnt;          // count on where we are in preamble
      int               d_chip_cnt;              // counts the chips collected

      unsigned int      d_header;                // header bits
      int               d_headerbitlen_cnt;      // how many so far

      unsigned char     d_packet[MAX_PKT_LEN];   // assembled payload
      unsigned char     d_packet_byte;           // byte being assembled
      int               d_packet_byte_index;     // which bit of d_packet_byte we're working on
      int               d_packetlen;             // length of packet
      int               d_packetlen_cnt;         // how many so far
      int               d_payload_cnt;           // how many bytes in payload

      unsigned int      d_lqi;                   // Link Quality Information
      unsigned int      d_lqi_sample_count;

      // FIXME:
      char buf[256];

     public:

      void enter_search();
      void enter_have_sync();
      void enter_have_header(int payload_len);
      unsigned char decode_chips(unsigned int chips);
      int slice(float x);

      packet_sink_subg_impl(int threshold);
      ~packet_sink_subg_impl();


      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_PACKET_SINK_SUBG_IMPL_H */

