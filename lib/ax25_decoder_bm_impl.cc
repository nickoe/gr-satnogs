/* -*- c++ -*- */
/*
 * gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2016-2019
 *  Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <satnogs/log.h>
#include <satnogs/ax25.h>
#include "ax25_decoder_bm_impl.h"

namespace gr
{
namespace satnogs
{

ax25_decoder_bm::sptr
ax25_decoder_bm::make (const std::string &addr, uint8_t ssid, bool promisc,
                       bool descramble, size_t max_frame_len)
{
  return gnuradio::get_initial_sptr (
      new ax25_decoder_bm_impl (addr, ssid, promisc, descramble, max_frame_len));
}

/*
 * The private constructor
 */
ax25_decoder_bm_impl::ax25_decoder_bm_impl (const std::string &addr,
                                            uint8_t ssid, bool promisc,
                                            bool descramble,
                                            size_t max_frame_len) :
        gr::sync_block ("ax25_decoder_bm",
                        gr::io_signature::make (1, 1, sizeof(uint8_t)),
                        gr::io_signature::make (0, 0, 0)),
        d_promisc (promisc),
        d_descramble (descramble),
        d_max_frame_len (max_frame_len),
        d_state (NO_SYNC),
        d_shift_reg (0x0),
        d_dec_b (0x0),
        d_prev_bit_nrzi (0),
        d_received_bytes (0),
        d_decoded_bits (0),
        d_lfsr (0x21, 0x0, 16),
        d_frame_buffer (
            new uint8_t[max_frame_len + AX25_MAX_ADDR_LEN + AX25_MAX_CTRL_LEN
                + sizeof(uint16_t)]),
        d_start_idx(0)
{
  /* Valid PDUs output message port */
  message_port_register_out (pmt::mp ("pdu"));
  /*
   * Valid invalid (wrong CRC, different destination Callsign/SSID,
   * wrong frame size)PDUs output message port
   */
  message_port_register_out (pmt::mp ("failed_pdu"));
}


void
ax25_decoder_bm_impl::decode ()
{
  while (1) {
    bool cont = false;
    switch (d_state)
      {
      case NO_SYNC:
        for (size_t i = 0; i < d_bitstream.size (); i++) {
          decode_1b (d_bitstream[i]);
          if (d_shift_reg == AX25_SYNC_FLAG) {
            d_bitstream.erase (d_bitstream.begin (),
                               d_bitstream.begin () + i + 1);
            enter_sync_state ();
            d_start_idx = 0;
            cont = true;
            break;
          }
        }
        if(cont) {
          continue;
        }
        d_bitstream.clear ();
        return;
      case IN_SYNC:
        /*
         * Most of the transmitters repeat several times the AX.25 SYNC
         * In case of G3RUH this is mandatory to allow the self synchronizing
         * scrambler to settle
         */
        for (size_t i = d_start_idx; i < d_bitstream.size (); i++) {
          decode_1b (d_bitstream[i]);
          d_decoded_bits++;
          if (d_decoded_bits == 8) {
            /* Perhaps we are in frame! */
            if (d_shift_reg != AX25_SYNC_FLAG) {
              d_start_idx = i + 1;
              enter_decoding_state ();
              cont = true;
              break;
            }
            d_decoded_bits = 0;
          }
        }
        if(cont) {
          continue;
        }
        d_start_idx = d_bitstream.size ();
        return;
      case DECODING:
        for (size_t i = d_start_idx; i < d_bitstream.size (); i++) {
          decode_1b (d_bitstream[i]);
          if (d_shift_reg == AX25_SYNC_FLAG) {
            LOG_DEBUG("Found frame end");
            if (enter_frame_end ()) {
              d_bitstream.erase (d_bitstream.begin (),
                                 d_bitstream.begin () + i + 1);
            }
            cont = true;
            break;
          }
          else if ((d_shift_reg & 0xfc) == 0x7c) {
            /*This was a stuffed bit */
            d_dec_b <<= 1;
          }
          else if ((d_shift_reg & 0xfe) == 0xfe) {
            LOG_DEBUG("Invalid shift register value %u", d_received_bytes);
            reset_state ();
            cont = true;
            break;
          }
          else {
            d_decoded_bits++;
            if (d_decoded_bits == 8) {
              d_frame_buffer[d_received_bytes++] = d_dec_b;
              d_decoded_bits = 0;

              /*Check if the frame limit was reached */
              if (d_received_bytes >= d_max_frame_len) {
                LOG_DEBUG("Wrong size");
                reset_state ();
                cont = true;
                break;
              }
            }
          }
        }
        if(cont) {
          continue;
        }
        d_start_idx = d_bitstream.size ();
        return;
      default:
        LOG_ERROR("Invalid decoding state");
        reset_state ();
        return;
      }
  }
}

/*
 * Our virtual destructor.
 */
ax25_decoder_bm_impl::~ax25_decoder_bm_impl ()
{
  delete[] d_frame_buffer;
  LOG_DEBUG("Left over: %lu", d_bitstream.size());
}

void
ax25_decoder_bm_impl::reset_state ()
{
  d_state = NO_SYNC;
  d_dec_b = 0x0;
  d_shift_reg = 0x0;
  d_decoded_bits = 0;
  d_received_bytes = 0;
}

void
ax25_decoder_bm_impl::enter_sync_state ()
{
  d_state = IN_SYNC;
  d_dec_b = 0x0;
  d_shift_reg = 0x0;
  d_decoded_bits = 0;
  d_received_bytes = 0;
}

void
ax25_decoder_bm_impl::enter_decoding_state ()
{
  uint8_t tmp;
  d_state = DECODING;
  d_decoded_bits = 0;
  d_received_bytes = 0;

  /*
   * Due to the possibility of bit stuffing on the first byte some special
   * handling is necessary
   */
  if ((d_shift_reg & 0xfc) == 0x7c) {
    /*This was a stuffed bit */
    d_dec_b <<= 1;
    d_decoded_bits = 7;
  }
  else {
    d_frame_buffer[0] = d_dec_b;
    d_decoded_bits = 0;
    d_received_bytes = 1;
  }
}

bool
ax25_decoder_bm_impl::enter_frame_end ()
{
  uint16_t fcs;
  uint16_t recv_fcs = 0x0;

  /* First check if the size of the frame is valid */
  if (d_received_bytes < AX25_MIN_ADDR_LEN + sizeof(uint16_t)) {
    reset_state ();
    return false;
  }

  /*
   * Check if the frame is correct using the FCS field
   * Using this field also try to correct up to 2 error bits
   */
  if (frame_check ()) {
    message_port_pub (
        pmt::mp ("pdu"),
        pmt::make_blob (d_frame_buffer, d_received_bytes - sizeof(uint16_t)));
    reset_state ();
    return true;
  }
  else {
    message_port_pub (
        pmt::mp ("failed_pdu"),
        pmt::make_blob (d_frame_buffer, d_received_bytes - sizeof(uint16_t)));
    LOG_DEBUG("Wrong crc");
    reset_state ();
    return false;
  }
}


inline void
ax25_decoder_bm_impl::decode_1b (uint8_t in)
{

  /* In AX.25 the LS bit is sent first */
  d_shift_reg = (d_shift_reg >> 1) | (in << 7);
  d_dec_b = (d_dec_b >> 1) | (in << 7);
}

bool
ax25_decoder_bm_impl::frame_check ()
{
  uint16_t fcs;
  uint16_t recv_fcs = 0x0;
  uint8_t orig_byte;

  /* Check if the frame is correct using the FCS field */
  fcs = ax25_fcs (d_frame_buffer, d_received_bytes - sizeof(uint16_t));
  recv_fcs = (((uint16_t) d_frame_buffer[d_received_bytes - 1]) << 8)
      | d_frame_buffer[d_received_bytes - 2];
  if (fcs == recv_fcs) {
    return true;
  }
  return false;
}

int
ax25_decoder_bm_impl::work (int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
{
  int ret;
  const uint8_t *in = (const uint8_t*) input_items[0];

  if (noutput_items < 1) {
    return noutput_items;
  }


  if (d_descramble) {
    for (int i = 0; i < noutput_items; i++) {
      /* Perform NRZI decoding */
      uint8_t b = (~((in[i] - d_prev_bit_nrzi) % 2)) & 0x1;
      d_prev_bit_nrzi = in[i];
      b = d_lfsr.next_bit_descramble (b);
      d_bitstream.push_back (b);
    }
  }
  else {
    for (int i = 0; i < noutput_items; i++) {
      /* Perform NRZI decoding */
      uint8_t b = (~((in[i] - d_prev_bit_nrzi) % 2)) & 0x1;
      d_prev_bit_nrzi = in[i];
      d_bitstream.push_back (b);
    }
  }
  decode();
  return noutput_items;
}

} /* namespace satnogs */
} /* namespace gr */

