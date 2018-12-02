/* -*- c++ -*- */
/*
 * gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2018, Libre Space Foundation <http://librespacefoundation.org/>
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

#include "frame_acquisition_impl.h"

#include <satnogs/golay24.h>
#include <satnogs/log.h>
#include <satnogs/utils.h>
#include <arpa/inet.h>

namespace gr
{
namespace satnogs
{

frame_acquisition::sptr
frame_acquisition::make (variant_t variant,
       const std::vector<uint8_t>& preamble,
       size_t preamble_threshold,
       const std::vector<uint8_t>& sync,
       size_t sync_threshold,
       size_t frame_size_field_len,
       size_t frame_len,
       checksum_t crc,
       whitening::whitening_sptr descrambler,
       size_t max_frame_len)
{
  return gnuradio::get_initial_sptr (
      new frame_acquisition_impl (variant,
                                  preamble,
                                  preamble_threshold,
                                  sync,
                                  sync_threshold,
                                  frame_size_field_len,
                                  frame_len,
                                  crc,
                                  descrambler,
                                  max_frame_len));
}

frame_acquisition_impl::frame_acquisition_impl (variant_t variant,
                          const std::vector<uint8_t>& preamble,
                          size_t preamble_threshold,
                          const std::vector<uint8_t>& sync,
                          size_t sync_threshold,
                          size_t frame_size_field_len,
                          size_t frame_len,
                          checksum_t crc,
                          whitening::whitening_sptr descrambler,
                          size_t max_frame_len) :
        gr::sync_block ("frame_acquisition",
                        gr::io_signature::make (1, 1, sizeof(uint8_t)),
                        gr::io_signature::make (0, 0, 0)),
                        d_variant(variant),
                        d_preamble(preamble.size() * 8),
                        d_preamble_shift_reg(preamble.size() * 8),
                        d_preamble_len(preamble.size() * 8),
                        d_preamble_thrsh(preamble_threshold),
                        d_sync(sync.size() * 8),
                        d_sync_shift_reg(sync.size() * 8),
                        d_sync_len(sync.size() * 8),
                        d_sync_thrsh(sync_threshold),
                        d_state(SEARCHING),
                        d_cnt(0),
                        d_frame_size_field_len(frame_size_field_len),
                        d_frame_len(frame_len),
                        d_max_frame_len(max_frame_len),
                        d_crc(crc),
                        d_crc_len(0),
                        d_whitening(descrambler)
{
  set_output_multiple(8);
  for(uint8_t b : preamble) {
    d_preamble <<= (b >> 7);
    d_preamble <<= ((b >> 6) & 0x1);
    d_preamble <<= ((b >> 5) & 0x1);
    d_preamble <<= ((b >> 4) & 0x1);
    d_preamble <<= ((b >> 3) & 0x1);
    d_preamble <<= ((b >> 2) & 0x1);
    d_preamble <<= ((b >> 1) & 0x1);
    d_preamble <<= (b & 0x1);
  }
  for(uint8_t b : sync) {
    d_sync <<= (b >> 7);
    d_sync <<= ((b >> 6) & 0x1);
    d_sync <<= ((b >> 5) & 0x1);
    d_sync <<= ((b >> 4) & 0x1);
    d_sync <<= ((b >> 3) & 0x1);
    d_sync <<= ((b >> 2) & 0x1);
    d_sync <<= ((b >> 1) & 0x1);
    d_sync <<= (b & 0x1);
  }

  /* Parameters checking */
  if (max_frame_len == 0) {
    throw std::invalid_argument (
        "The maximum frame size should be at least 1 byte");
  }

  if(d_sync_len < 8) {
    throw std::invalid_argument("SYNC word should be at least 8 bits");
  }

  if(d_preamble_len < 8) {
    throw std::invalid_argument("Preamble should be at least 8 bits");
  }

  if (d_preamble_len < 2 * d_preamble_thrsh) {
    throw std::invalid_argument (
        "Too many error bits are allowed for the preamble."
        "Consider lowering the threshold");
  }

  if (d_sync_len < 2 * d_sync_thrsh) {
    throw std::invalid_argument (
        "Too many error bits are allowed for the sync word. "
        "Consider lowering the threshold");
  }

  if (d_frame_size_field_len > 4) {
    throw std::invalid_argument ("Frame length field can be up to 4 bytes");
  }

  if (d_frame_size_field_len == 0 && d_variant != GENERIC_CONSTANT_FRAME_LEN) {
    throw std::invalid_argument ("Frame length field cannot be 0");
  }

  if(d_variant == GENERIC_CONSTANT_FRAME_LEN) {
    d_frame_size_field_len = 0;
  }

  /* Set the CRC length */
  switch(d_crc) {
    case CRC16_CCITT:
      d_crc_len = 2;
      break;
    case CRC16_CCITT_REVERSED:
      d_crc_len = 2;
      break;
    case CRC16_IBM:
      d_crc_len = 2;
      break;
    case CRC32:
      d_crc_len = 4;
      break;
    default:
      d_crc_len = 0;
      break;
  }

  d_pdu = new uint8_t[max_frame_len];

  /* Register the PMT message queue */
  message_port_register_out(pmt::mp("out"));
}

/*
 * Our virtual destructor.
 */
frame_acquisition_impl::~frame_acquisition_impl ()
{
  delete [] d_pdu;
}



int
frame_acquisition_impl::work (int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
{
  const uint8_t *in = (const uint8_t *) input_items[0];

  switch(d_state)
  {
    case SEARCHING:
      return searching_preamble(in, noutput_items);
    case SEARCHING_SYNC:
      return searching_sync(in, noutput_items);
    case DECODING_GENERIC_FRAME_LEN:
      return dec_generic_frame_len(in, noutput_items);
    case DECODING_GOLAY24_FRAME_LEN:
      return dec_golay24_frame_len(in, noutput_items);
    case DECODING_PAYLOAD:
      return decoding(in, noutput_items);
    default:
      return noutput_items;
  }
}

int
frame_acquisition_impl::searching_preamble (const uint8_t* in, int len)
{
  for(int i = 0; i < len; i++) {
    d_preamble_shift_reg <<= in[i];
    shift_reg tmp = d_preamble_shift_reg ^ d_preamble;
    if(tmp.count() <= d_preamble_thrsh) {
      LOG_DEBUG("Found PREAMBLE");
      d_state = SEARCHING_SYNC;
      d_cnt = 0;
      return i+1;
    }
  }
  return len;
}

int
frame_acquisition_impl::searching_sync (const uint8_t* in, int len)
{
  for (int i = 0; i < len; i++) {
    d_sync_shift_reg <<= in[i];
    shift_reg tmp = d_sync_shift_reg ^ d_sync;
    d_cnt++;
    if (tmp.count () <= d_sync_thrsh) {
      LOG_DEBUG("Found SYNC");
      switch(d_variant) {
        case GENERIC_CONSTANT_FRAME_LEN:
          d_state = DECODING_PAYLOAD;
          break;
        case GENERIC_VAR_FRAME_LEN:
          d_state = DECODING_GENERIC_FRAME_LEN;
          break;
        case GOLAY24_CODED_FRAME_LEN:
          d_state = DECODING_GOLAY24_FRAME_LEN;
          break;
      }
      d_cnt = 0;
      return i + 1;
    }

    /* The sync word should be available by now */
    if(d_cnt > d_preamble_len * 2 + d_sync_len) {
      reset();
      return i + 1;
    }
  }
  return len;
}

int
frame_acquisition_impl::dec_generic_frame_len (const uint8_t* in, int len)
{
  const int s = std::min(len / 8, (int) d_frame_size_field_len);
  for(int i = 0; i < s; i++) {
    uint8_t b = 0x0;
    b |= in[i * 8] << 7;
    b |= in[i * 8 + 1] << 6;
    b |= in[i * 8 + 2] << 5;
    b |= in[i * 8 + 3] << 4;
    b |= in[i * 8 + 4] << 3;
    b |= in[i * 8 + 5] << 2;
    b |= in[i * 8 + 6] << 1;
    b |= in[i * 8 + 7];
    d_frame_len <<= 8;
    d_frame_len |= b;
    d_cnt++;
    if(d_cnt == d_frame_size_field_len) {
      /* Most of the available modems apply whitening on the frame length too */
      if(d_whitening) {
        uint32_t descrambled = 0x0;
        d_whitening->descramble((uint8_t *)&descrambled,
                                (const uint8_t *)&d_frame_len,
                                d_frame_size_field_len, false);
        d_frame_len = descrambled;
      }

      /* Mask the bits that are not used */
      d_frame_len &= ~(0xFFFFFFFF << (d_frame_size_field_len * 8));
      LOG_DEBUG("Found frame length: %u", d_frame_len);

      /* Length field is needed for the CRC calculation */
      for(uint32_t j = 0; j < d_frame_size_field_len; j++) {
        d_pdu[j] = (d_frame_len >> ((d_frame_size_field_len - 1 - j) * 8)) & 0xFF;
      }
      d_frame_len += d_frame_size_field_len;

      /* Append the CRC length if any */
      d_frame_len += d_crc_len;

      if(d_frame_len < d_max_frame_len) {
        d_state = DECODING_PAYLOAD;
      }
      else{
        reset();
        return (i + 1) * 8;
      }
      d_cnt = d_frame_size_field_len;
      return (i + 1) * 8;
    }
  }
  return s * 8;
}

int
frame_acquisition_impl::dec_golay24_frame_len (const uint8_t* in, int len)
{
  /* Golay24 needs 3 bytes to decode */
  const int s = std::min(len / 8, 3);
  for(int i = 0; i < s; i++) {
    uint8_t b = 0x0;
    b |= in[i * 8] << 7;
    b |= in[i * 8 + 1] << 6;
    b |= in[i * 8 + 2] << 5;
    b |= in[i * 8 + 3] << 4;
    b |= in[i * 8 + 4] << 3;
    b |= in[i * 8 + 5] << 2;
    b |= in[i * 8 + 6] << 1;
    b |= in[i * 8 + 7];
    d_frame_len <<= 8;
    d_frame_len |= b;
    d_cnt++;

    /* Try to decode the frame length */
    if (d_cnt == 3) {
      if(d_whitening) {
        uint32_t descrambled = 0x0;
        d_whitening->descramble((uint8_t *)&descrambled,
                                (const uint8_t *)&d_frame_len, 3, false);
        d_frame_len = descrambled;
      }
      golay24 g = golay24 ();
      uint16_t tmp = 0;
      if (g.decode24 (&tmp, d_frame_len)) {
        d_frame_len = tmp;

        /* Append the CRC length if any */
        d_frame_len += d_crc_len;

        /* Check if the payload can fit in the buffer */
        if(d_frame_len > d_max_frame_len) {
          reset();
          return (i + 1) * 8;
        }
        else{
          d_state = DECODING_PAYLOAD;
        }
      }
      else {
        reset ();
        return (i + 1) * 8;
      }
      d_cnt = 0;
      return (i + 1) * 8;
    }
  }
  return s * 8;
}

int
frame_acquisition_impl::decoding (const uint8_t* in, int len)
{
  const int s = len / 8;
  for(int i = 0; i < s; i++) {
    uint8_t b = 0x0;
    b = in[i * 8] << 7;
    b |= in[i * 8 + 1] << 6;
    b |= in[i * 8 + 2] << 5;
    b |= in[i * 8 + 3] << 4;
    b |= in[i * 8 + 4] << 3;
    b |= in[i * 8 + 5] << 2;
    b |= in[i * 8 + 6] << 1;
    b |= in[i * 8 + 7];
    d_pdu[d_cnt++] = b;
    if(d_cnt == d_frame_len) {
      if(d_whitening) {
        d_whitening->descramble(d_pdu  + d_frame_size_field_len,
                                d_pdu + d_frame_size_field_len,
                                d_frame_len - d_frame_size_field_len, false);
      }

      if (check_crc ()) {
        message_port_pub (
            pmt::mp ("out"),
            pmt::make_blob (d_pdu + d_frame_size_field_len,
                            d_frame_len - d_crc_len - d_frame_size_field_len));
      }
      reset();
      return (i+1) * 8;
    }
  }
  return len;
}

void
frame_acquisition_impl::reset ()
{
  if(d_whitening) {
    d_whitening->reset();
  }
  d_cnt = 0;
  d_state = SEARCHING;
  d_preamble_shift_reg.reset();
  d_sync_shift_reg.reset();
}

bool
frame_acquisition_impl::check_crc ()
{
  uint16_t crc16_c;
  uint16_t crc16_received;
  uint32_t crc32_c;
  uint32_t crc32_received;
  switch(d_crc){
    case CRC_NONE:
      return true;
    case CRC16_CCITT:
      crc16_c = crc16_ccitt(d_pdu, d_frame_len - 2);
      memcpy(&crc16_received, d_pdu + d_frame_len - 2, 2);
      crc16_received = ntohs(crc16_received);
      LOG_DEBUG("Received: 0x%02x Computed: 0x%02x", crc16_received, crc16_c);
      if(crc16_c == crc16_received) {
        return true;
      }
      return false;
    case CRC16_CCITT_REVERSED:
      crc16_c = crc16_ccitt_reversed(d_pdu, d_frame_len - 2);
      memcpy(&crc16_received, d_pdu + d_frame_len - 2, 2);
      crc16_received = ntohs(crc16_received);
      LOG_DEBUG("Received: 0x%02x Computed: 0x%02x", crc16_received, crc16_c);
      if(crc16_c == crc16_received) {
        return true;
      }
      return false;
    case CRC16_IBM:
      crc16_c = crc16_ibm(d_pdu, d_frame_len - 2);
      memcpy(&crc16_received, d_pdu + d_frame_len - 2, 2);
      crc16_received = ntohs(crc16_received);
      LOG_DEBUG("Received: 0x%02x Computed: 0x%02x", crc16_received, crc16_c);
      if(crc16_c == crc16_received) {
        return true;
      }
      return false;
    case CRC32:
      crc32_c = crc32(d_pdu, d_frame_len - 4);
      memcpy(&crc32_received, d_pdu + d_frame_len - 4, 4);
      crc32_received = ntohl(crc32_received);
      if(crc32_c == crc32_received) {
        return true;
      }
      return false;
    default:
      return false;
  }
}

} /* namespace satnogs */
} /* namespace gr */

