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

#ifndef INCLUDED_SATNOGS_FRAME_ACQUISITION_IMPL_H
#define INCLUDED_SATNOGS_FRAME_ACQUISITION_IMPL_H

#include <satnogs/shift_reg.h>
#include <satnogs/frame_acquisition.h>

namespace gr
{
namespace satnogs
{

class frame_acquisition_impl : public frame_acquisition
{

public:
  frame_acquisition_impl (variant_t variant,
                          const std::vector<uint8_t>& preamble,
                          size_t preamble_threshold,
                          const std::vector<uint8_t>& sync,
                          size_t sync_threshold,
                          size_t frame_size_field_len,
                          size_t frame_len,
                          checksum_t crc,
                          whitening::whitening_sptr descrambler,
                          size_t max_frame_len);

  ~frame_acquisition_impl ();

  // Where all the action really happens
  int
  work (int noutput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
  /**
   * Decoding FSM
   */
  typedef enum
  {
    SEARCHING,   //!< when searching for the start of the preamble
    SEARCHING_SYNC,
    DECODING_GENERIC_FRAME_LEN,
    DECODING_GOLAY24_FRAME_LEN,
    DECODING_PAYLOAD
  } decoding_state_t;

  const variant_t               d_variant;
  shift_reg                     d_preamble;
  shift_reg                     d_preamble_shift_reg;
  const size_t                  d_preamble_len;
  const size_t                  d_preamble_thrsh;
  shift_reg                     d_sync;
  shift_reg                     d_sync_shift_reg;
  const size_t                  d_sync_len;
  const size_t                  d_sync_thrsh;
  decoding_state_t              d_state;
  uint32_t                      d_cnt;
  uint32_t                      d_frame_size_field_len;
  uint32_t                      d_frame_len;
  const uint32_t                d_max_frame_len;
  const checksum_t              d_crc;
  uint32_t                      d_crc_len;
  whitening::whitening_sptr     d_whitening;
  uint8_t                       *d_pdu;


  int
  searching_preamble(const uint8_t *in, int len);

  int
  searching_sync(const uint8_t *in, int len);

  int dec_generic_frame_len(const uint8_t *in, int len);


  int dec_golay24_frame_len(const uint8_t *in, int len);

  int
  decoding(const uint8_t *in, int len);

  void
  reset();

  bool
  check_crc();
};

} // namespace satnogs
} // namespace gr

#endif /* INCLUDED_SATNOGS_FRAME_ACQUISITION_IMPL_H */

