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

#ifndef INCLUDED_SATNOGS_FRAME_ACQUISITION_H
#define INCLUDED_SATNOGS_FRAME_ACQUISITION_H

#include <satnogs/api.h>
#include <satnogs/whitening.h>
#include <gnuradio/sync_block.h>

namespace gr
{
namespace satnogs
{

/*!
 * \brief A generic frame acquisition block
 *
 * A generic frame acquisition block trying to cover a variaty of different
 * framing schemes.
 *
 * The goal of this block is to provide a unified way to acquire the frame
 * from different satellites with different but closely related framing schemes.
 * To keep the logic inside the block simple, it assumes that the received
 * bit stream is ready for framing extraction. Any bit stream coding, like
 * NRZI, Manchester, etc should be done prior this block.
 *
 * Currently the supported are:
 * - TI framing, constant frame length
 * - TI framing, variable frame length
 * - TI like framing, variable frame length with a 12-bit frame legth field
 * coded with Golay (24, 12, 8) scheme.
 *
 * The block supports also an arbitrary descrambler (if provided) using the
 * satnogs::whitening claass.
 *
 * For the CRC calculation (if any) the currently supported schemes are:
 * - CRC16-CCITT
 * - Reversed CRC16-CCITT
 * - CRC16-IBM
 * - CRC32-CCITT
 *
 * \ingroup satnogs
 *
 */
class SATNOGS_API frame_acquisition : virtual public gr::sync_block
{
public:
  typedef boost::shared_ptr<frame_acquisition> sptr;

  /**
   * Different framing schemes variants
   */
  typedef enum {
    GENERIC_CONSTANT_FRAME_LEN = 0,//!< TI CCXXX like, constant frame length
    GENERIC_VAR_FRAME_LEN,         //!< TI CCXXX like, variable frame length
    GOLAY24_CODED_FRAME_LEN        //!< TI CCXXX like, variable frame length, 12-bit frame legth field coded with Golay (24, 12, 8)
  } variant_t;

  typedef enum {
    CRC_NONE = 0,
    CRC16_CCITT,
    CRC16_CCITT_REVERSED,
    CRC16_IBM,
    CRC32
  } checksum_t;


  /**
   * A generic frame acquisition block trying to cover a variaty of different
   * framing schemes.
   *
   * The goal of this block is to provide a unified way to acquire the frame
   * from different satellites with different but closely related framing schemes.
   * To keep the logic inside the block simple, it assumes that the received
   * bit stream is ready for framing extraction. Any bit stream coding, like
   * NRZI, Manchester, etc should be done prior this block.
   *
   * @param variant the framing variant
   *
   * @param preamble the preamble should be a repeated word. Note that due to AGC
   * settling, the receiver may not receive the whole preamble. If the preamble
   * is indeed a repeated pattern, a portion of it can be given as parameter.
   * The block should be able to deal with this. However, a quite small subset
   * may lead to a larger number of false alarms.
   *
   * @param preamble_threshold the maximum number of bits that are
   * allowed to be wrong at the preamble
   *
   * @param sync the sysnchronization work following the preamble
   *
   * @param sync_threshold the maximum number of bits that are
   * allowed to be wrong at the synchronization word

   * @param frame_size_field_len the length of the field describing the frame
   * length. I most cases should be 1 or 2.
   *
   * @param frame_len if the variant dictates a constant frame length, this
   * parameter provides the length of the frame
   *
   * @param crc the CRC scheme to use
   * @param descrambler the descramble used
   * @param max_frame_len the maximum allowed frame length
   * @return a shared pointer of a frame_acquisition block
   */
  static sptr
  make(variant_t variant,
       const std::vector<uint8_t>& preamble,
       size_t preamble_threshold,
       const std::vector<uint8_t>& sync,
       size_t sync_threshold,
       size_t frame_size_field_len,
       size_t frame_len,
       checksum_t crc = CRC_NONE,
       whitening::whitening_sptr descrambler = nullptr,
       size_t max_frame_len = 2048);
};

} // namespace satnogs
} // namespace gr

#endif /* INCLUDED_SATNOGS_FRAME_ACQUISITION_H */

