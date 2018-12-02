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
 * \brief <+description of block+>
 * \ingroup satnogs
 *
 */
class SATNOGS_API frame_acquisition : virtual public gr::sync_block
{
public:
  typedef boost::shared_ptr<frame_acquisition> sptr;

  typedef enum {
    GENERIC_CONSTANT_FRAME_LEN = 0,
    GENERIC_VAR_FRAME_LEN,
    GOLAY24_CODED_FRAME_LEN
  } variant_t;

  typedef enum {
    CRC_NONE = 0,
    CRC16_CCITT,
    CRC16_CCITT_REVERSED,
    CRC16_IBM,
    CRC32
  } checksum_t;


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

