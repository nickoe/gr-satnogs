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
  frame_acquisition_impl (const std::vector<uint8_t>& preamble,
                          size_t preamble_threshold,
                          const std::vector<uint8_t>& sync,
                          size_t sync_threshold);
  ~frame_acquisition_impl ();

  // Where all the action really happens
  int
  work (int noutput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
  shift_reg                     d_preamble;
  const size_t                  d_preamble_len;
  const size_t                  d_preamble_thrsh;
  shift_reg                     d_sync;
  const size_t                  d_sync_len;
  const size_t                  d_sync_thrsh;
};

} // namespace satnogs
} // namespace gr

#endif /* INCLUDED_SATNOGS_FRAME_ACQUISITION_IMPL_H */

