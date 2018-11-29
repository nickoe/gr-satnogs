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

namespace gr
{
namespace satnogs
{

frame_acquisition::sptr
frame_acquisition::make (const std::vector<uint8_t>& preamble,
                         size_t preamble_threshold,
                         const std::vector<uint8_t>& sync,
                         size_t sync_threshold)
{
  return gnuradio::get_initial_sptr (
      new frame_acquisition_impl (preamble, preamble_threshold, sync,
                                  sync_threshold));
}

frame_acquisition_impl::frame_acquisition_impl (
    const std::vector<uint8_t>& preamble, size_t preamble_threshold,
    const std::vector<uint8_t>& sync, size_t sync_threshold) :
        gr::sync_block ("frame_acquisition",
                        gr::io_signature::make (1, 1, sizeof(uint8_t)),
                        gr::io_signature::make (0, 0, 0)),
                        d_preamble(preamble.size() * 8),
                        d_preamble_len(preamble.size() * 8),
                        d_preamble_thrsh(preamble_threshold),
                        d_sync(sync.size() * 8),
                        d_sync_len(sync.size() * 8),
                        d_sync_thrsh(sync_threshold)
{
  set_output_multiple(8);
  for(uint8_t b : preamble) {
    d_preamble <<= (b >> 7);
    d_preamble <<= ((b >> 6) & 0x1);
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
    d_sync <<= ((b >> 6) & 0x1);
    d_sync <<= ((b >> 5) & 0x1);
    d_sync <<= ((b >> 4) & 0x1);
    d_sync <<= ((b >> 3) & 0x1);
    d_sync <<= ((b >> 2) & 0x1);
    d_sync <<= ((b >> 1) & 0x1);
    d_sync <<= (b & 0x1);
  }
}

/*
 * Our virtual destructor.
 */
frame_acquisition_impl::~frame_acquisition_impl ()
{
}



int
frame_acquisition_impl::work (int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
{
  const uint8_t *in = (const uint8_t *) input_items[0];

  // Do <+signal processing+>

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace satnogs */
} /* namespace gr */

