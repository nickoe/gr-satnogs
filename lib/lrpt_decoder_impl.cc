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
#include "lrpt_decoder_impl.h"
#include <satnogs/log.h>
#include <fec.h>

namespace gr
{
namespace satnogs
{

lrpt_decoder::sptr
lrpt_decoder::make ()
{
  return gnuradio::get_initial_sptr (new lrpt_decoder_impl ());
}

/*
 * The private constructor
 */
lrpt_decoder_impl::lrpt_decoder_impl()
: gr::block("lrpt_decoder",
    gr::io_signature::make(0, 0, 0),
    gr::io_signature::make(0, 0, 0)),
    d_cadu_len(1020),
    d_coded_cadu_len(1020 * 2),
    d_conv_deinterl(36, 2048)
{

  message_port_register_in(pmt::mp("cadu"));
  message_port_register_in(pmt::mp("reset"));

  set_msg_handler (
      pmt::mp ("cadu"),
      boost::bind (&lrpt_decoder_impl::decode, this, _1));

  set_msg_handler (
      pmt::mp ("reset"),
      boost::bind (&lrpt_decoder_impl::reset, this, _1));

  d_vt = create_viterbi27(d_cadu_len * 8);
  if(!d_vt) {
    throw std::runtime_error("lrpt_decoder: Failed to init Viterbi decoder");
  }
  int polys[2] = {0x79, 0x5b};
  set_viterbi27_polynomial(polys);

  d_cadu = new uint8_t[d_cadu_len];
  d_coded_cadu_deinterl = new uint8_t[d_coded_cadu_len];

}

/*
 * Our virtual destructor.
 */
lrpt_decoder_impl::~lrpt_decoder_impl ()
{

  delete [] d_cadu;
  delete [] d_coded_cadu_deinterl;
}

void
lrpt_decoder_impl::decode (pmt::pmt_t m)
{
  const uint8_t *coded_cadu = (const uint8_t *)pmt::blob_data(m);
  if(pmt::blob_length(m) != d_coded_cadu_len) {
    LOG_ERROR("Wrong CADU size");
    return;
  }

  for(size_t i = 0; i < d_coded_cadu_len; i++) {
    d_coded_cadu_deinterl[i] = d_conv_deinterl.decode_byte(coded_cadu[i]);
  }
  init_viterbi27(d_vt, 0);

}

void
lrpt_decoder_impl::reset (pmt::pmt_t m)
{
  if(pmt::to_bool(m)) {
    d_conv_deinterl.reset();
  }
}


} /* namespace satnogs */
} /* namespace gr */

